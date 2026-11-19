//! Bitstream reader and writer (liblc3 `bits.c`/`bits.h`): arithmetic-coded symbols at the
//! front of the frame, plain bits at the back through a 32-bit accumulator.

/// `struct lc3_ac_model`: 17 symbols, `[low, range]` each.
pub(crate) type AcModel = [[u16; 2]; 17];

/// `LC3_ACCU_BITS`.
const ACCU_BITS: i32 = 32;

/// `lc3_bits_t` in `LC3_BITS_MODE_READ`.
#[derive(Debug)]
pub(crate) struct Bits<'a> {
    buf: &'a [u8],
    /// Arithmetic decoder: `low`, `range`, sticky `error`.
    low: u32,
    range: u32,
    error: bool,
    /// Accumulator: value, bits consumed, bits read past the start.
    v: u32,
    n: i32,
    nover: i32,
    /// Forward (arithmetic) and backward (plain bits) positions.
    p_fw: usize,
    p_bw: usize,
}

impl<'a> Bits<'a> {
    /// `lc3_setup_bits(LC3_BITS_MODE_READ)`.
    pub(crate) fn new(buf: &'a [u8]) -> Self {
        let mut b = Bits {
            buf,
            low: 0,
            range: 0xff_ffff,
            error: false,
            v: 0,
            n: ACCU_BITS,
            nover: 0,
            p_fw: 0,
            p_bw: buf.len(),
        };
        b.low = b.ac_get() << 16;
        b.low |= b.ac_get() << 8;
        b.low |= b.ac_get();
        b.accu_load();
        b
    }

    /// `ac_get`: the next byte from the front, 0 past the end.
    fn ac_get(&mut self) -> u32 {
        if self.p_fw < self.buf.len() {
            self.p_fw += 1;
            u32::from(self.buf[self.p_fw - 1])
        } else {
            0
        }
    }

    /// `accu_load`: refills the accumulator from the back.
    fn accu_load(&mut self) {
        let nbytes = (self.n >> 3).min(self.p_bw as i32);
        self.n -= 8 * nbytes;
        for _ in 0..nbytes {
            self.p_bw -= 1;
            self.v >>= 8;
            self.v |= u32::from(self.buf[self.p_bw]) << (ACCU_BITS - 8);
        }
        if self.n >= 8 {
            self.nover = (self.nover + self.n).min(ACCU_BITS);
            // The C shifts by 32 here once the frame is exhausted (undefined; x86 keeps
            // `v`, ARM clears it): such a frame fails `check` and is concealed either way.
            self.v = self.v.checked_shr(self.n as u32).unwrap_or(0);
            self.n = 0;
        }
    }

    /// `lc3_get_bits`: `n` (1 to 25) bits, LSB first.
    pub(crate) fn get_bits(&mut self, n: i32) -> u32 {
        if self.n + n <= ACCU_BITS {
            let v = (self.v >> self.n) & ((1u32 << n) - 1);
            self.n += n;
            v
        } else {
            self.get_bits_generic(n)
        }
    }

    /// `lc3_get_bit`.
    pub(crate) fn get_bit(&mut self) -> u32 {
        self.get_bits(1)
    }

    /// `lc3_get_bits_generic`: a read across an accumulator refill.
    fn get_bits_generic(&mut self, n: i32) -> u32 {
        self.accu_load();
        let n1 = (ACCU_BITS - self.n).min(n);
        let mut v = (self.v >> self.n) & ((1u32 << n1) - 1);
        self.n += n1;
        let n2 = n - n1;
        if n2 != 0 {
            self.accu_load();
            v |= ((self.v >> self.n) & ((1u32 << n2) - 1)) << n1;
            self.n += n2;
        }
        v
    }

    /// `lc3_get_symbol`: one arithmetic-coded symbol of `model`.
    pub(crate) fn get_symbol(&mut self, model: &AcModel) -> u32 {
        let range = (self.range >> 10) & 0xffff;
        self.error |= self.low >= (range << 10);
        if self.error {
            self.low = 0;
        }
        let low = |s: usize| range * u32::from(model[s][0]);
        let mut s = 16usize;
        if self.low < low(s) {
            s >>= 1;
            s = if self.low < low(s) { s - 4 } else { s + 4 };
            s = if self.low < low(s) { s - 2 } else { s + 2 };
            s = if self.low < low(s) { s - 1 } else { s + 1 };
            s -= usize::from(self.low < low(s));
        }
        self.low -= low(s);
        self.range = range * u32::from(model[s][1]);
        if self.range < 0x10000 {
            self.ac_read_renorm();
        }
        s as u32
    }

    /// `lc3_ac_read_renorm`.
    fn ac_read_renorm(&mut self) {
        while self.range < 0x10000 {
            self.low = ((self.low << 8) | self.ac_get()) & 0xff_ffff;
            self.range <<= 8;
        }
    }

    /// `get_bits_left` (read mode): may be negative once the two ends crossed.
    fn bits_left(&self) -> i32 {
        let end = self.p_bw as i32 + ACCU_BITS / 8;
        let start = self.p_fw as i32 - 24 / 8;
        let range_bits = 32 - self.range.leading_zeros() as i32;
        8 * (end - start) - (self.n + self.nover + 26 - range_bits)
    }

    /// `lc3_get_bits_left`.
    pub(crate) fn get_bits_left(&self) -> i32 {
        self.bits_left().max(0)
    }

    /// `lc3_check_bits`: whether the frame was read within its size, without AC error.
    pub(crate) fn check(&self) -> bool {
        self.bits_left() >= 0 && !self.error
    }
}

/// `lc3_bits_t` in `LC3_BITS_MODE_WRITE`.
#[derive(Debug)]
pub(crate) struct Writer<'a> {
    buf: &'a mut [u8],
    /// Arithmetic coder: `low`, `range`, `cache` (-1: none), `carry`, `carry_count`.
    low: u32,
    range: u32,
    cache: i32,
    carry: u32,
    carry_count: i32,
    /// Accumulator: value, bits held.
    v: u32,
    n: i32,
    p_fw: usize,
    p_bw: usize,
}

impl<'a> Writer<'a> {
    /// `lc3_setup_bits(LC3_BITS_MODE_WRITE)` over the whole frame.
    pub(crate) fn new(buf: &'a mut [u8]) -> Self {
        let len = buf.len();
        Writer {
            buf,
            low: 0,
            range: 0xff_ffff,
            cache: -1,
            carry: 0,
            carry_count: 0,
            v: 0,
            n: 0,
            p_fw: 0,
            p_bw: len,
        }
    }

    /// `ac_get_range_bits`.
    fn range_bits(&self) -> i32 {
        32 - self.range.leading_zeros() as i32
    }

    /// `lc3_get_bits_left` (write mode).
    pub(crate) fn get_bits_left(&self) -> i32 {
        let n = self.p_bw as i32 - self.p_fw as i32;
        let pending = 26 - self.range_bits() + (i32::from(self.cache >= 0) + self.carry_count) * 8;
        (8 * n - (self.n + pending)).max(0)
    }

    /// `accu_flush`.
    fn accu_flush(&mut self) {
        let nbytes = (self.n >> 3).min((self.p_bw as i32 - self.p_fw as i32).max(0));
        self.n -= 8 * nbytes;
        for _ in 0..nbytes {
            self.p_bw -= 1;
            self.buf[self.p_bw] = self.v as u8;
            self.v >>= 8;
        }
        if self.n >= 8 {
            self.n = 0;
        }
    }

    /// `ac_put`: a byte at the front, dropped past the end.
    fn ac_put(&mut self, byte: i32) {
        if self.p_fw < self.buf.len() {
            self.buf[self.p_fw] = byte as u8;
            self.p_fw += 1;
        }
    }

    /// `ac_shift`.
    fn ac_shift(&mut self) {
        if self.low < 0xff_0000 || self.carry != 0 {
            if self.cache >= 0 {
                self.ac_put(self.cache + self.carry as i32);
            }
            while self.carry_count > 0 {
                self.ac_put(if self.carry != 0 { 0x00 } else { 0xff });
                self.carry_count -= 1;
            }
            self.cache = (self.low >> 16) as i32;
            self.carry = 0;
        } else {
            self.carry_count += 1;
        }
        self.low = (self.low << 8) & 0xff_ffff;
    }

    /// `ac_terminate`.
    fn ac_terminate(&mut self) {
        let mut nbits = 25 - self.range_bits();
        let mut mask = 0xff_ffffu32 >> nbits;
        let mut val = self.low + mask;
        let high = self.low + self.range;
        let over_val = val >> 24 != 0;
        let over_high = high >> 24 != 0;
        val = (val & 0xff_ffff) & !mask;
        let high = high & 0xff_ffff;
        if over_val == over_high {
            if val + mask >= high {
                nbits += 1;
                mask >>= 1;
                val = ((self.low + mask) & 0xff_ffff) & !mask;
            }
            self.carry |= u32::from(val < self.low);
        }
        self.low = val;
        while nbits > 8 {
            self.ac_shift();
            nbits -= 8;
        }
        self.ac_shift();
        let mut end_val = self.cache >> (8 - nbits);
        if self.carry_count != 0 {
            self.ac_put(self.cache);
            while self.carry_count > 1 {
                self.ac_put(0xff);
                self.carry_count -= 1;
            }
            end_val = if nbits < 8 { 0 } else { 0xff };
        }
        if self.p_fw < self.buf.len() {
            let b = &mut self.buf[self.p_fw];
            *b &= (0xffu32 >> nbits) as u8;
            *b |= (end_val << (8 - nbits)) as u8;
        }
    }

    /// `lc3_flush_bits`: zero bits up to the arithmetic part, then both ends written.
    pub(crate) fn flush(mut self) {
        let nleft = self.p_bw as i32 - self.p_fw as i32;
        let mut n = 8 * nleft - self.n;
        while n > 0 {
            self.put_bits(0, n.min(32));
            n -= 32;
        }
        self.accu_flush();
        self.ac_terminate();
    }

    /// `lc3_put_bits`: `n` (1 to 32) bits, LSB first.
    pub(crate) fn put_bits(&mut self, v: u32, n: i32) {
        if self.n + n <= ACCU_BITS {
            self.v |= v << self.n;
            self.n += n;
        } else {
            self.put_bits_generic(v, n);
        }
    }

    /// `lc3_put_bit`.
    pub(crate) fn put_bit(&mut self, v: bool) {
        self.put_bits(u32::from(v), 1);
    }

    /// `lc3_put_bits_generic`: a write across an accumulator flush.
    fn put_bits_generic(&mut self, v: u32, n: i32) {
        let n1 = (ACCU_BITS - self.n).min(n);
        if n1 != 0 {
            self.v |= v << self.n;
            self.n = ACCU_BITS;
        }
        self.accu_flush();
        self.v = v.checked_shr(n1 as u32).unwrap_or(0);
        self.n = n - n1;
    }

    /// `lc3_put_symbol`: symbol `s` of `model`.
    pub(crate) fn put_symbol(&mut self, model: &AcModel, s: usize) {
        let range = self.range >> 10;
        self.low += range * u32::from(model[s][0]);
        self.range = range * u32::from(model[s][1]);
        self.carry |= self.low >> 24;
        self.low &= 0xff_ffff;
        while self.range < 0x10000 {
            self.ac_shift();
            self.range <<= 8;
        }
    }
}
