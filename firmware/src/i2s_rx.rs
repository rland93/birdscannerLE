//! Receive-only I2S master for the nRF5340 application core.
//!
//! embassy-nrf's I2S driver only offers 8/16/24-bit half-frames. I2S MEMS
//! microphones (INMP441, ICS-43434, SPH0645, ...) expect 32 SCK cycles per
//! slot (64 per frame), which on the nRF5340 needs `SWIDTH = 32Bit`. This
//! driver programs the peripheral directly and streams stereo frames into a
//! ring of EasyDMA buffers.
//!
//! Clocking: MCK = 32 MHz / 16 = 2 MHz from PCLK32M, LRCK = MCK / 64 =
//! 31 250 Hz, SCK = 64 × LRCK = 2 MHz. Integer dividers only, so no
//! fractional-divider jitter, and both clocks sit inside the supported range
//! of the common I2S MEMS microphones.

use core::future::poll_fn;
use core::sync::atomic::{Ordering, compiler_fence};
use core::task::Poll;

use embassy_nrf::Peri;
use embassy_nrf::gpio::{Flex, OutputDrive, Pin, Pull};
use embassy_nrf::interrupt::typelevel::{self, Binding, Interrupt as _};
use embassy_nrf::pac::i2s::vals;
use embassy_nrf::pac::{self, shared::regs::Psel};
use embassy_sync::waitqueue::AtomicWaker;

/// LRCK frequency produced by the clock configuration below.
pub const SAMPLE_RATE_HZ: u32 = 2_000_000 / 64;
/// Stereo frames per DMA block (16.4 ms at 31.25 kHz).
pub const FRAMES_PER_BLOCK: usize = 512;
/// 32-bit words per DMA block: one per slot.
const WORDS_PER_BLOCK: usize = FRAMES_PER_BLOCK * 2;
/// Blocks in the DMA ring. With three, a completed block stays untouched for
/// a full block period after the next one completes.
const BLOCKS: usize = 3;
/// PSEL value with the CONNECT bit set to "disconnected".
const DISCONNECTED: Psel = Psel(1 << 31);

/// One stereo frame as laid out by EasyDMA: left slot, then right slot.
/// Each slot is the raw 32-bit half-frame, MSB-first, i.e. a Q31 sample.
#[derive(Clone, Copy, Default)]
#[repr(C)]
pub struct Frame {
    pub left: i32,
    pub right: i32,
}

pub type Block = [Frame; FRAMES_PER_BLOCK];
pub type DmaBuffers = [Block; BLOCKS];

const _: () = assert!(size_of::<Block>() == WORDS_PER_BLOCK * 4);

static WAKER: AtomicWaker = AtomicWaker::new();

pub struct InterruptHandler;

impl typelevel::Handler<typelevel::I2S0> for InterruptHandler {
    unsafe fn on_interrupt() {
        let r = pac::I2S0;
        if r.events_rxptrupd().read() != 0 {
            r.intenclr().write(|w| w.set_rxptrupd(true));
            WAKER.wake();
        }
    }
}

pub struct I2sRx<'d> {
    buffers: *mut Block,
    /// Block EasyDMA is currently writing.
    filling: usize,
    /// Block whose address is latched in RXD.PTR for the next transfer.
    queued: usize,
    _sck: Flex<'d>,
    _lrck: Flex<'d>,
    _sdin: Flex<'d>,
}

impl<'d> I2sRx<'d> {
    /// Configure I2S0 as clock master and start receiving.
    ///
    /// `sdin` gets the internal pull-down so the slot the microphone does not
    /// drive (its DOUT is tri-stated there) reads as zero instead of noise.
    ///
    /// embassy-nrf has no I2S0 peripheral singleton for the nRF5340, so
    /// exclusivity rests on the interrupt binding: only one `I2sRx` may exist.
    pub async fn start(
        _irq: impl Binding<typelevel::I2S0, InterruptHandler> + 'd,
        sck: Peri<'d, impl Pin>,
        lrck: Peri<'d, impl Pin>,
        sdin: Peri<'d, impl Pin>,
        buffers: &'d mut DmaBuffers,
    ) -> Self {
        let r = pac::I2S0;

        let psel_sck = sck.psel_bits();
        let psel_lrck = lrck.psel_bits();
        let psel_sdin = sdin.psel_bits();
        let mut sck = Flex::new(sck);
        sck.set_as_output(OutputDrive::Standard);
        let mut lrck = Flex::new(lrck);
        lrck.set_as_output(OutputDrive::Standard);
        let mut sdin = Flex::new(sdin);
        sdin.set_as_input(Pull::Down);

        let c = r.config();
        c.mode().write(|w| w.set_mode(vals::Mode::Master));
        c.rxen().write(|w| w.set_rxen(true));
        c.txen().write(|w| w.set_txen(false));
        c.clkconfig().write(|w| {
            w.set_clksrc(vals::Clksrc::Pclk32m);
            w.set_bypass(false);
        });
        c.mcken().write(|w| w.set_mcken(true));
        c.mckfreq().write(|w| w.set_mckfreq(vals::Mckfreq::_32mdiv16));
        c.ratio().write(|w| w.set_ratio(vals::Ratio::_64x));
        c.swidth().write(|w| w.set_swidth(vals::Swidth::_32bit));
        c.align().write(|w| w.set_align(vals::Align::Left));
        c.format().write(|w| w.set_format(vals::Format::I2s));
        c.channels().write(|w| w.set_channels(vals::Channels::Stereo));

        let psel = r.psel();
        psel.mck().write_value(DISCONNECTED);
        psel.sdout().write_value(DISCONNECTED);
        psel.sck().write_value(psel_sck);
        psel.lrck().write_value(psel_lrck);
        psel.sdin().write_value(psel_sdin);

        let buffers = buffers.as_mut_ptr();
        r.rxd().ptr().write_value(buffers as u32);
        r.rxtxd().maxcnt().write(|w| w.set_maxcnt(WORDS_PER_BLOCK as u16));

        r.events_rxptrupd().write_value(0);
        r.events_stopped().write_value(0);
        r.intenclr().write(|w| {
            w.set_rxptrupd(true);
            w.set_txptrupd(true);
            w.set_stopped(true);
            w.set_framestart(true);
        });
        r.intenset().write(|w| w.set_rxptrupd(true));
        typelevel::I2S0::unpend();
        unsafe { typelevel::I2S0::enable() };

        let mut this = Self {
            buffers,
            filling: 0,
            queued: 0,
            _sck: sck,
            _lrck: lrck,
            _sdin: sdin,
        };

        compiler_fence(Ordering::SeqCst);
        r.enable().write(|w| w.set_enable(true));
        r.tasks_start().write_value(1);

        // START latches RXD.PTR (block 0) immediately; queue block 1 behind it.
        wait_rxptrupd().await;
        this.queue(1);
        this
    }

    /// Wait for the next completed block of stereo frames.
    ///
    /// The returned block stays valid until the next call; EasyDMA does not
    /// touch it again until two further blocks have been latched.
    pub async fn next_block(&mut self) -> &Block {
        wait_rxptrupd().await;
        // The queued block was just latched and is now being filled, so the
        // one before it is complete.
        let done = self.filling;
        self.filling = self.queued;
        self.queue((self.queued + 1) % BLOCKS);
        compiler_fence(Ordering::SeqCst);
        // SAFETY: `done` is neither being written (`filling`) nor latched for
        // the next transfer (`queued`), and the shared borrow ends before
        // `self` can be borrowed again to advance the ring.
        unsafe { &*self.buffers.add(done) }
    }

    fn queue(&mut self, block: usize) {
        self.queued = block;
        // SAFETY: `block < BLOCKS`; only the address is computed.
        let ptr = unsafe { self.buffers.add(block) };
        pac::I2S0.rxd().ptr().write_value(ptr as u32);
    }
}

impl Drop for I2sRx<'_> {
    fn drop(&mut self) {
        let r = pac::I2S0;
        typelevel::I2S0::disable();
        r.intenclr().write(|w| w.set_rxptrupd(true));
        r.events_stopped().write_value(0);
        r.tasks_stop().write_value(1);
        while r.events_stopped().read() == 0 {}
        r.events_stopped().write_value(0);
        r.enable().write(|w| w.set_enable(false));
        let psel = r.psel();
        psel.sck().write_value(DISCONNECTED);
        psel.lrck().write_value(DISCONNECTED);
        psel.sdin().write_value(DISCONNECTED);
        compiler_fence(Ordering::SeqCst);
    }
}

async fn wait_rxptrupd() {
    let r = pac::I2S0;
    poll_fn(|cx| {
        WAKER.register(cx.waker());
        if r.events_rxptrupd().read() != 0 {
            r.events_rxptrupd().write_value(0);
            r.intenset().write(|w| w.set_rxptrupd(true));
            Poll::Ready(())
        } else {
            Poll::Pending
        }
    })
    .await;
}
