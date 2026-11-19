//! The LC3 decoder (liblc3 `lc3.c`, decoder side): bitstream → spectrum → TNS, SNS → inverse
//! MDCT → LTPF, with packet loss concealment.

use alloc::vec;
use alloc::vec::Vec;

use crate::bits::Bits;
use crate::common::{DT_2M5, DT_5M, DT_7M5, DT_10M, MAX_NS, hr, nd, ne, nh, ns};
use crate::fastmath::ldexpf;
use crate::ltpf::{self, LtpfData, LtpfSynthesis};
use crate::mdct::mdct_inverse;
use crate::plc::PlcState;
use crate::sns::{self, SnsData};
use crate::spec;
use crate::tns::{self, TnsData};

/// `LC3_MIN_FRAME_BYTES`, `LC3_MAX_FRAME_BYTES`.
pub const MIN_FRAME_BYTES: usize = 20;
/// The largest frame, in bytes.
pub const MAX_FRAME_BYTES: usize = 400;

/// Why a decoder can't be built, or a frame can't be decoded.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error {
    /// Frame duration or sample rate not supported, or a PCM rate below the coded rate.
    Config,
    /// A frame shorter than [`MIN_FRAME_BYTES`] or longer than [`MAX_FRAME_BYTES`].
    FrameSize,
}

/// What [`Decoder::decode`] did with a frame.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Status {
    /// The frame was decoded.
    Decoded,
    /// The frame was missing or corrupt: concealed from the previous ones.
    Concealed,
}

/// `struct side_data` of `lc3.c`.
#[derive(Clone, Copy, Debug, Default)]
struct Side {
    bw: usize,
    tns: TnsData,
    pitch_present: bool,
    sns: SnsData,
    ltpf: LtpfData,
}

/// `resolve_dt` (LC3 and LC3plus frame durations, no high-resolution mode).
pub(crate) fn resolve_dt(us: u32) -> Option<usize> {
    match us {
        2500 => Some(DT_2M5),
        5000 => Some(DT_5M),
        7500 => Some(DT_7M5),
        10000 => Some(DT_10M),
        _ => None,
    }
}

/// `resolve_srate` (no high-resolution mode).
pub(crate) fn resolve_srate(hz: u32) -> Option<usize> {
    [8000, 16000, 24000, 32000, 48000]
        .iter()
        .position(|&r| r == hz)
}

/// `lc3_bwdet_get_bw`.
fn get_bw(bits: &mut Bits, sr: usize) -> Result<usize, ()> {
    let nbits = if hr(sr) {
        0
    } else {
        usize::from(sr > 0) + usize::from(sr > 1) + usize::from(sr > 3)
    };
    let bw = if nbits == 0 {
        sr
    } else {
        bits.get_bits(nbits as i32) as usize
    };
    if bw > sr { Err(()) } else { Ok(bw) }
}

/// An LC3 decoder for one channel (`struct lc3_decoder`).
#[derive(Clone, Debug)]
pub struct Decoder {
    dt: usize,
    sr: usize,
    sr_pcm: usize,
    ltpf: LtpfSynthesis,
    plc: PlcState,
    /// History ring (nh + ns), MDCT delay (nd), last shaped spectrum (ns), as the C's `x`.
    x: Vec<f32>,
    xs_off: usize,
}

impl Decoder {
    /// A decoder for frames of `frame_us` (2500, 5000, 7500 or 10000) coded at `sample_rate`
    /// (8000, 16000, 24000, 32000 or 48000 Hz), output at `pcm_rate` (≥ `sample_rate`, from the
    /// same list; liblc3's `lc3_setup_decoder`).
    pub fn new(frame_us: u32, sample_rate: u32, pcm_rate: u32) -> Result<Self, Error> {
        let dt = resolve_dt(frame_us).ok_or(Error::Config)?;
        let sr = resolve_srate(sample_rate).ok_or(Error::Config)?;
        let sr_pcm = resolve_srate(pcm_rate).ok_or(Error::Config)?;
        if sr > sr_pcm {
            return Err(Error::Config);
        }
        let (nh, ns, nd) = (nh(dt, sr_pcm), ns(dt, sr_pcm), nd(dt, sr_pcm));
        Ok(Decoder {
            dt,
            sr,
            sr_pcm,
            ltpf: LtpfSynthesis::default(),
            plc: PlcState::new(),
            x: vec![0.0; nh + ns + nd + ns],
            xs_off: nh,
        })
    }

    /// PCM samples per frame.
    pub fn frame_samples(&self) -> usize {
        ns(self.dt, self.sr_pcm)
    }

    /// (ring length nh + ns, delay offset, spectrum offset) of `x`.
    fn layout(&self) -> (usize, usize) {
        let ring = nh(self.dt, self.sr_pcm) + ns(self.dt, self.sr_pcm);
        (ring, ring + nd(self.dt, self.sr_pcm))
    }

    /// `decode` of `lc3.c`: the frame's side data, its spectrum into `xf`.
    fn read(&mut self, data: &[u8]) -> Result<Side, ()> {
        let (dt, sr) = (self.dt, self.sr);
        let nbytes = data.len() as i32;
        let (ns, ne) = (ns(dt, sr), ne(dt, sr));
        let mut bits = Bits::new(data);
        let bw = get_bw(&mut bits, sr)?;
        let spec = spec::get_side(&mut bits, dt, sr).map_err(|_| ())?;
        let tns = tns::get_data(&mut bits, dt, bw, nbytes)?;
        let pitch_present = bits.get_bit() != 0;
        let sns = sns::get_data(&mut bits)?;
        let ltpf = if pitch_present {
            ltpf::get_data(&mut bits)
        } else {
            LtpfData::default()
        };
        let xf = &mut self.x[self.xs_off..];
        spec::decode(&mut bits, dt, sr, bw, nbytes, &spec, xf)?;
        xf[ne..ns].fill(0.0);
        if bits.check() {
            Ok(Side {
                bw,
                tns,
                pitch_present,
                sns,
                ltpf,
            })
        } else {
            Err(())
        }
    }

    /// `synthesize` of `lc3.c`; `side` `None` conceals.
    fn synthesize(&mut self, side: Option<&Side>, nbytes: i32) {
        let (dt, sr, sr_pcm) = (self.dt, self.sr, self.sr_pcm);
        let (ns, ne) = (ns(dt, sr_pcm), ne(dt, sr));
        let (ring_len, xg_off) = self.layout();
        let xs = self.xs_off;
        let (ring, rest) = self.x.split_at_mut(ring_len);
        let (xd, xg) = rest.split_at_mut(xg_off - ring_len);
        match side {
            Some(side) => {
                self.plc.suspend();
                tns::synthesize(dt, side.bw, &side.tns, &mut ring[xs..]);
                sns::synthesize(dt, sr, &side.sns, &ring[xs..], xg);
                mdct_inverse(dt, sr_pcm, sr, xg, xd, &mut ring[xs..xs + ns]);
            }
            None => {
                self.plc.synthesize(dt, sr, xg, &mut ring[xs..]);
                ring[xs + ne..xs + ns].fill(0.0);
                let mut xf = [0f32; MAX_NS];
                xf[..ns].copy_from_slice(&ring[xs..xs + ns]);
                mdct_inverse(dt, sr_pcm, sr, &xf[..ns], xd, &mut ring[xs..xs + ns]);
            }
        }
        if !hr(sr) {
            let data = side.filter(|s| s.pitch_present).map(|s| &s.ltpf);
            ltpf::synthesize(dt, sr_pcm, nbytes, &mut self.ltpf, data, ring, xs);
        }
    }

    /// `complete` of `lc3.c`: advances the ring.
    fn complete(&mut self) {
        let (nh, ns) = (nh(self.dt, self.sr_pcm), ns(self.dt, self.sr_pcm));
        self.xs_off = if self.xs_off < nh {
            self.xs_off + ns
        } else {
            0
        };
    }

    /// Decodes one frame (`None`: a lost frame, concealed) into `frame_samples()` samples of
    /// float PCM in [-1, 1] (liblc3's `LC3_PCM_FORMAT_FLOAT`).
    // `fminf(fmaxf(s, -1), 1)`: NaN gives -1, where `clamp` would keep NaN
    #[allow(clippy::manual_clamp)]
    pub fn decode_f32(&mut self, frame: Option<&[u8]>, pcm: &mut [f32]) -> Result<Status, Error> {
        let status = self.run(frame)?;
        let ns = self.frame_samples();
        for (o, &s) in pcm[..ns]
            .iter_mut()
            .zip(&self.x[self.xs_off..self.xs_off + ns])
        {
            *o = ldexpf(s, -15).max(-1.0).min(1.0);
        }
        self.complete();
        Ok(status)
    }

    /// Decodes one frame (`None`: a lost frame, concealed) into `frame_samples()` 16-bit
    /// samples (liblc3's `LC3_PCM_FORMAT_S16`).
    pub fn decode(&mut self, frame: Option<&[u8]>, pcm: &mut [i16]) -> Result<Status, Error> {
        let status = self.run(frame)?;
        let ns = self.frame_samples();
        for (o, &s) in pcm[..ns]
            .iter_mut()
            .zip(&self.x[self.xs_off..self.xs_off + ns])
        {
            // The C converts with `(int)`: out of the int range it is undefined (x86: INT_MIN,
            // so a huge positive sample would come out -32768); this saturates.
            let v = if s >= 0.0 {
                (s + 0.5) as i32
            } else {
                (s - 0.5) as i32
            };
            *o = v.clamp(-32768, 32767) as i16;
        }
        self.complete();
        Ok(status)
    }

    /// The frame check, read and synthesis of `lc3_decode`.
    fn run(&mut self, frame: Option<&[u8]>) -> Result<Status, Error> {
        if let Some(f) = frame
            && !(MIN_FRAME_BYTES..=MAX_FRAME_BYTES).contains(&f.len())
        {
            return Err(Error::FrameSize);
        }
        let side = frame.and_then(|f| self.read(f).ok());
        let nbytes = frame.map_or(0, |f| f.len() as i32);
        self.synthesize(side.as_ref(), nbytes);
        Ok(if side.is_some() {
            Status::Decoded
        } else {
            Status::Concealed
        })
    }
}
