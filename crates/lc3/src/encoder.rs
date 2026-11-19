//! The LC3 encoder (liblc3 `lc3.c`, encoder side): attack detection and LTPF analysis on the
//! 16-bit signal, MDCT, band energies, bandwidth detection, SNS, TNS and spectral
//! quantization, then the bitstream.

use alloc::vec;
use alloc::vec::Vec;

use crate::attdet::{self, AttDet};
use crate::bits::Writer;
use crate::bwdet;
use crate::common::{MAX_NS, nd, ns};
use crate::decoder::{Error, MAX_FRAME_BYTES, MIN_FRAME_BYTES, resolve_dt, resolve_srate};
use crate::energy;
use crate::fastmath::ldexpf;
use crate::ltpf::{self, LtpfAnalysis, LtpfData};
use crate::mdct::mdct_forward;
use crate::sns::{self, SnsData};
use crate::spec::{self, SpecAnalysis, SpecSide};
use crate::tables::LC3_NS_2M5;
use crate::tns::{self, TnsData};

/// `struct side_data` of `lc3.c`, encoder side.
#[derive(Clone, Copy, Debug)]
struct Side {
    bw: usize,
    tns: TnsData,
    pitch_present: bool,
    ltpf: LtpfData,
    sns: SnsData,
    spec: SpecSide,
}

/// `lc3_nt`: samples of history kept for the temporal analyses (1.25 ms).
fn nt(sr: usize) -> usize {
    LC3_NS_2M5[sr] >> 1
}

/// An LC3 encoder for one channel (`struct lc3_encoder`).
#[derive(Clone, Debug)]
pub struct Encoder {
    dt: usize,
    sr: usize,
    sr_pcm: usize,
    attdet: AttDet,
    ltpf: LtpfAnalysis,
    spec: SpecAnalysis,
    /// 16-bit signal: `nt` samples of history, then the frame (the C's `xt`).
    xt: Vec<i16>,
    /// The frame as float, then its spectrum (`xs`); the MDCT delay (`xd`).
    xs: Vec<f32>,
    xd: Vec<f32>,
}

impl Encoder {
    /// An encoder for frames of `frame_us` (2500, 5000, 7500 or 10000) coded at `sample_rate`
    /// (8000, 16000, 24000, 32000 or 48000 Hz), from PCM at `pcm_rate` (≥ `sample_rate`, from
    /// the same list; liblc3's `lc3_setup_encoder`).
    pub fn new(frame_us: u32, sample_rate: u32, pcm_rate: u32) -> Result<Self, Error> {
        let dt = resolve_dt(frame_us).ok_or(Error::Config)?;
        let sr = resolve_srate(sample_rate).ok_or(Error::Config)?;
        let sr_pcm = resolve_srate(pcm_rate).ok_or(Error::Config)?;
        if sr > sr_pcm {
            return Err(Error::Config);
        }
        let ns = ns(dt, sr_pcm);
        Ok(Encoder {
            dt,
            sr,
            sr_pcm,
            attdet: AttDet::default(),
            ltpf: LtpfAnalysis::default(),
            spec: SpecAnalysis::default(),
            xt: vec![0; nt(sr_pcm) + ns],
            xs: vec![0.0; ns],
            xd: vec![0.0; nd(dt, sr_pcm)],
        })
    }

    /// PCM samples per frame.
    pub fn frame_samples(&self) -> usize {
        ns(self.dt, self.sr_pcm)
    }

    /// Encodes `frame_samples()` 16-bit samples (liblc3's `LC3_PCM_FORMAT_S16`) into a frame
    /// of `out.len()` bytes (20 to 400); samples past `frame_samples()` are ignored.
    ///
    /// # Panics
    /// If `pcm` is shorter than `frame_samples()`.
    pub fn encode(&mut self, pcm: &[i16], out: &mut [u8]) -> Result<(), Error> {
        check(out)?;
        let (nt, ns) = (nt(self.sr_pcm), self.frame_samples());
        for (i, &v) in pcm[..ns].iter().enumerate() {
            self.xt[nt + i] = v;
            self.xs[i] = f32::from(v);
        }
        self.run(out);
        Ok(())
    }

    /// Encodes `frame_samples()` float samples in [-1, 1] (liblc3's `LC3_PCM_FORMAT_FLOAT`)
    /// into a frame of `out.len()` bytes (20 to 400); samples past `frame_samples()` are
    /// ignored. Beyond [-1, 1] the analysis saturates as liblc3's does, up to magnitude 65536;
    /// from there (and for NaN) liblc3's float-to-int cast is undefined, and the frames may differ.
    ///
    /// # Panics
    /// If `pcm` is shorter than `frame_samples()`.
    pub fn encode_f32(&mut self, pcm: &[f32], out: &mut [u8]) -> Result<(), Error> {
        check(out)?;
        let (nt, ns) = (nt(self.sr_pcm), self.frame_samples());
        for (i, &v) in pcm[..ns].iter().enumerate() {
            self.xs[i] = ldexpf(v, 15);
            // `LC3_SAT16((int32_t)xs[i])`; out of the int range the C's cast is undefined
            self.xt[nt + i] = (self.xs[i] as i32).clamp(-32768, 32767) as i16;
        }
        self.run(out);
        Ok(())
    }

    /// `analyze` then `encode` of `lc3.c`.
    fn run(&mut self, out: &mut [u8]) {
        let nbytes = out.len() as i32;
        let side = self.analyze(nbytes);
        let (dt, sr) = (self.dt, self.sr);
        let mut bits = Writer::new(out);
        bwdet::put_bw(&mut bits, sr, side.bw);
        spec::put_side(&mut bits, dt, sr, &side.spec);
        tns::put_data(&mut bits, &side.tns);
        bits.put_bit(side.pitch_present);
        sns::put_data(&mut bits, &side.sns);
        if side.pitch_present {
            ltpf::put_data(&mut bits, &side.ltpf);
        }
        spec::encode(&mut bits, dt, sr, side.bw, nbytes, &side.spec, &mut self.xs);
        bits.flush();
    }

    /// `analyze` of `lc3.c`: the side data; the quantized spectrum in `xs`.
    fn analyze(&mut self, nbytes: i32) -> Side {
        let (dt, sr, sr_pcm) = (self.dt, self.sr, self.sr_pcm);
        let (nt, ns) = (nt(sr_pcm), ns(dt, sr_pcm));

        // Temporal
        let att = attdet::run(dt, sr_pcm, nbytes, &mut self.attdet, &self.xt, nt);
        let mut ltpf_data = LtpfData::default();
        let pitch_present = ltpf::analyse(dt, sr_pcm, &mut self.ltpf, &self.xt, nt, &mut ltpf_data);
        self.xt.copy_within(ns..ns + nt, 0);

        // Spectral
        let mut x = [0f32; MAX_NS];
        x[..ns].copy_from_slice(&self.xs);
        mdct_forward(dt, sr_pcm, sr, &x[..ns], &mut self.xd, &mut self.xs);
        let xf = &mut self.xs;
        let mut e = [0f32; 64];
        let nn_flag = energy::compute(dt, sr, xf, &mut e);
        if nn_flag {
            ltpf_data.active = false;
        }
        let bw = bwdet::run(dt, sr, &e);
        let sns = sns::analyze(dt, sr, &e, att, xf);
        let tns = tns::analyze(dt, bw, nn_flag, nbytes, xf);
        let nbits_other = bwdet::get_nbits(sr)
            + ltpf::get_nbits(pitch_present)
            + sns::get_nbits()
            + tns::get_nbits(&tns);
        let spec = spec::analyze(dt, sr, nbytes, nbits_other, &mut self.spec, xf);
        Side {
            bw,
            tns,
            pitch_present,
            ltpf: ltpf_data,
            sns,
            spec,
        }
    }
}

/// The frame-size check of `lc3_encode`.
fn check(out: &[u8]) -> Result<(), Error> {
    if (MIN_FRAME_BYTES..=MAX_FRAME_BYTES).contains(&out.len()) {
        Ok(())
    } else {
        Err(Error::FrameSize)
    }
}
