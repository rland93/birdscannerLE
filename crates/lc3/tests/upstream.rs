//! Bit-exactness of this no_std port against the unmodified crates.io `lc3` 0.1.0.

/// Deterministic xorshift64* generator (fixed seed: reproducible inputs).
struct Rng(u64);

impl Rng {
    fn new(seed: u64) -> Self {
        Self(seed.wrapping_mul(0x9E37_79B9_7F4A_7C15) | 1)
    }
    fn u64(&mut self) -> u64 {
        self.0 ^= self.0 >> 12;
        self.0 ^= self.0 << 25;
        self.0 ^= self.0 >> 27;
        self.0.wrapping_mul(0x2545_F491_4F6C_DD1D)
    }
}

const SECONDS_PER_SIGNAL: u32 = 3;

/// The test input at `rate` Hz: a 3 s 50 Hz → rate/2 exponential sine sweep at -3 dBFS, 3 s of
/// full-band white noise, 3 s of silence, 3 s of a full-scale 440 Hz square, then the sweep
/// again quieter (-30 dBFS) so the encoder also sees a level change mid-stream.
fn signal(rate: u32) -> Vec<i16> {
    let n = (rate * SECONDS_PER_SIGNAL) as usize;
    let fs = f64::from(rate);
    let sweep = |amp: f64| {
        let (f0, f1, t1) = (50.0f64, fs / 2.0, f64::from(SECONDS_PER_SIGNAL));
        let k = (f1 / f0).ln() / t1;
        (0..n).map(move |i| {
            let t = i as f64 / fs;
            let phase = 2.0 * std::f64::consts::PI * f0 * ((k * t).exp() - 1.0) / k;
            (amp * phase.sin()).round() as i16
        })
    };
    let mut rng = Rng::new(0x1c3);
    let mut out = Vec::with_capacity(5 * n);
    out.extend(sweep(23_197.0));
    out.extend((0..n).map(|_| rng.u64() as i16));
    out.extend(std::iter::repeat_n(0i16, n));
    let half_period = (fs / 880.0) as usize;
    out.extend((0..n).map(|i| {
        if (i / half_period).is_multiple_of(2) {
            i16::MAX
        } else {
            i16::MIN
        }
    }));
    out.extend(sweep(1_037.0));
    out
}

/// Every frame of `pcm` from both encoders (16-bit input), asserted equal; returns them.
fn encode_both(frame_us: u32, sr: u32, sr_pcm: u32, bytes: usize, pcm: &[i16]) -> Vec<Vec<u8>> {
    let mut ours = lc3::Encoder::new(frame_us, sr, sr_pcm).unwrap();
    let mut theirs = lc3_upstream::Encoder::new(frame_us, sr, sr_pcm).unwrap();
    let ns = ours.frame_samples();
    assert_eq!(ns, theirs.frame_samples());
    let mut frames = Vec::new();
    for (k, chunk) in pcm.chunks_exact(ns).enumerate() {
        let (mut a, mut b) = (vec![0u8; bytes], vec![0u8; bytes]);
        ours.encode(chunk, &mut a).unwrap();
        theirs.encode(chunk, &mut b).unwrap();
        assert_eq!(a, b, "{frame_us} us {sr}/{sr_pcm} Hz {bytes} B: frame {k}");
        frames.push(a);
    }
    assert!(frames.len() >= 5 * SECONDS_PER_SIGNAL as usize * 100_000 / frame_us as usize - 1);
    frames
}

/// (frame µs, coded Hz, PCM Hz, frame bytes): the firmware's candidate operating points.
const CONFIGS: [(u32, u32, u32, usize); 5] = [
    (10_000, 16_000, 16_000, 40),
    (10_000, 16_000, 32_000, 40),
    (10_000, 24_000, 32_000, 60),
    (10_000, 32_000, 32_000, 80),
    (7_500, 16_000, 16_000, 30),
];

#[test]
fn encoder_s16_bit_exact() {
    for (frame_us, sr, sr_pcm, bytes) in CONFIGS {
        encode_both(frame_us, sr, sr_pcm, bytes, &signal(sr_pcm));
    }
}

#[test]
fn encoder_f32_bit_exact() {
    for (frame_us, sr, sr_pcm, bytes) in CONFIGS {
        let pcm: Vec<f32> = signal(sr_pcm).iter().map(|&s| f32::from(s) / 32768.0).collect();
        let mut ours = lc3::Encoder::new(frame_us, sr, sr_pcm).unwrap();
        let mut theirs = lc3_upstream::Encoder::new(frame_us, sr, sr_pcm).unwrap();
        for (k, chunk) in pcm.chunks_exact(ours.frame_samples()).enumerate() {
            let (mut a, mut b) = (vec![0u8; bytes], vec![0u8; bytes]);
            ours.encode_f32(chunk, &mut a).unwrap();
            theirs.encode_f32(chunk, &mut b).unwrap();
            assert_eq!(a, b, "{frame_us} us {sr}/{sr_pcm} Hz {bytes} B: frame {k}");
        }
    }
}

/// The largest frames (48 kHz, `frame_samples()` = 480 at 10 ms): the no_std port's on-stack
/// MDCT buffers are sized for exactly these, so they must stay exact too.
#[test]
fn encoder_48k_bit_exact() {
    for (frame_us, bytes) in [(10_000, 120), (7_500, 90)] {
        let frames = encode_both(frame_us, 48_000, 48_000, bytes, &signal(48_000));
        let mut ours = lc3::Decoder::new(frame_us, 48_000, 48_000).unwrap();
        let mut theirs = lc3_upstream::Decoder::new(frame_us, 48_000, 48_000).unwrap();
        let ns = ours.frame_samples();
        let (mut a, mut b) = (vec![0i16; ns], vec![0i16; ns]);
        for (k, frame) in frames.iter().enumerate() {
            let input = (k % 29 != 28).then_some(&frame[..]);
            ours.decode(input, &mut a).unwrap();
            theirs.decode(input, &mut b).unwrap();
            assert_eq!(a, b, "{frame_us} us 48 kHz: frame {k}");
        }
    }
}

/// Round trip of (10 ms, 16 kHz coded, 32 kHz PCM, 40 B) through both decoders, with every
/// 37th frame lost and every 53rd corrupted (bit flips) so packet loss concealment runs too.
#[test]
fn decoder_bit_exact() {
    let (frame_us, sr, sr_pcm, bytes) = CONFIGS[1];
    let frames = encode_both(frame_us, sr, sr_pcm, bytes, &signal(sr_pcm));
    let mut ours = lc3::Decoder::new(frame_us, sr, sr_pcm).unwrap();
    let mut theirs = lc3_upstream::Decoder::new(frame_us, sr, sr_pcm).unwrap();
    let mut ours_f = lc3::Decoder::new(frame_us, sr, sr_pcm).unwrap();
    let mut theirs_f = lc3_upstream::Decoder::new(frame_us, sr, sr_pcm).unwrap();
    let ns = ours.frame_samples();
    let (mut a, mut b) = (vec![0i16; ns], vec![0i16; ns]);
    let (mut af, mut bf) = (vec![0f32; ns], vec![0f32; ns]);
    let mut rng = Rng::new(53);
    let (mut decoded, mut concealed) = (0, 0);
    for (k, frame) in frames.iter().enumerate() {
        let mut frame = frame.clone();
        if k % 53 == 52 {
            for _ in 0..8 {
                let bit = (rng.u64() % (8 * bytes as u64)) as usize;
                frame[bit / 8] ^= 1 << (bit % 8);
            }
        }
        let input = (k % 37 != 36).then_some(&frame[..]);
        let sa = ours.decode(input, &mut a).unwrap();
        let sb = theirs.decode(input, &mut b).unwrap();
        assert_eq!(format!("{sa:?}"), format!("{sb:?}"), "frame {k}");
        assert_eq!(a, b, "frame {k}");
        match sa {
            lc3::Status::Decoded => decoded += 1,
            lc3::Status::Concealed => concealed += 1,
        }
        ours_f.decode_f32(input, &mut af).unwrap();
        theirs_f.decode_f32(input, &mut bf).unwrap();
        let bits = |v: &[f32]| v.iter().map(|x| x.to_bits()).collect::<Vec<_>>();
        assert_eq!(bits(&af), bits(&bf), "frame {k} (f32)");
    }
    assert!(decoded > 1000 && concealed > 30, "{decoded} decoded, {concealed} concealed");
}

#[test]
fn constants_and_errors_match() {
    assert_eq!(lc3::MAX_FRAME_BYTES, lc3_upstream::MAX_FRAME_BYTES);
    assert_eq!(lc3::MIN_FRAME_BYTES, lc3_upstream::MIN_FRAME_BYTES);
    assert_eq!(lc3::Encoder::new(10_000, 32_000, 16_000).unwrap_err(), lc3::Error::Config);
    let mut enc = lc3::Encoder::new(10_000, 16_000, 16_000).unwrap();
    let pcm = vec![0i16; enc.frame_samples()];
    assert_eq!(enc.encode(&pcm, &mut [0u8; 19]), Err(lc3::Error::FrameSize));
}
