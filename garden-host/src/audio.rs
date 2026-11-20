//! Per-mic audio: LC3 decoding with packet-loss concealment driven by the ISO
//! sequence numbers, and a level meter.

/// At most this many frames are concealed for one gap (0.5 s at 10 ms);
/// longer outages are skipped rather than burst out at once.
pub const MAX_CONCEAL: u16 = 50;

/// Consecutive frames with a sequence number behind the last one after which
/// the stream is taken to have restarted its numbering.
const RESYNC_AFTER: u32 = 4;

/// What one [`StreamDecoder::push`] did.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Pushed {
    /// Frames decoded from received data.
    pub decoded: u32,
    /// Frames concealed (lost, empty or undecodable).
    pub concealed: u32,
    /// Frames ignored (duplicate or late sequence numbers).
    pub dropped: u32,
}

/// Decodes one mic's LC3 stream.
#[derive(Debug)]
pub struct StreamDecoder {
    pub sampling_hz: u32,
    pub frame_us: u32,
    pub octets_per_frame: u16,
    dec: lc3::Decoder,
    last_seq: Option<u16>,
    behind: u32,
    pcm: Vec<i16>,
}

impl StreamDecoder {
    pub fn new(sampling_hz: u32, frame_us: u32, octets_per_frame: u16) -> Result<Self, lc3::Error> {
        let dec = lc3::Decoder::new(frame_us, sampling_hz, sampling_hz)?;
        let pcm = vec![0; dec.frame_samples()];
        Ok(StreamDecoder {
            sampling_hz,
            frame_us,
            octets_per_frame,
            dec,
            last_seq: None,
            behind: 0,
            pcm,
        })
    }

    pub fn frame_samples(&self) -> usize {
        self.pcm.len()
    }

    /// Decodes the frame with ISO sequence number `seq` (empty: reported
    /// lost), first concealing any frames missing before it. `out` receives
    /// every PCM frame produced, in order.
    pub fn push(&mut self, seq: u16, frame: &[u8], mut out: impl FnMut(&[i16])) -> Pushed {
        let mut done = Pushed::default();
        if let Some(last) = self.last_seq {
            let delta = seq.wrapping_sub(last);
            if delta == 0 || delta >= 0x8000 {
                self.behind += 1;
                if self.behind <= RESYNC_AFTER {
                    done.dropped = 1;
                    return done;
                }
            } else {
                for _ in 0..(delta - 1).min(MAX_CONCEAL) {
                    self.conceal();
                    out(&self.pcm);
                    done.concealed += 1;
                }
            }
        }
        self.behind = 0;
        self.last_seq = Some(seq);
        if !frame.is_empty()
            && self.dec.decode(Some(frame), &mut self.pcm) == Ok(lc3::Status::Decoded)
        {
            done.decoded += 1;
        } else {
            self.conceal();
            done.concealed += 1;
        }
        out(&self.pcm);
        done
    }

    fn conceal(&mut self) {
        if self.dec.decode(None, &mut self.pcm).is_err() {
            self.pcm.fill(0);
        }
    }
}

/// Signal level over one meter window, in dB relative to full scale.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Level {
    pub rms_dbfs: f32,
    pub peak_dbfs: f32,
}

/// The floor reported for digital silence.
pub const SILENCE_DBFS: f32 = -96.0;

fn dbfs(x: f64) -> f32 {
    if x <= 0.0 {
        SILENCE_DBFS
    } else {
        (20.0 * (x / 32768.0).log10()).max(f64::from(SILENCE_DBFS)) as f32
    }
}

/// RMS and peak over windows of a fixed number of samples (100 ms).
#[derive(Debug)]
pub struct Meter {
    window: usize,
    n: usize,
    sum_sq: f64,
    peak: u16,
}

impl Meter {
    pub fn new(sampling_hz: u32) -> Meter {
        Meter {
            window: (sampling_hz / 10).max(1) as usize,
            n: 0,
            sum_sq: 0.0,
            peak: 0,
        }
    }

    /// Adds `pcm`; returns the level each time a window completes.
    pub fn feed(&mut self, pcm: &[i16]) -> Option<Level> {
        let mut level = None;
        for &s in pcm {
            self.sum_sq += f64::from(s) * f64::from(s);
            self.peak = self.peak.max(s.unsigned_abs());
            self.n += 1;
            if self.n == self.window {
                level = Some(Level {
                    rms_dbfs: dbfs((self.sum_sq / self.n as f64).sqrt()),
                    peak_dbfs: dbfs(f64::from(self.peak)),
                });
                self.n = 0;
                self.sum_sq = 0.0;
                self.peak = 0;
            }
        }
        level
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tone(enc: &mut lc3::Encoder, octets: usize, n: usize) -> Vec<Vec<u8>> {
        let ns = enc.frame_samples();
        (0..n)
            .map(|f| {
                let pcm: Vec<i16> = (0..ns)
                    .map(|i| {
                        let t = (f * ns + i) as f32 / 16_000.0;
                        (8000.0 * (2.0 * std::f32::consts::PI * 440.0 * t).sin()) as i16
                    })
                    .collect();
                let mut out = vec![0u8; octets];
                enc.encode(&pcm, &mut out).unwrap();
                out
            })
            .collect()
    }

    #[test]
    fn gaps_duplicates_and_wraparound() {
        let mut enc = lc3::Encoder::new(10_000, 16_000, 16_000).unwrap();
        let frames = tone(&mut enc, 40, 6);
        let mut d = StreamDecoder::new(16_000, 10_000, 40).unwrap();
        let mut samples = 0;
        let mut count = |p: &[i16]| samples += p.len();
        assert_eq!(
            d.push(0xFFFE, &frames[0], &mut count),
            Pushed {
                decoded: 1,
                ..Default::default()
            }
        );
        // 0xFFFF lost: wraps to 0, one frame concealed.
        assert_eq!(
            d.push(0x0000, &frames[1], &mut count),
            Pushed {
                decoded: 1,
                concealed: 1,
                dropped: 0
            }
        );
        assert_eq!(
            d.push(0x0000, &frames[1], &mut count),
            Pushed {
                dropped: 1,
                ..Default::default()
            }
        );
        assert_eq!(
            d.push(0x0001, &[], &mut count),
            Pushed {
                concealed: 1,
                ..Default::default()
            }
        );
        // A corrupt frame (too short for LC3) is concealed too.
        assert_eq!(
            d.push(0x0002, &[1, 2, 3], &mut count),
            Pushed {
                concealed: 1,
                ..Default::default()
            }
        );
        // A huge gap conceals at most MAX_CONCEAL frames.
        let p = d.push(0x1000, &frames[2], &mut count);
        assert_eq!(
            p,
            Pushed {
                decoded: 1,
                concealed: u32::from(MAX_CONCEAL),
                dropped: 0
            }
        );
        assert_eq!(samples, 160 * (1 + 2 + 1 + 1 + 51));
    }

    #[test]
    fn renumbered_stream_resyncs() {
        let mut enc = lc3::Encoder::new(10_000, 16_000, 16_000).unwrap();
        let frames = tone(&mut enc, 40, 1);
        let mut d = StreamDecoder::new(16_000, 10_000, 40).unwrap();
        let mut n = 0;
        d.push(5000, &frames[0], |_| n += 1);
        for seq in 0..RESYNC_AFTER as u16 {
            assert_eq!(d.push(seq, &frames[0], |_| n += 1).dropped, 1);
        }
        assert_eq!(
            d.push(RESYNC_AFTER as u16, &frames[0], |_| n += 1).decoded,
            1
        );
        assert_eq!(n, 2);
    }

    #[test]
    fn meter_reports_every_100_ms() {
        let mut m = Meter::new(16_000);
        assert_eq!(m.feed(&[0; 1599]), None);
        let l = m.feed(&[0; 1]).unwrap();
        assert_eq!(l.rms_dbfs, SILENCE_DBFS);
        let full: Vec<i16> = (0..1600)
            .map(|i| if i % 2 == 0 { 32767 } else { -32768 })
            .collect();
        let l = m.feed(&full).unwrap();
        assert!(l.rms_dbfs > -0.01 && l.peak_dbfs.abs() < 0.01, "{l:?}");
        let half = vec![16384i16; 1600];
        let l = m.feed(&half).unwrap();
        assert!((l.rms_dbfs + 6.02).abs() < 0.01, "{l:?}");
    }
}
