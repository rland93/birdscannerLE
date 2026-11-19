//! Record the microphone PCM stream that the nrf53bird firmware publishes on
//! RTT up channel "pcm" and write it to a WAV file.
//!
//! The firmware must already be running; this attaches without resetting or
//! halting the target. Stop any other probe-rs session first.

use std::path::PathBuf;
use std::thread::sleep;
use std::time::{Duration, Instant};

use anyhow::{Context, Result, bail};
use clap::Parser;
use probe_rs::probe::WireProtocol;
use probe_rs::probe::list::Lister;
use probe_rs::rtt::{Rtt, ScanRegion};
use probe_rs::{Permissions, Session};

/// Must match the firmware (`i2s_rx::SAMPLE_RATE_HZ`, `FRAMES_PER_BLOCK`, PCM framing).
const SAMPLE_RATE_HZ: u32 = 31_250;
const SAMPLES_PER_FRAME: usize = 512;
const MAGIC: [u8; 2] = [0xA5, 0x5A];
const FRAME_BYTES: usize = 4 + SAMPLES_PER_FRAME * 2;

#[derive(Parser)]
#[command(about = "Capture nrf53bird microphone audio over RTT into a WAV file")]
struct Args {
    /// Output WAV file.
    output: PathBuf,
    /// Recording length in seconds.
    #[arg(short, long, default_value_t = 5.0)]
    seconds: f64,
    /// probe-rs chip name.
    #[arg(long, default_value = "nRF5340_xxAA_APPONLY")]
    chip: String,
    /// SWD clock in kHz. At the J-Link OB's default clock RTT can't keep up
    /// with the 62.5 kB/s stream; 2 MHz is the fastest this probe accepts.
    #[arg(long, default_value_t = 2000)]
    speed: u32,
}

fn main() -> Result<()> {
    let args = Args::parse();

    let probes = Lister::new().list_all();
    let info = probes.first().context("no debug probe found")?;
    let mut probe = info.open().context("opening probe")?;
    probe.select_protocol(WireProtocol::Swd).context("selecting SWD")?;
    probe.set_speed(args.speed).context("setting SWD speed")?;
    let mut session: Session = probe
        .attach(args.chip.as_str(), Permissions::default())
        .context("attaching to target")?;
    let mut core = session.core(0).context("selecting application core")?;

    let mut rtt = Rtt::attach_region(&mut core, &ScanRegion::Ram).context("RTT control block not found; is the firmware running?")?;
    let channel = rtt
        .up_channels()
        .iter()
        .position(|c| c.name() == Some("pcm"))
        .context("firmware has no \"pcm\" RTT channel")?;
    let pcm = &mut rtt.up_channels()[channel];

    // Discard whatever queued up before we attached so the recording starts now.
    let mut buf = vec![0u8; 64 * 1024];
    while pcm.read(&mut core, &mut buf)? > 0 {}

    let mut parser = FrameParser::default();
    let want_samples = (args.seconds * f64::from(SAMPLE_RATE_HZ)) as usize;
    let mut samples: Vec<i16> = Vec::with_capacity(want_samples + SAMPLES_PER_FRAME);
    let started = Instant::now();
    let mut last_data = Instant::now();
    eprintln!("recording {:.1} s at {SAMPLE_RATE_HZ} Hz ...", args.seconds);
    while samples.len() < want_samples {
        let n = pcm.read(&mut core, &mut buf)?;
        if n == 0 {
            if last_data.elapsed() > Duration::from_secs(2) {
                bail!("no PCM data for 2 s: is a microphone detected? (see the defmt log)");
            }
            sleep(Duration::from_millis(1));
            continue;
        }
        last_data = Instant::now();
        parser.feed(&buf[..n], &mut samples);
    }
    let wall = started.elapsed().as_secs_f64();
    samples.truncate(want_samples);

    let spec = hound::WavSpec {
        channels: 1,
        sample_rate: SAMPLE_RATE_HZ,
        bits_per_sample: 16,
        sample_format: hound::SampleFormat::Int,
    };
    let mut wav = hound::WavWriter::create(&args.output, spec)?;
    for &s in &samples {
        wav.write_sample(s)?;
    }
    wav.finalize()?;

    let peak = samples.iter().map(|s| s.unsigned_abs()).max().unwrap_or(0);
    let rms = (samples.iter().map(|&s| f64::from(s).powi(2)).sum::<f64>() / samples.len() as f64).sqrt();
    let dbfs = |x: f64| 20.0 * (x.max(1e-9) / 32768.0).log10();
    eprintln!(
        "wrote {} ({} samples, {:.2} s audio in {:.2} s wall)",
        args.output.display(),
        samples.len(),
        samples.len() as f64 / f64::from(SAMPLE_RATE_HZ),
        wall
    );
    eprintln!("level: rms {:.1} dBFS, peak {:.1} dBFS", dbfs(rms), dbfs(f64::from(peak)));
    if parser.lost_frames > 0 || parser.resyncs > 0 {
        eprintln!(
            "warning: {} frames lost (zero-filled), {} resyncs; raise --speed or close other probe users",
            parser.lost_frames, parser.resyncs
        );
    }
    Ok(())
}

/// Reassembles `[A5 5A seq_lo seq_hi][512 × i16 LE]` frames from the byte stream.
#[derive(Default)]
struct FrameParser {
    pending: Vec<u8>,
    next_seq: Option<u16>,
    lost_frames: u64,
    resyncs: u64,
}

impl FrameParser {
    fn feed(&mut self, bytes: &[u8], out: &mut Vec<i16>) {
        self.pending.extend_from_slice(bytes);
        let mut pos = 0;
        while self.pending.len() - pos >= FRAME_BYTES {
            let frame = &self.pending[pos..pos + FRAME_BYTES];
            if frame[..2] != MAGIC {
                // Firmware writes whole frames or nothing, so this only happens
                // if we attached mid-frame. Slide forward to the next magic.
                self.resyncs += 1;
                pos += 1;
                while self.pending.len() - pos >= 2 && self.pending[pos..pos + 2] != MAGIC {
                    pos += 1;
                }
                continue;
            }
            let seq = u16::from_le_bytes([frame[2], frame[3]]);
            if let Some(expected) = self.next_seq {
                let gap = seq.wrapping_sub(expected);
                if gap != 0 {
                    // Firmware skipped frames because its RTT buffer was full.
                    self.lost_frames += u64::from(gap);
                    out.resize(out.len() + usize::from(gap) * SAMPLES_PER_FRAME, 0);
                }
            }
            self.next_seq = Some(seq.wrapping_add(1));
            out.extend(frame[4..].chunks_exact(2).map(|b| i16::from_le_bytes([b[0], b[1]])));
            pos += FRAME_BYTES;
        }
        self.pending.drain(..pos);
    }
}
