//! Microphone pipeline: I2S capture, level metering, VU LEDs, the RTT PCM tap
//! and, while an LE Audio stream is active, LC3 encoding into ISO SDUs.

use defmt::{info, warn};
use embassy_nrf::gpio::{Level, Output};
use embassy_time::Instant;
use rtt_target::{ChannelMode, UpChannel};

use crate::audio::{self, StreamConfig};
use crate::i2s_rx::{FRAMES_PER_BLOCK, I2sRx};
use crate::meter::{Report, Slot};

/// Blocks per log/report window (500 ms).
const BLOCKS_PER_WINDOW: u32 = 50;
/// Block peak (dBFS) needed to light LED1..LED4. A quiet room peaks around
/// -55 dBFS, so all LEDs stay dark until there is actual sound.
const VU_THRESHOLDS_DBFS: [f32; 4] = [-50.0, -40.0, -30.0, -20.0];

/// PCM frame on RTT channel 1: magic, little-endian sequence number, then
/// FRAMES_PER_BLOCK little-endian i16 samples. Must match `mic-capture`.
const PCM_MAGIC: [u8; 2] = [0xA5, 0x5A];
const PCM_FRAME_BYTES: usize = 4 + FRAMES_PER_BLOCK * 2;

#[derive(Clone, Copy, PartialEq, Eq, defmt::Format)]
enum Which {
    Left,
    Right,
}

/// LC3 encoder bound to one CIS.
struct Encoding {
    encoder: lc3::Encoder,
    cfg: StreamConfig,
    seq: u16,
    dropped: u16,
    frame: [u8; lc3::MAX_FRAME_BYTES],
    encode_us_max: u64,
}

impl Encoding {
    fn new(cfg: StreamConfig) -> Option<Self> {
        match lc3::Encoder::new(10_000, cfg.sampling_hz, crate::i2s_rx::SAMPLE_RATE_HZ) {
            Ok(encoder) => {
                info!(
                    "LC3: {=u32} Hz, {=u16} B/frame on CIS {=u16:#x}",
                    cfg.sampling_hz, cfg.octets_per_frame, cfg.cis_handle
                );
                Some(Self { encoder, cfg, seq: 0, dropped: 0, frame: [0; lc3::MAX_FRAME_BYTES], encode_us_max: 0 })
            }
            Err(e) => {
                warn!("LC3: unsupported config {=u32} Hz: {}", cfg.sampling_hz, defmt::Debug2Format(&e));
                None
            }
        }
    }

    fn push(&mut self, pcm: &[i16; FRAMES_PER_BLOCK]) {
        let t0 = Instant::now();
        let out = &mut self.frame[..usize::from(self.cfg.octets_per_frame)];
        if let Err(e) = self.encoder.encode(pcm, out) {
            warn!("LC3 encode failed: {}", defmt::Debug2Format(&e));
            return;
        }
        self.encode_us_max = self.encode_us_max.max(t0.elapsed().as_micros());
        if !crate::ble::queue_iso_sdu(self.cfg.cis_handle, self.seq, out) {
            self.dropped = self.dropped.wrapping_add(1);
        }
        // The sequence number counts SDU intervals, dropped or not.
        self.seq = self.seq.wrapping_add(1);
    }
}

#[embassy_executor::task]
pub async fn run(mut i2s: I2sRx<'static>, mut leds: [Output<'static>; 4], mut pcm: UpChannel) -> ! {
    let mut left = Slot::new();
    let mut right = Slot::new();
    let mut active: Option<Which> = None;
    let mut announced = false;
    let mut pcm_frame = [0u8; PCM_FRAME_BYTES];
    pcm_frame[..2].copy_from_slice(&PCM_MAGIC);
    let mut pcm_seq: u16 = 0;
    let mut mono = [0i16; FRAMES_PER_BLOCK];
    let mut stream: Option<Encoding> = None;
    let mut blocks: u32 = 0;
    let mut window_start = Instant::now();

    loop {
        let block = i2s.next_block().await;
        for ((frame, out), m) in block.iter().zip(pcm_frame[4..].chunks_exact_mut(2)).zip(mono.iter_mut()) {
            let l = left.push(frame.left);
            let r = right.push(frame.right);
            let y = match active {
                Some(Which::Right) => r,
                _ => l,
            };
            let s = libm::roundf(y / 256.0).clamp(i16::MIN as f32, i16::MAX as f32) as i16;
            *m = s;
            out.copy_from_slice(&s.to_le_bytes());
        }
        let peaks = (left.end_block(), right.end_block());

        let peak = match active {
            Some(Which::Left) => Some(peaks.0),
            Some(Which::Right) => Some(peaks.1),
            None => None,
        };
        for (led, threshold) in leds.iter_mut().zip(VU_THRESHOLDS_DBFS) {
            // Active-low LEDs.
            led.set_level(Level::from(!peak.is_some_and(|p| p >= threshold)));
        }

        if let Some(cfg) = audio::STREAM.try_take() {
            if let Some(done) = stream.take() {
                info!(
                    "LC3: stream on CIS {=u16:#x} stopped after {=u16} SDUs ({=u16} dropped)",
                    done.cfg.cis_handle, done.seq, done.dropped
                );
            }
            stream = cfg.and_then(Encoding::new);
        }
        if let Some(enc) = stream.as_mut() {
            // Silence until a microphone slot has been detected.
            if active.is_none() {
                mono.fill(0);
            }
            enc.push(&mono);
        }

        if active.is_some() {
            pcm_frame[2..4].copy_from_slice(&pcm_seq.to_le_bytes());
            // Skipped frames (no host reading, or host too slow) show up as
            // sequence gaps in `mic-capture`.
            pcm_seq = pcm_seq.wrapping_add(1);
            send_pcm(&mut pcm, &pcm_frame);
        }

        blocks += 1;
        if blocks < BLOCKS_PER_WINDOW {
            continue;
        }
        let now = Instant::now();
        let elapsed_us = (now - window_start).as_micros();
        let rate_hz = (u64::from(blocks) * FRAMES_PER_BLOCK as u64 * 1_000_000 / elapsed_us) as u32;
        window_start = now;
        blocks = 0;

        let l = left.take_report();
        let r = right.take_report();
        info!("L: {} | R: {} | LRCK {=u32} Hz", l, r, rate_hz);
        if let Some(enc) = stream.as_mut() {
            info!(
                "LC3: {=u16} SDUs, {=u16} dropped, worst encode {=u64} us",
                enc.seq, enc.dropped, enc.encode_us_max
            );
            enc.encode_us_max = 0;
        }

        let detected = detect(&l, &r);
        if detected != active || !announced {
            match detected {
                Some(Which::Left) => info!("microphone on LEFT slot (SEL=GND)"),
                Some(Which::Right) => info!("microphone on RIGHT slot (SEL=VDD)"),
                None => warn!("no microphone data on either slot: check DOUT->P1.14, VDD, GND"),
            }
            active = detected;
            announced = true;
        }
    }
}

/// Pick the slot carrying microphone data. The undriven slot reads the SDIN
/// pull-down, i.e. a constant zero.
fn detect(l: &Report, r: &Report) -> Option<Which> {
    match (l.rms_dbfs(), r.rms_dbfs()) {
        (Some(_), None) => Some(Which::Left),
        (None, Some(_)) => Some(Which::Right),
        (Some(a), Some(b)) => {
            warn!("both slots carry data: SEL floating or SDIN shared?");
            Some(if a >= b { Which::Left } else { Which::Right })
        }
        (None, None) => None,
    }
}

/// Write one whole PCM frame or nothing. The host may switch the channel to
/// blocking mode on attach; force skip mode so audio never stalls the loop.
fn send_pcm(ch: &mut UpChannel, frame: &[u8]) {
    ch.set_mode(ChannelMode::NoBlockSkip);
    ch.write(frame);
}
