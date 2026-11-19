//! I2S MEMS microphone bring-up on the nRF5340 DK (application core).
//!
//! Wiring (nRF is I2S clock master):
//!   BCLK  -> P1.15 (SCK)
//!   LRCLK -> P1.12 (LRCK)
//!   DOUT  -> P1.14 (SDIN)
//!   SEL   -> GND = left slot, VDD = right slot (auto-detected)
//!
//! Output:
//!   * defmt log (RTT channel 0, "defmt"): per-slot level, DC offset and the
//!     measured LRCK rate about twice a second.
//!   * LED1..LED4: VU meter of the detected slot.
//!   * RTT channel 1 ("pcm"): DC-blocked 16-bit mono PCM of the detected slot
//!     in framed blocks, recorded by the `mic-capture` host tool.

#![no_std]
#![no_main]

mod i2s_rx;
mod meter;

use defmt::{info, warn};
use embassy_executor::Spawner;
use embassy_nrf::config::{Config, HfclkSource, LfclkSource, LfxoCapacitance};
use embassy_nrf::gpio::{Level, Output, OutputDrive};
use embassy_nrf::bind_interrupts;
use embassy_time::Instant;
use panic_probe as _;
use rtt_target::{ChannelMode, UpChannel, rtt_init};
use static_cell::ConstStaticCell;

use i2s_rx::{DmaBuffers, FRAMES_PER_BLOCK, Frame, I2sRx, SAMPLE_RATE_HZ};
use meter::{Report, Slot};

bind_interrupts!(struct Irqs {
    I2S0 => i2s_rx::InterruptHandler;
});

/// Blocks per log/report window (~524 ms).
const BLOCKS_PER_WINDOW: u32 = 32;
/// Block peak (dBFS) needed to light LED1..LED4. A quiet room peaks around
/// -55 dBFS, so all LEDs stay dark until there is actual sound.
const VU_THRESHOLDS_DBFS: [f32; 4] = [-50.0, -40.0, -30.0, -20.0];

/// PCM frame on RTT channel 1: magic, little-endian sequence number, then
/// FRAMES_PER_BLOCK little-endian i16 samples. Must match `mic-capture`.
const PCM_MAGIC: [u8; 2] = [0xA5, 0x5A];
const PCM_FRAME_BYTES: usize = 4 + FRAMES_PER_BLOCK * 2;

static DMA_BUFFERS: ConstStaticCell<DmaBuffers> =
    ConstStaticCell::new([[Frame { left: 0, right: 0 }; FRAMES_PER_BLOCK]; 3]);

#[derive(Clone, Copy, PartialEq, Eq, defmt::Format)]
enum Which {
    Left,
    Right,
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let channels = rtt_init! {
        up: {
            0: { size: 4096, mode: ChannelMode::NoBlockSkip, name: "defmt" }
            1: { size: 65536, mode: ChannelMode::NoBlockSkip, name: "pcm" }
        }
    };
    rtt_target::set_defmt_channel(channels.up.0);
    let mut pcm = channels.up.1;

    let mut config = Config::default();
    config.hfclk_source = HfclkSource::ExternalXtal;
    config.lfclk_source = LfclkSource::ExternalXtal;
    // nRF5340 DK: LFXO uses the internal 7 pF load capacitors.
    config.internal_capacitors.lfxo = Some(LfxoCapacitance::_7pF);
    let p = embassy_nrf::init(config);

    let mut leds = [
        Output::new(p.P0_28, Level::High, OutputDrive::Standard),
        Output::new(p.P0_29, Level::High, OutputDrive::Standard),
        Output::new(p.P0_30, Level::High, OutputDrive::Standard),
        Output::new(p.P0_31, Level::High, OutputDrive::Standard),
    ];

    info!(
        "I2S mic: SCK=P1.15 LRCK=P1.12 SDIN=P1.14, {=u32} Hz, 32-bit slots",
        SAMPLE_RATE_HZ
    );
    let mut i2s = I2sRx::start(Irqs, p.P1_15, p.P1_12, p.P1_14, DMA_BUFFERS.take()).await;

    let mut left = Slot::new();
    let mut right = Slot::new();
    let mut active: Option<Which> = None;
    let mut announced = false;
    let mut pcm_frame = [0u8; PCM_FRAME_BYTES];
    pcm_frame[..2].copy_from_slice(&PCM_MAGIC);
    let mut pcm_seq: u16 = 0;
    let mut blocks: u32 = 0;
    let mut window_start = Instant::now();

    loop {
        let block = i2s.next_block().await;
        for (frame, out) in block.iter().zip(pcm_frame[4..].chunks_exact_mut(2)) {
            let l = left.push(frame.left);
            let r = right.push(frame.right);
            let y = match active {
                Some(Which::Right) => r,
                _ => l,
            };
            let s = libm::roundf(y / 256.0).clamp(i16::MIN as f32, i16::MAX as f32) as i16;
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
