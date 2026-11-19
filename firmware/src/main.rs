//! nRF5340 DK application core: I2S MEMS microphone exposed as a Bluetooth
//! LE Audio source (BAP Unicast Server, LC3 over a CIS).
//!
//! Wiring (nRF is I2S clock master):
//!   BCLK  -> P1.15 (SCK)
//!   LRCLK -> P1.12 (LRCK)
//!   DOUT  -> P1.14 (SDIN)
//!   SEL   -> GND = left slot, VDD = right slot (auto-detected)
//!
//! The BLE controller runs on the network core (`netcore/`, Nordic
//! SoftDevice Controller); this core runs the host (trouble-host) and talks
//! HCI to it over shared RAM (`hci_link`).
//!
//! Output:
//!   * defmt log (RTT channel 0, "defmt"): mic levels, BLE/LE Audio events.
//!   * LED1..LED4: VU meter of the detected slot.
//!   * RTT channel 1 ("pcm"): DC-blocked 16-bit mono PCM, see `mic-capture`.

#![no_std]
#![no_main]

extern crate alloc;

mod audio;
mod ble;
mod bonds;
mod hci_link;
mod i2s_rx;
mod meter;
mod mic;

use defmt::info;
use embassy_executor::Spawner;
use embassy_nrf::bind_interrupts;
use embassy_nrf::config::{Config, HfclkSource, LfclkSource, LfxoCapacitance};
use embassy_nrf::gpio::{Level, Output, OutputDrive};
use embedded_alloc::LlffHeap;
use panic_probe as _;
use rtt_target::{ChannelMode, rtt_init};
use static_cell::ConstStaticCell;

use i2s_rx::{DmaBuffers, FRAMES_PER_BLOCK, Frame, I2sRx, SAMPLE_RATE_HZ};

bind_interrupts!(struct Irqs {
    I2S0 => i2s_rx::InterruptHandler;
    IPC => hci_link::IpcInterruptHandler;
});

/// Heap for the LC3 encoder state (~3 KB per stream).
#[global_allocator]
static HEAP: LlffHeap = LlffHeap::empty();
const HEAP_SIZE: usize = 16 * 1024;

static DMA_BUFFERS: ConstStaticCell<DmaBuffers> =
    ConstStaticCell::new([[Frame { left: 0, right: 0 }; FRAMES_PER_BLOCK]; 3]);

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let channels = rtt_init! {
        up: {
            0: { size: 4096, mode: ChannelMode::NoBlockSkip, name: "defmt" }
            1: { size: 65536, mode: ChannelMode::NoBlockSkip, name: "pcm" }
        }
    };
    rtt_target::set_defmt_channel(channels.up.0);

    {
        use core::mem::MaybeUninit;
        static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
        // SAFETY: called once, before any allocation.
        unsafe { HEAP.init(&raw mut HEAP_MEM as usize, HEAP_SIZE) }
    }

    let mut config = Config::default();
    config.hfclk_source = HfclkSource::ExternalXtal;
    config.lfclk_source = LfclkSource::ExternalXtal;
    // nRF5340 DK: LFXO uses the internal 7 pF load capacitors.
    config.internal_capacitors.lfxo = Some(LfxoCapacitance::_7pF);
    let p = embassy_nrf::init(config);
    full_speed_cpu();

    let transport = hci_link::start_netcore(Irqs);
    ble::start(spawner, transport, bonds::BondStore::new(p.NVMC));

    let leds = [
        Output::new(p.P0_28, Level::High, OutputDrive::Standard),
        Output::new(p.P0_29, Level::High, OutputDrive::Standard),
        Output::new(p.P0_30, Level::High, OutputDrive::Standard),
        Output::new(p.P0_31, Level::High, OutputDrive::Standard),
    ];
    info!("I2S mic: SCK=P1.15 LRCK=P1.12 SDIN=P1.14, {=u32} Hz, 32-bit slots", SAMPLE_RATE_HZ);
    let i2s = I2sRx::start(Irqs, p.P1_15, p.P1_12, p.P1_14, DMA_BUFFERS.take()).await;
    spawner.spawn(mic::run(i2s, leds, channels.up.1).unwrap());
}

/// The app core resets with its instruction cache off and HCLK = 64 MHz
/// (CLOCK.HFCLKCTRL = Div2); embassy-nrf changes neither. Without the cache
/// every flash fetch pays wait states, which made LC3 encoding several times
/// slower than real time.
fn full_speed_cpu() {
    use embassy_nrf::pac;
    pac::CACHE.invalidate().write(|w| w.set_invalidate(true));
    pac::CACHE.enable().write(|w| w.set_enable(true));
    pac::CLOCK.hfclkctrl().write(|w| w.set_hclk(pac::clock::vals::Hclk::Div1));
}
