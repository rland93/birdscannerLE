//! nRF52840 Dongle: LE Audio unicast client for the birdscannerLE garden mics.
//!
//! The SoftDevice Controller and trouble-host run on this chip; mics are
//! connected, paired and streamed on request from the host application, and
//! their LC3 frames are forwarded over USB (`garden-link` protocol).

#![no_std]
#![no_main]

mod hci_local;
mod link;
mod mics;

use defmt::{info, unwrap};
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_nrf::mode::Blocking;
use embassy_nrf::peripherals::{RNG, USBD};
use embassy_nrf::{bind_interrupts, config, rng, usb};
use nrf_sdc::mpsl::MultiprotocolServiceLayer;
use nrf_sdc::{self as sdc, mpsl, raw};
use panic_probe as _;
use static_cell::StaticCell;

bind_interrupts!(pub struct Irqs {
    RNG => rng::InterruptHandler<RNG>;
    EGU0_SWI0 => mpsl::LowPrioInterruptHandler;
    CLOCK_POWER => mpsl::ClockInterruptHandler;
    RADIO => mpsl::HighPrioInterruptHandler;
    TIMER0 => mpsl::HighPrioInterruptHandler;
    RTC0 => mpsl::HighPrioInterruptHandler;
    USBD => usb::InterruptHandler<USBD>;
});

/// Mics the dongle can hold connections (and a CIS each) to at once.
pub const MAX_MICS: usize = 3;
/// SDC working memory; `build` logs the exact requirement if this is wrong.
const SDC_MEM: usize = 24 * 1024;

#[embassy_executor::task]
async fn mpsl_task(mpsl: &'static MultiprotocolServiceLayer<'static>) -> ! {
    mpsl.run().await
}

fn sdc_cfg(cfg_type: u32, value: raw::sdc_cfg_t) {
    // SAFETY: called between sdc_init (Builder::new) and sdc_enable (build).
    let ret = unsafe { raw::sdc_cfg_set(raw::SDC_DEFAULT_RESOURCE_CFG_TAG as u8, cfg_type as u8, &value) };
    assert!(ret >= 0, "sdc_cfg_set({}) failed: {}", cfg_type, ret);
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut config = config::Config::default();
    config.hfclk_source = config::HfclkSource::ExternalXtal;
    // Calibrated RC: works whether or not a 32.768 kHz crystal is fitted.
    config.lfclk_source = config::LfclkSource::InternalRC;
    let p = embassy_nrf::init(config);
    info!("birdscannerLE receiver dongle starting");

    let mpsl_p = mpsl::Peripherals::new(p.RTC0, p.TIMER0, p.TEMP, p.PPI_CH19, p.PPI_CH30, p.PPI_CH31);
    let lfclk_cfg = mpsl::raw::mpsl_clock_lfclk_cfg_t {
        source: mpsl::raw::MPSL_CLOCK_LF_SRC_RC as u8,
        rc_ctiv: mpsl::raw::MPSL_RECOMMENDED_RC_CTIV as u8,
        rc_temp_ctiv: mpsl::raw::MPSL_RECOMMENDED_RC_TEMP_CTIV as u8,
        accuracy_ppm: mpsl::raw::MPSL_DEFAULT_CLOCK_ACCURACY_PPM as u16,
        skip_wait_lfclk_started: mpsl::raw::MPSL_DEFAULT_SKIP_WAIT_LFCLK_STARTED != 0,
    };
    static MPSL: StaticCell<MultiprotocolServiceLayer> = StaticCell::new();
    let mpsl = MPSL.init(unwrap!(MultiprotocolServiceLayer::new(mpsl_p, Irqs, lfclk_cfg)));
    spawner.spawn(unwrap!(mpsl_task(mpsl)));

    // USB needs the crystal oscillator running all the time; MPSL would
    // otherwise stop it between radio events.
    static HFCLK: StaticCell<mpsl::Hfclk> = StaticCell::new();
    HFCLK.init(unwrap!(mpsl.request_hfclk().await));
    spawner.spawn(unwrap!(link::run(p.USBD)));

    let sdc_p = sdc::Peripherals::new(
        p.PPI_CH17, p.PPI_CH18, p.PPI_CH20, p.PPI_CH21, p.PPI_CH22, p.PPI_CH23, p.PPI_CH24, p.PPI_CH25, p.PPI_CH26,
        p.PPI_CH27, p.PPI_CH28, p.PPI_CH29,
    );
    static RNG_CELL: StaticCell<rng::Rng<'static, Blocking>> = StaticCell::new();
    let rng = RNG_CELL.init(rng::Rng::new_blocking(p.RNG));

    let builder = unwrap!(sdc::Builder::new())
        .support_scan()
        .support_central()
        .support_dle_central()
        .support_le_2m_phy()
        .support_phy_update_central()
        .support_sca_central()
        .support_cis_central()
        // The host may keep scanning (Add page) while a mic is being connected.
        .support_parallel_scanning_and_initiating();
    let builder = unwrap!(unwrap!(builder.central_count(MAX_MICS as u8)).peripheral_count(0));
    let builder = unwrap!(builder.buffer_cfg(251, 251, 3, 3));
    // One CIG per mic, each with a single CIS: mics come and go independently.
    sdc_cfg(raw::SDC_CFG_TYPE_CIG_COUNT, raw::sdc_cfg_t { cig_count: raw::sdc_cfg_role_count_t { count: MAX_MICS as u8 } });
    sdc_cfg(raw::SDC_CFG_TYPE_CIS_COUNT, raw::sdc_cfg_t { cis_count: raw::sdc_cfg_role_count_t { count: MAX_MICS as u8 } });
    sdc_cfg(
        raw::SDC_CFG_TYPE_ISO_BUFFER_CFG,
        raw::sdc_cfg_t {
            iso_buffer_cfg: raw::sdc_cfg_iso_buffer_cfg_t {
                tx_sdu_buffer_count: 1,
                tx_sdu_buffer_size: 32,
                tx_pdu_buffer_per_stream_count: 1,
                rx_pdu_buffer_per_stream_count: 2,
                rx_sdu_buffer_count: 2 * MAX_MICS as u8,
                rx_sdu_buffer_size: 160,
            },
        },
    );
    info!("SDC needs {=usize} bytes of RAM", unwrap!(builder.required_memory()));
    static MEM: StaticCell<sdc::Mem<SDC_MEM>> = StaticCell::new();
    static SDC: StaticCell<sdc::SoftdeviceController<'static>> = StaticCell::new();
    let sdc = SDC.init(unwrap!(builder.build(sdc_p, rng, mpsl, MEM.init(sdc::Mem::new()))));

    mics::start(spawner, hci_local::LocalTransport::new(sdc));
}
