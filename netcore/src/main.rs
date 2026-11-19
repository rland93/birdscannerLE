//! nRF5340 network core: Nordic SoftDevice Controller (BLE link layer incl.
//! connected isochronous channels) driven over HCI by the app core.

#![no_std]
#![no_main]

mod dispatch;
mod hci;

use defmt::{info, unwrap};
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_nrf::mode::Blocking;
use embassy_nrf::peripherals::RNG;
use embassy_nrf::{bind_interrupts, config, rng};
use embassy_time::Timer;
use hci_shm::Shared;
use nrf_sdc::mpsl::MultiprotocolServiceLayer;
use nrf_sdc::{self as sdc, mpsl, raw};
use panic_probe as _;
use static_cell::StaticCell;

bind_interrupts!(struct Irqs {
    RNG => rng::InterruptHandler<RNG>;
    SWI0 => mpsl::LowPrioInterruptHandler;
    CLOCK_POWER => mpsl::ClockInterruptHandler;
    RADIO => mpsl::HighPrioInterruptHandler;
    TIMER0 => mpsl::HighPrioInterruptHandler;
    RTC0 => mpsl::HighPrioInterruptHandler;
    IPC => hci::IpcInterruptHandler;
});

/// SDC working memory; `build` logs the exact requirement if this is wrong.
const SDC_MEM: usize = 12 * 1024;

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
    config.lfclk_source = config::LfclkSource::ExternalXtal;
    let p = embassy_nrf::init(config);

    // SAFETY: the shared region is reserved for this in both linker scripts.
    let shm = unsafe { Shared::get() };
    while !shm.ready() {
        Timer::after_millis(1).await;
    }

    let mpsl_p = mpsl::Peripherals::new(p.RTC0, p.TIMER0, p.TIMER1, p.TEMP, p.PPI_CH0, p.PPI_CH1, p.PPI_CH2);
    let lfclk_cfg = mpsl::raw::mpsl_clock_lfclk_cfg_t {
        source: mpsl::raw::MPSL_CLOCK_LF_SRC_XTAL as u8,
        rc_ctiv: 0,
        rc_temp_ctiv: 0,
        accuracy_ppm: mpsl::raw::MPSL_DEFAULT_CLOCK_ACCURACY_PPM as u16,
        skip_wait_lfclk_started: false,
    };
    static MPSL: StaticCell<MultiprotocolServiceLayer> = StaticCell::new();
    let mpsl = MPSL.init(unwrap!(MultiprotocolServiceLayer::new(mpsl_p, Irqs, lfclk_cfg)));
    spawner.spawn(unwrap!(mpsl_task(mpsl)));

    let sdc_p = sdc::Peripherals::new(
        p.PPI_CH3, p.PPI_CH4, p.PPI_CH5, p.PPI_CH6, p.PPI_CH7, p.PPI_CH8, p.PPI_CH9, p.PPI_CH10, p.PPI_CH11,
        p.PPI_CH12,
    );
    static RNG_CELL: StaticCell<rng::Rng<'static, Blocking>> = StaticCell::new();
    let rng = RNG_CELL.init(rng::Rng::new_blocking(p.RNG));

    let builder = unwrap!(sdc::Builder::new())
        .support_adv()
        .support_ext_adv()
        .support_peripheral()
        .support_dle_peripheral()
        .support_le_2m_phy()
        .support_phy_update_peripheral()
        .support_sca_peripheral()
        .support_cis_peripheral();
    let builder = unwrap!(unwrap!(unwrap!(builder.peripheral_count(1)).adv_count(1)).buffer_cfg(251, 251, 3, 3));
    // One CIG with up to two CISes: one Source ASE today, room for a second.
    sdc_cfg(raw::SDC_CFG_TYPE_CIG_COUNT, raw::sdc_cfg_t { cig_count: raw::sdc_cfg_role_count_t { count: 1 } });
    sdc_cfg(raw::SDC_CFG_TYPE_CIS_COUNT, raw::sdc_cfg_t { cis_count: raw::sdc_cfg_role_count_t { count: 2 } });
    sdc_cfg(
        raw::SDC_CFG_TYPE_ISO_BUFFER_CFG,
        raw::sdc_cfg_t {
            iso_buffer_cfg: raw::sdc_cfg_iso_buffer_cfg_t {
                tx_sdu_buffer_count: 4,
                tx_sdu_buffer_size: 160,
                tx_pdu_buffer_per_stream_count: 2,
                rx_pdu_buffer_per_stream_count: 1,
                rx_sdu_buffer_count: 1,
                rx_sdu_buffer_size: 160,
            },
        },
    );
    info!("SDC needs {=usize} bytes of RAM", unwrap!(builder.required_memory()));

    static MEM: StaticCell<sdc::Mem<SDC_MEM>> = StaticCell::new();
    let mem = MEM.init(sdc::Mem::new());
    static SDC: StaticCell<sdc::SoftdeviceController<'static>> = StaticCell::new();
    let sdc = SDC.init(unwrap!(builder.build(sdc_p, rng, mpsl, mem)));
    info!("SoftDevice Controller up, bridging HCI");

    hci::run(shm, sdc).await
}
