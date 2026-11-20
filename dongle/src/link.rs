//! USB CDC-ACM link to the host application, speaking `garden-link`.
//!
//! [`send`] encodes a message into the outgoing queue (dropping it when the
//! host is not keeping up, which only ever affects audio); [`FROM_HOST`]
//! carries decoded commands.

use defmt::{info, warn};
use embassy_futures::join::join3;
use embassy_nrf::usb::Driver;
use embassy_nrf::usb::vbus_detect::SoftwareVbusDetect;
use embassy_nrf::{Peri, peripherals};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::driver::EndpointError;
use embassy_usb::{Builder, Config};
use garden_link::postcard::accumulator::{CobsAccumulator, FeedResult};
use garden_link::{DongleToHost, HostToDongle, MAX_FRAME};
use heapless::Vec;
use static_cell::StaticCell;

use crate::Irqs;

type Frame = Vec<u8, MAX_FRAME>;

/// ~160 ms of audio from three mics, plus status messages.
static TO_HOST: Channel<CriticalSectionRawMutex, Frame, 48> = Channel::new();
pub static FROM_HOST: Channel<CriticalSectionRawMutex, HostToDongle, 8> = Channel::new();

/// Queue a message for the host. Returns false if it was dropped.
pub fn send(msg: &DongleToHost<'_>) -> bool {
    let mut buf = [0u8; MAX_FRAME];
    let Ok(frame) = garden_link::encode(msg, &mut buf) else {
        warn!("link: message too large, dropped");
        return false;
    };
    let frame = Vec::from_slice(frame).unwrap();
    TO_HOST.try_send(frame).is_ok()
}

/// Send `Log` text to the host (also logged locally).
pub fn log(text: &str) {
    info!("{}", text);
    send(&DongleToHost::Log { text });
}

/// Formatted [`log`]: `link::logf(format_args!(...))`.
pub fn logf(args: core::fmt::Arguments<'_>) {
    let mut s: heapless::String<120> = heapless::String::new();
    let _ = core::fmt::write(&mut s, args);
    log(&s);
}

const MAX_PACKET: u16 = 64;

#[embassy_executor::task]
pub async fn run(usbd: Peri<'static, peripherals::USBD>) -> ! {
    // The dongle is bus powered: VBUS is always present while it runs.
    static VBUS: StaticCell<SoftwareVbusDetect> = StaticCell::new();
    let vbus = VBUS.init(SoftwareVbusDetect::new(true, true));
    let driver = Driver::new(usbd, Irqs, &*vbus);

    let mut config = Config::new(garden_link::USB_VID, garden_link::USB_PID);
    config.manufacturer = Some("birdscannerLE");
    config.product = Some(garden_link::USB_PRODUCT);
    config.serial_number = Some(serial_number());
    config.max_power = 100;
    config.max_packet_size_0 = 64;

    static CONFIG_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static BOS_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static CONTROL_BUF: StaticCell<[u8; 64]> = StaticCell::new();
    static STATE: StaticCell<State> = StaticCell::new();
    let mut builder = Builder::new(
        driver,
        config,
        CONFIG_DESC.init([0; 256]),
        BOS_DESC.init([0; 256]),
        &mut [],
        CONTROL_BUF.init([0; 64]),
    );
    let class = CdcAcmClass::new(&mut builder, STATE.init(State::new()), MAX_PACKET);
    let mut usb = builder.build();
    let (mut tx, mut rx) = class.split();

    let tx_loop = async {
        loop {
            let frame = TO_HOST.receive().await;
            if let Err(EndpointError::Disabled) = write_frame(&mut tx, &frame).await {
                // Unplugged or not configured yet; wait for the host.
                tx.wait_connection().await;
            }
        }
    };
    let rx_loop = async {
        let mut acc: CobsAccumulator<MAX_FRAME> = CobsAccumulator::new();
        let mut buf = [0u8; MAX_PACKET as usize];
        loop {
            let n = match rx.read_packet(&mut buf).await {
                Ok(n) => n,
                Err(_) => {
                    rx.wait_connection().await;
                    continue;
                }
            };
            let mut window = &buf[..n];
            while !window.is_empty() {
                window = match acc.feed::<HostToDongle>(window) {
                    FeedResult::Consumed => break,
                    FeedResult::OverFull(rest) | FeedResult::DeserError(rest) => {
                        warn!("link: bad frame from host");
                        rest
                    }
                    FeedResult::Success { data, remaining } => {
                        // Never block USB intake on a busy manager (that
                        // would also stall the bootloader request).
                        if FROM_HOST.try_send(data).is_err() {
                            warn!("link: command queue full, dropped");
                        }
                        remaining
                    }
                };
            }
        }
    };
    join3(usb.run(), tx_loop, rx_loop).await;
    unreachable!()
}

async fn write_frame<'d>(
    tx: &mut embassy_usb::class::cdc_acm::Sender<'d, Driver<'d, &'d SoftwareVbusDetect>>,
    frame: &[u8],
) -> Result<(), EndpointError> {
    for chunk in frame.chunks(MAX_PACKET as usize) {
        tx.write_packet(chunk).await?;
    }
    // A full last packet needs a zero-length packet to end the transfer.
    if frame.len().is_multiple_of(MAX_PACKET as usize) {
        tx.write_packet(&[]).await?;
    }
    Ok(())
}

/// Unique per chip, so several dongles can be told apart on the host.
fn serial_number() -> &'static str {
    static SERIAL: StaticCell<[u8; 16]> = StaticCell::new();
    let ficr = embassy_nrf::pac::FICR;
    let id = (u64::from(ficr.deviceid(1).read()) << 32) | u64::from(ficr.deviceid(0).read());
    let buf = SERIAL.init([0; 16]);
    for (i, b) in buf.iter_mut().enumerate() {
        *b = b"0123456789ABCDEF"[((id >> (60 - 4 * i)) & 0xf) as usize];
    }
    core::str::from_utf8(buf).unwrap()
}
