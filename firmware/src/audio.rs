//! Hand-off between the LE Audio control plane (`ble`) and the microphone
//! pipeline (`mic`).

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;

/// What the microphone task must encode and where to send it.
#[derive(Clone, Copy, Debug, defmt::Format)]
pub struct StreamConfig {
    pub cis_handle: u16,
    /// LC3 coded sample rate; the PCM rate is always the I2S rate.
    pub sampling_hz: u32,
    pub octets_per_frame: u16,
}

/// `Some` starts (or retargets) encoding, `None` stops it.
pub static STREAM: Signal<CriticalSectionRawMutex, Option<StreamConfig>> = Signal::new();
