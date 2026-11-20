//! Messages between the receiver dongle (`dongle/`, nRF52840) and the host
//! application (`garden-host/`).
//!
//! Transport: USB CDC-ACM (a plain serial port on Windows, macOS and Linux,
//! no driver needed). Each message is postcard-serialised and COBS-framed, so
//! a `0x00` byte ends every frame. Use [`encode`] to write one and
//! [`postcard::accumulator::CobsAccumulator`] (or [`decode`] on a complete
//! frame) to read them back.
//!
//! The dongle is the LE Audio unicast client. The host decides which mics to
//! connect and when they stream; the dongle reports what happens and forwards
//! LC3 frames untouched, so decoding happens on the host.

#![no_std]

use serde::{Deserialize, Serialize};

pub use postcard;

/// Bumped on any incompatible change to the messages below.
pub const PROTOCOL_VERSION: u16 = 2;

/// USB IDs of the dongle: pid.codes open-source VID with its test PID.
pub const USB_VID: u16 = 0x1209;
pub const USB_PID: u16 = 0x0001;
pub const USB_PRODUCT: &str = "nrf53bird receiver";

/// Upper bound of one encoded frame (COBS overhead and the 0x00 included).
/// Sized for an [`DongleToHost::Audio`] frame of up to 155 LC3 bytes.
pub const MAX_FRAME: usize = 256;

/// A Bluetooth LE device address.
#[derive(Serialize, Deserialize, Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Addr {
    /// 0 = public, 1 = random.
    pub kind: u8,
    /// Most significant byte first, i.e. in display order.
    pub bytes: [u8; 6],
}

impl core::fmt::Display for Addr {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        let b = self.bytes;
        write!(f, "{:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X}", b[0], b[1], b[2], b[3], b[4], b[5])
    }
}

/// The dongle's handle for a mic while it is known to the dongle (a
/// connection slot). Stable from [`HostToDongle::Connect`] until the mic is
/// [`MicLink::Gone`].
#[derive(Serialize, Deserialize, Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct MicId(pub u8);

/// LC3 settings the host can ask for; each is a BAP-defined configuration
/// with 10 ms frames.
#[derive(Serialize, Deserialize, Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Quality {
    /// 16_2: 16 kHz, 40 bytes/frame (32 kbit/s). Lowest air time and power.
    Low,
    /// 24_2: 24 kHz, 60 bytes/frame (48 kbit/s).
    Medium,
    /// 32_2: 32 kHz, 80 bytes/frame (64 kbit/s).
    High,
}

impl Quality {
    pub const fn sampling_hz(self) -> u32 {
        match self {
            Quality::Low => 16_000,
            Quality::Medium => 24_000,
            Quality::High => 32_000,
        }
    }

    pub const fn octets_per_frame(self) -> u16 {
        match self {
            Quality::Low => 40,
            Quality::Medium => 60,
            Quality::High => 80,
        }
    }
}

/// Why a mic link failed or ended.
#[derive(Serialize, Deserialize, Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Failure {
    /// Connection could not be established in time.
    ConnectTimeout,
    /// The mic has no usable PACS/ASCS or LC3 capability we can use.
    NotAnLeAudioSource,
    /// The mic rejected a stream configuration step.
    Rejected,
    /// The CIS could not be created or was lost.
    Iso,
    /// Out of connection slots on the dongle.
    NoSlot,
    /// Link dropped; HCI disconnect reason code.
    Disconnected(u8),
}

/// Lifecycle of one mic as seen by the dongle.
#[derive(Serialize, Deserialize, Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum MicLink {
    Connecting,
    /// Connected; reading PACS/ASCS.
    Discovering,
    /// Connected and idle (no stream). Cheap for the mic.
    Ready,
    /// Configuring the ASE and creating the CIS.
    Starting,
    Streaming,
    Stopping,
    /// No longer connected; the [`MicId`] is free again.
    Gone(Failure),
}

#[derive(Serialize, Deserialize, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DongleToHost<'a> {
    /// Sent on USB connect and in reply to [`HostToDongle::Hello`].
    Hello { protocol: u16, firmware: &'a str, max_mics: u8 },
    /// An LE Audio source (advertising ASCS service data) seen while scanning.
    Advert { addr: Addr, rssi: i8, name: &'a str },
    /// State of a mic slot changed.
    MicStatus { mic: MicId, addr: Addr, link: MicLink },
    /// Signal strength of a connected mic, reported periodically.
    Rssi { mic: MicId, rssi: i8 },
    /// The stream that follows on `mic` uses these LC3 parameters. Repeated
    /// in the `Hello` snapshot for mic already streaming, so a host that
    /// starts mid-stream still decodes.
    StreamConfig { mic: MicId, sampling_hz: u32, frame_duration_us: u32, octets_per_frame: u16 },
    /// One LC3 frame. `seq` is the ISO packet sequence number; an empty
    /// `frame` means the controller reported it lost (decode it as a gap).
    Audio { mic: MicId, seq: u16, frame: &'a [u8] },
    /// Human-readable diagnostics.
    Log { text: &'a str },
}

#[derive(Serialize, Deserialize, Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum HostToDongle {
    /// Ask for a [`DongleToHost::Hello`] and the status of every mic slot.
    Hello,
    /// Start or stop reporting [`DongleToHost::Advert`]s.
    Scan { enable: bool },
    /// Connect to a mic and leave it [`MicLink::Ready`].
    Connect { addr: Addr },
    Disconnect { mic: MicId },
    StartStream { mic: MicId, quality: Quality },
    StopStream { mic: MicId },
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// Output buffer too small or message malformed.
    Encode,
    Decode,
}

/// Serialise `msg` as one COBS frame (terminated by 0x00) into `buf`.
pub fn encode<'b, T: Serialize>(msg: &T, buf: &'b mut [u8]) -> Result<&'b mut [u8], Error> {
    postcard::to_slice_cobs(msg, buf).map_err(|_| Error::Encode)
}

/// Decode one complete COBS frame (with or without its trailing 0x00), in place.
pub fn decode<'a, T: Deserialize<'a>>(frame: &'a mut [u8]) -> Result<T, Error> {
    postcard::from_bytes_cobs(frame).map_err(|_| Error::Decode)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn audio_frame_round_trips_and_fits() {
        let lc3 = [0xAAu8; 155];
        let msg = DongleToHost::Audio { mic: MicId(3), seq: 0xBEEF, frame: &lc3 };
        let mut buf = [0u8; MAX_FRAME];
        let n = encode(&msg, &mut buf).unwrap().len();
        assert_eq!(buf[n - 1], 0);
        assert!(buf[..n - 1].iter().all(|&b| b != 0));
        let back: DongleToHost = decode(&mut buf[..n]).unwrap();
        assert_eq!(back, msg);
    }

    #[test]
    fn accumulator_splits_a_byte_stream_into_messages() {
        use postcard::accumulator::{CobsAccumulator, FeedResult};
        let a = HostToDongle::StartStream { mic: MicId(1), quality: Quality::Low };
        let b = HostToDongle::Connect { addr: Addr { kind: 1, bytes: [0xDE, 0x10, 0xEB, 0xCE, 0x60, 0x9E] } };
        let mut wire = [0u8; 2 * MAX_FRAME];
        let n1 = encode(&a, &mut wire).unwrap().len();
        let n2 = encode(&b, &mut wire[n1..]).unwrap().len();
        let mut acc: CobsAccumulator<MAX_FRAME> = CobsAccumulator::new();
        let mut got = [None, None];
        let mut i = 0;
        // Feed in awkward 3-byte chunks.
        for chunk in wire[..n1 + n2].chunks(3) {
            let mut window = chunk;
            while !window.is_empty() {
                window = match acc.feed::<HostToDongle>(window) {
                    FeedResult::Consumed => break,
                    FeedResult::Success { data, remaining } => {
                        got[i] = Some(data);
                        i += 1;
                        remaining
                    }
                    other => panic!("unexpected {:?}", core::mem::discriminant(&other)),
                };
            }
        }
        assert_eq!(got, [Some(a), Some(b)]);
    }

    #[test]
    fn addr_displays_msb_first() {
        extern crate std;
        let a = Addr { kind: 1, bytes: [0xDE, 0x10, 0xEB, 0xCE, 0x60, 0x9E] };
        assert_eq!(std::format!("{a}"), "DE:10:EB:CE:60:9E");
    }
}
