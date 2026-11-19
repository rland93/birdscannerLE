//! Announcement service data for extended advertising.
//!
//! The returned arrays are the payload of a Service Data - 16-bit UUID AD structure
//! (AD type 0x16): the UUID (LE) followed by the service-specific data. The
//! integrator prepends the AD length and type octets.

use crate::{pacs, uuid};

/// Announcement Type: connectable, not requesting a connection (BAP v1.0 §3.5.3).
pub const GENERAL_ANNOUNCEMENT: u8 = 0x00;
/// Announcement Type: connectable and requesting a connection (BAP v1.0 §3.5.3).
pub const TARGETED_ANNOUNCEMENT: u8 = 0x01;

/// BAP Unicast Server announcement (BAP v1.0 Table 3.7): ASCS UUID, Announcement
/// Type (general), Available Audio Contexts (sink u16 LE, source u16 LE),
/// Metadata_Length = 0.
pub fn ascs_service_data() -> [u8; 8] {
    let [u0, u1] = uuid::ASCS.to_le_bytes();
    let [s0, s1, r0, r1] = pacs::available_audio_contexts();
    [u0, u1, GENERAL_ANNOUNCEMENT, s0, s1, r0, r1, 0x00]
}

/// CAP Acceptor announcement (CAP v1.0, "Common Audio Service" service data): CAS
/// UUID, Announcement Type (general).
pub fn cas_service_data() -> [u8; 3] {
    let [u0, u1] = uuid::CAS.to_le_bytes();
    [u0, u1, GENERAL_ANNOUNCEMENT]
}
