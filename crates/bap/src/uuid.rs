//! 16-bit UUIDs (Bluetooth Assigned Numbers, "16-bit UUIDs": GATT services and
//! characteristics). Values cross-checked against Zephyr `include/zephyr/bluetooth/uuid.h`.

/// Audio Stream Control Service.
pub const ASCS: u16 = 0x184E;
/// Published Audio Capabilities Service.
pub const PACS: u16 = 0x1850;
/// Common Audio Service.
pub const CAS: u16 = 0x1853;

/// Sink ASE characteristic (unused here: no Sink ASEs are exposed).
pub const SINK_ASE: u16 = 0x2BC4;
/// Source ASE characteristic.
pub const SOURCE_ASE: u16 = 0x2BC5;
/// ASE Control Point characteristic.
pub const ASE_CONTROL_POINT: u16 = 0x2BC6;

/// Sink PAC characteristic (unused here: no sink capability).
pub const SINK_PAC: u16 = 0x2BC9;
/// Sink Audio Locations characteristic (unused here).
pub const SINK_AUDIO_LOCATIONS: u16 = 0x2BCA;
/// Source PAC characteristic.
pub const SOURCE_PAC: u16 = 0x2BCB;
/// Source Audio Locations characteristic.
pub const SOURCE_AUDIO_LOCATIONS: u16 = 0x2BCC;
/// Available Audio Contexts characteristic.
pub const AVAILABLE_AUDIO_CONTEXTS: u16 = 0x2BCD;
/// Supported Audio Contexts characteristic.
pub const SUPPORTED_AUDIO_CONTEXTS: u16 = 0x2BCE;
