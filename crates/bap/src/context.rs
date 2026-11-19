//! Context Type bit values (Bluetooth Assigned Numbers, Generic Audio: "Context Type").
//! Used by the Available/Supported Audio Contexts characteristics, the
//! Preferred_Audio_Contexts / Streaming_Audio_Contexts metadata and announcements.

/// No context type (0x0000 is "Prohibited" as a context value).
pub const PROHIBITED: u16 = 0x0000;
pub const UNSPECIFIED: u16 = 0x0001;
pub const CONVERSATIONAL: u16 = 0x0002;
pub const MEDIA: u16 = 0x0004;
pub const GAME: u16 = 0x0008;
pub const INSTRUCTIONAL: u16 = 0x0010;
pub const VOICE_ASSISTANTS: u16 = 0x0020;
pub const LIVE: u16 = 0x0040;
pub const SOUND_EFFECTS: u16 = 0x0080;
pub const NOTIFICATIONS: u16 = 0x0100;
pub const RINGTONE: u16 = 0x0200;
pub const ALERTS: u16 = 0x0400;
pub const EMERGENCY_ALARM: u16 = 0x0800;
