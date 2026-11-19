//! Published Audio Capabilities Service (PACS v1.0) characteristic values and the
//! fixed capabilities of this device (mono LC3 microphone, Source direction only).
//!
//! All multi-octet fields are little-endian.

use crate::context;

/// Codec_ID for LC3: Coding_Format 0x06 (Assigned Numbers, "Coding Format"),
/// Company_ID 0x0000, Vendor-specific codec ID 0x0000 (both shall be 0 when the
/// coding format is not 0xFF, ASCS v1.0.1 Table 4.3).
pub const LC3_CODEC_ID: [u8; 5] = [0x06, 0x00, 0x00, 0x00, 0x00];

/// Supported sampling frequencies (Hz).
pub const SAMPLING_FREQUENCIES_HZ: [u32; 3] = [16_000, 24_000, 32_000];
/// The only supported LC3 frame duration (µs).
pub const FRAME_DURATION_US: u32 = 10_000;
/// Smallest supported Octets_Per_Codec_Frame. The range 26..=80 contains the BAP v1.0
/// Table 3.5 codec settings 16_2 (40 octets), 24_2 (60) and 32_2 (80).
pub const MIN_OCTETS_PER_FRAME: u16 = 26;
/// Largest supported Octets_Per_Codec_Frame.
pub const MAX_OCTETS_PER_FRAME: u16 = 80;
/// Maximum number of codec frames (per channel) in one SDU.
pub const MAX_FRAMES_PER_SDU: u8 = 1;

/// Source Audio Locations: Front Left (Assigned Numbers, "Audio Location", bit 0).
pub const SOURCE_AUDIO_LOCATIONS: u32 = 0x0000_0001;

/// Sink contexts (supported and available): none, there is no sink.
pub const SINK_CONTEXTS: u16 = context::PROHIBITED;
/// Source contexts (supported and available). PACS v1.0 requires Unspecified to be
/// set for every supported direction.
pub const SOURCE_CONTEXTS: u16 = context::UNSPECIFIED
    | context::CONVERSATIONAL
    | context::MEDIA
    | context::GAME
    | context::VOICE_ASSISTANTS
    | context::LIVE;

/// Source PAC characteristic value (PACS v1.0 Table 3.2): one PAC record.
///
/// Codec_Specific_Capabilities LTVs (Assigned Numbers, Generic Audio
/// "Codec_Specific_Capabilities LTV structures"):
/// * 0x01 Supported_Sampling_Frequencies, u16 bitfield: bit 2 = 16 kHz,
///   bit 4 = 24 kHz, bit 5 = 32 kHz -> 0x0034.
/// * 0x02 Supported_Frame_Durations: bit 1 = 10 ms supported -> 0x02.
/// * 0x03 Supported_Audio_Channel_Counts: bit 0 = 1 channel -> 0x01.
/// * 0x04 Supported_Octets_Per_Codec_Frame: min u16, max u16 -> 26, 80.
/// * 0x05 Supported_Max_Codec_Frames_Per_SDU -> 1.
///
/// The record carries no metadata (Preferred_Audio_Contexts is optional).
const SOURCE_PAC: [u8; 27] = [
    0x01, // Number_of_PAC_records
    0x06, 0x00, 0x00, 0x00, 0x00, // Codec_ID: LC3
    0x13, // Codec_Specific_Capabilities_Length = 19
    0x03, 0x01, 0x34, 0x00, // Supported_Sampling_Frequencies: 16|24|32 kHz
    0x02, 0x02, 0x02, // Supported_Frame_Durations: 10 ms
    0x02, 0x03, 0x01, // Supported_Audio_Channel_Counts: 1
    0x05, 0x04, 0x1A, 0x00, 0x50, 0x00, // Supported_Octets_Per_Codec_Frame: 26..=80
    0x02, 0x05, 0x01, // Supported_Max_Codec_Frames_Per_SDU: 1
    0x00, // Metadata_Length
];

/// Length of the Source PAC value; `out` passed to [`source_pac`] must be at least this long.
pub const SOURCE_PAC_MAX_LEN: usize = SOURCE_PAC.len();

/// Source PAC characteristic value. Returns bytes written.
///
/// # Panics
/// If `out` is shorter than [`SOURCE_PAC_MAX_LEN`].
pub fn source_pac(out: &mut [u8]) -> usize {
    out[..SOURCE_PAC.len()].copy_from_slice(&SOURCE_PAC);
    SOURCE_PAC.len()
}

/// Source Audio Locations characteristic value (u32 LE bitfield).
pub fn source_audio_locations() -> [u8; 4] {
    SOURCE_AUDIO_LOCATIONS.to_le_bytes()
}

/// Available Audio Contexts characteristic value: Available_Sink_Contexts (u16 LE)
/// followed by Available_Source_Contexts (u16 LE).
pub fn available_audio_contexts() -> [u8; 4] {
    contexts_value(SINK_CONTEXTS, SOURCE_CONTEXTS)
}

/// Supported Audio Contexts characteristic value: Supported_Sink_Contexts (u16 LE)
/// followed by Supported_Source_Contexts (u16 LE).
pub fn supported_audio_contexts() -> [u8; 4] {
    contexts_value(SINK_CONTEXTS, SOURCE_CONTEXTS)
}

fn contexts_value(sink: u16, source: u16) -> [u8; 4] {
    let [s0, s1] = sink.to_le_bytes();
    let [r0, r1] = source.to_le_bytes();
    [s0, s1, r0, r1]
}
