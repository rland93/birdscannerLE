//! LC3 Codec_Specific_Configuration LTVs (BAP v1.0 §4.3.2, Assigned Numbers, Generic
//! Audio "Codec_Specific_Configuration LTV structures"): decoding for both sides,
//! encoding for the client.

use crate::ascs::CodecConfig;
use crate::client::ParseError;
use crate::{ltv, wire};

/// Codec_Specific_Configuration LTV types.
pub(crate) mod ltv_type {
    pub const SAMPLING_FREQUENCY: u8 = 0x01;
    pub const FRAME_DURATION: u8 = 0x02;
    pub const AUDIO_CHANNEL_ALLOCATION: u8 = 0x03;
    pub const OCTETS_PER_CODEC_FRAME: u8 = 0x04;
    pub const CODEC_FRAME_BLOCKS_PER_SDU: u8 = 0x05;
}

/// Sampling frequencies in Hz indexed by Sampling_Frequency code - 1 (codes
/// 0x01..=0x0D). Bit `code - 1` is the same frequency in the
/// Supported_Sampling_Frequencies capability bitfield.
const SAMPLING_HZ: [u32; 13] = [
    8_000, 11_025, 16_000, 22_050, 24_000, 32_000, 44_100, 48_000, 88_200, 96_000, 176_400,
    192_000, 384_000,
];

/// Sampling_Frequency code for `hz`.
pub(crate) fn sampling_code(hz: u32) -> Option<u8> {
    SAMPLING_HZ
        .iter()
        .position(|&f| f == hz)
        .map(|i| i as u8 + 1)
}

/// Frame_Duration code: 0x00 7.5 ms, 0x01 10 ms.
pub(crate) fn frame_duration_code(us: u32) -> Option<u8> {
    match us {
        7_500 => Some(0x00),
        10_000 => Some(0x01),
        _ => None,
    }
}

/// Number of audio channels an Audio_Channel_Allocation describes (0 = mono without
/// a location).
pub(crate) fn channel_count(audio_locations: u32) -> u32 {
    audio_locations.count_ones().max(1)
}

/// Length of [`encode`]'s output: all five LTVs.
pub(crate) const ENCODED_LEN: usize = 3 + 3 + 6 + 4 + 3;

/// Decodes an LC3 Codec_Specific_Configuration. Sampling_Frequency, Frame_Duration
/// and Octets_Per_Codec_Frame are mandatory; Audio_Channel_Allocation defaults to 0
/// and Codec_Frame_Blocks_Per_SDU to 1. Unknown LTV types are ignored.
pub(crate) fn decode(cc: &[u8]) -> Result<CodecConfig, ParseError> {
    let mut sampling = None;
    let mut duration = None;
    let mut allocation = 0u32;
    let mut octets = None;
    let mut blocks = 1u8;
    for item in ltv::iter(cc) {
        let (ty, v) = item.map_err(|_| ParseError::MalformedLtv)?;
        match ty {
            ltv_type::SAMPLING_FREQUENCY => sampling = Some(*one(v)?),
            ltv_type::FRAME_DURATION => duration = Some(*one(v)?),
            ltv_type::AUDIO_CHANNEL_ALLOCATION => {
                let v: [u8; 4] = v.try_into().map_err(|_| ParseError::MalformedLtv)?;
                allocation = u32::from_le_bytes(v);
            }
            ltv_type::OCTETS_PER_CODEC_FRAME => {
                let v: [u8; 2] = v.try_into().map_err(|_| ParseError::MalformedLtv)?;
                octets = Some(u16::from_le_bytes(v));
            }
            ltv_type::CODEC_FRAME_BLOCKS_PER_SDU => blocks = *one(v)?,
            _ => {}
        }
    }
    let (Some(sampling), Some(duration), Some(octets)) = (sampling, duration, octets) else {
        return Err(ParseError::MissingLtv);
    };
    let sampling_hz = *usize::from(sampling)
        .checked_sub(1)
        .and_then(|i| SAMPLING_HZ.get(i))
        .ok_or(ParseError::InvalidValue)?;
    let frame_duration_us = match duration {
        0x00 => 7_500,
        0x01 => 10_000,
        _ => return Err(ParseError::InvalidValue),
    };
    Ok(CodecConfig {
        sampling_hz,
        frame_duration_us,
        octets_per_frame: octets,
        frames_per_sdu: blocks,
        audio_locations: allocation,
    })
}

/// Encodes all five LTVs ([`ENCODED_LEN`] octets) into `out`. None if the sampling
/// frequency or frame duration has no code.
///
/// # Panics
/// If `out` is shorter than [`ENCODED_LEN`].
pub(crate) fn encode(codec: &CodecConfig, out: &mut [u8]) -> Option<usize> {
    let sampling = sampling_code(codec.sampling_hz)?;
    let duration = frame_duration_code(codec.frame_duration_us)?;
    let mut w = wire::Writer::new(out);
    w.bytes(&[2, ltv_type::SAMPLING_FREQUENCY, sampling]);
    w.bytes(&[2, ltv_type::FRAME_DURATION, duration]);
    w.bytes(&[5, ltv_type::AUDIO_CHANNEL_ALLOCATION]);
    w.le32(codec.audio_locations);
    w.bytes(&[3, ltv_type::OCTETS_PER_CODEC_FRAME]);
    w.le16(codec.octets_per_frame);
    w.bytes(&[
        2,
        ltv_type::CODEC_FRAME_BLOCKS_PER_SDU,
        codec.frames_per_sdu,
    ]);
    Some(w.pos)
}

fn one(v: &[u8]) -> Result<&u8, ParseError> {
    wire::one(v).ok_or(ParseError::MalformedLtv)
}
