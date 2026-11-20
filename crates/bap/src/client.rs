//! BAP Unicast Client side for driving a remote Source ASE (the client is the Audio
//! Sink): parses PACS/ASCS values read or notified by a Unicast Server and builds the
//! ASE Control Point writes.
//!
//! Like the server side this performs no I/O. A typical sequence for one Source ASE:
//! 1. Read the Source PAC: [`parse_pac`], then [`choose_codec_config`].
//! 2. Write [`config_codec`]; parse the Control Point notification with
//!    [`parse_cp_response`] and the ASE notification with [`parse_ase`]
//!    ([`AseStatus::CodecConfigured`]).
//! 3. [`derive_qos`] from the server's [`QosPreferences`], write [`config_qos`]
//!    ([`AseStatus::QosConfigured`]); create the CIG and CIS with those parameters.
//! 4. Write [`enable`] ([`AseStatus::Enabling`]), connect the CIS, set up the ISO data
//!    path (Controller to Host), then write [`receiver_start_ready`]
//!    ([`AseStatus::Streaming`]).
//! 5. Stop with [`disable`] ([`AseStatus::Disabling`]), remove the data path, write
//!    [`receiver_stop_ready`] (back to QoS Configured); tear down with [`release`].
//!
//! Response_Code and Reason values are in [`ascs::response`] and [`ascs::reason`];
//! opcodes in [`ascs::opcode`]. All multi-octet fields are little-endian.
//!
//! A Codec Configured ASE value is longer than the 20 octets that fit a notification
//! at the default ATT_MTU of 23; a truncated value parses as
//! [`ParseError::Truncated`], after which the client reads the characteristic (Read
//! Long) instead.

use crate::ascs::{self, AseState, CodecConfig, QosConfig, metadata_type, opcode};
use crate::wire::{Reader, Writer};
use crate::{codec_config, ltv, pacs};

/// Why a value could not be parsed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ParseError {
    /// The value ends inside a field or length-prefixed block.
    Truncated,
    /// Octets remain after the last field.
    TrailingData,
    /// An LTV overruns its block, or a known LTV has the wrong length.
    MalformedLtv,
    /// A mandatory LTV is missing (LC3 Codec_Specific_Configuration).
    MissingLtv,
    /// A field holds an undefined value (ASE_State, Sampling_Frequency or
    /// Frame_Duration code).
    InvalidValue,
}

/// Target_Latency values for Config Codec (ASCS v1.0.1 Table 5.2).
pub mod target_latency {
    pub const LOW_LATENCY: u8 = 0x01;
    pub const BALANCED: u8 = 0x02;
    pub const HIGH_RELIABILITY: u8 = 0x03;
}

/// Target_PHY values for Config Codec (ASCS v1.0.1 Table 5.2).
pub mod target_phy {
    pub const LE_1M: u8 = 0x01;
    pub const LE_2M: u8 = 0x02;
    pub const LE_CODED: u8 = 0x03;
}

/// PHY bitfield values (Preferred_PHY, Config QoS PHY).
pub mod phy {
    pub const LE_1M: u8 = 0x01;
    pub const LE_2M: u8 = 0x02;
    pub const LE_CODED: u8 = 0x04;
}

/// Framing values (Config QoS) and Preferred Framing (Codec Configured state).
pub mod framing {
    /// Config QoS: unframed ISOAL PDUs. Preference: unframed supported.
    pub const UNFRAMED: u8 = 0x00;
    /// Config QoS: framed ISOAL PDUs. Preference: unframed not supported.
    pub const FRAMED: u8 = 0x01;
}

/// Codec_Specific_Capabilities LTV types (Assigned Numbers, Generic Audio).
mod cap_type {
    pub const SUPPORTED_SAMPLING_FREQUENCIES: u8 = 0x01;
    pub const SUPPORTED_FRAME_DURATIONS: u8 = 0x02;
    pub const SUPPORTED_AUDIO_CHANNEL_COUNTS: u8 = 0x03;
    pub const SUPPORTED_OCTETS_PER_CODEC_FRAME: u8 = 0x04;
    pub const SUPPORTED_MAX_CODEC_FRAMES_PER_SDU: u8 = 0x05;
}

/// Upper bound of every PDU built here (Config Codec with the five-LTV LC3
/// configuration: 2 + 9 + 19 = 30).
pub const CP_PDU_MAX_LEN: usize = 2 + 9 + codec_config::ENCODED_LEN;

// ---------------------------------------------------------------------------
// PACS

/// A parsed Sink or Source PAC characteristic value (PACS v1.0 Table 3.2). Its
/// structure is fully validated by [`parse_pac`], so [`Pac::records`] cannot fail.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Pac<'a> {
    count: u8,
    records: &'a [u8],
}

/// One PAC record. The capability fields are decoded only for LC3 records (they are
/// vendor-defined otherwise) and are 0 when the LTV is absent, except
/// `channel_counts` and `max_frames_per_sdu`, which default to 1 channel / 1 frame
/// (Assigned Numbers, Generic Audio).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PacRecord<'a> {
    pub codec_id: [u8; 5],
    /// Raw Codec_Specific_Capabilities.
    pub codec_specific_capabilities: &'a [u8],
    /// Raw Metadata LTVs.
    pub metadata: &'a [u8],
    /// Supported_Sampling_Frequencies: bit `n` = Sampling_Frequency code `n + 1`
    /// (bit 2 16 kHz, bit 4 24 kHz, bit 5 32 kHz, bit 7 48 kHz, ...).
    pub sampling_frequencies: u16,
    /// Supported_Frame_Durations: bit 0 7.5 ms, bit 1 10 ms, bit 4 7.5 ms preferred,
    /// bit 5 10 ms preferred.
    pub frame_durations: u8,
    /// Supported_Audio_Channel_Counts: bit `n` = `n + 1` channels.
    pub channel_counts: u8,
    pub octets_per_frame_min: u16,
    pub octets_per_frame_max: u16,
    /// Supported_Max_Codec_Frames_Per_SDU (all channels together).
    pub max_frames_per_sdu: u8,
    /// Preferred_Audio_Contexts metadata, if present.
    pub preferred_contexts: Option<u16>,
}

/// Iterator over the records of a [`Pac`].
pub struct PacRecords<'a> {
    reader: Reader<'a>,
    left: u8,
}

/// Parses a Sink or Source PAC characteristic value: Number_of_PAC_records, then per
/// record Codec_ID (5 octets), Codec_Specific_Capabilities_Length, Codec_Specific_Capabilities,
/// Metadata_Length, Metadata.
pub fn parse_pac(value: &[u8]) -> Result<Pac<'_>, ParseError> {
    let mut r = Reader::new(value);
    let count = r.u8()?;
    let records = &value[1..];
    for _ in 0..count {
        parse_pac_record(&mut r)?;
    }
    r.finish()?;
    Ok(Pac { count, records })
}

impl<'a> Pac<'a> {
    /// Number_of_PAC_records.
    pub fn len(&self) -> usize {
        usize::from(self.count)
    }

    pub fn is_empty(&self) -> bool {
        self.count == 0
    }

    pub fn records(&self) -> PacRecords<'a> {
        PacRecords {
            reader: Reader::new(self.records),
            left: self.count,
        }
    }
}

impl<'a> Iterator for PacRecords<'a> {
    type Item = PacRecord<'a>;

    fn next(&mut self) -> Option<PacRecord<'a>> {
        self.left = self.left.checked_sub(1)?;
        // Validated by parse_pac.
        parse_pac_record(&mut self.reader).ok()
    }
}

impl PacRecord<'_> {
    pub fn is_lc3(&self) -> bool {
        self.codec_id == pacs::LC3_CODEC_ID
    }

    /// True if this is an LC3 record whose capabilities cover `codec`: sampling
    /// frequency, frame duration, channel count (from `audio_locations`, 0 = mono),
    /// octets per frame, and channels x blocks codec frames per SDU.
    pub fn supports(&self, codec: &CodecConfig) -> bool {
        let Some(sampling) = codec_config::sampling_code(codec.sampling_hz) else {
            return false;
        };
        let Some(duration) = codec_config::frame_duration_code(codec.frame_duration_us) else {
            return false;
        };
        let channels = codec_config::channel_count(codec.audio_locations);
        let frames = channels * u32::from(codec.frames_per_sdu);
        self.is_lc3()
            && self.sampling_frequencies & (1 << (sampling - 1)) != 0
            && self.frame_durations & (1 << duration) != 0
            && channels <= 8
            && self.channel_counts & (1 << (channels - 1)) != 0
            && (self.octets_per_frame_min..=self.octets_per_frame_max)
                .contains(&codec.octets_per_frame)
            && codec.frames_per_sdu >= 1
            && frames <= u32::from(self.max_frames_per_sdu)
    }
}

fn parse_pac_record<'a>(r: &mut Reader<'a>) -> Result<PacRecord<'a>, ParseError> {
    let codec_id = r.array::<5>()?;
    let caps = r.block()?;
    let metadata = r.block()?;
    let mut rec = PacRecord {
        codec_id,
        codec_specific_capabilities: caps,
        metadata,
        sampling_frequencies: 0,
        frame_durations: 0,
        channel_counts: 0x01,
        octets_per_frame_min: 0,
        octets_per_frame_max: 0,
        max_frames_per_sdu: 1,
        preferred_contexts: metadata_contexts(metadata, metadata_type::PREFERRED_AUDIO_CONTEXTS)?,
    };
    if rec.is_lc3() {
        for item in ltv::iter(caps) {
            let (ty, v) = item.map_err(|_| ParseError::MalformedLtv)?;
            match ty {
                cap_type::SUPPORTED_SAMPLING_FREQUENCIES => {
                    rec.sampling_frequencies = u16::from_le_bytes(fixed(v)?);
                }
                cap_type::SUPPORTED_FRAME_DURATIONS => rec.frame_durations = fixed::<1>(v)?[0],
                cap_type::SUPPORTED_AUDIO_CHANNEL_COUNTS => {
                    rec.channel_counts = fixed::<1>(v)?[0];
                }
                cap_type::SUPPORTED_OCTETS_PER_CODEC_FRAME => {
                    let [a, b, c, d] = fixed(v)?;
                    rec.octets_per_frame_min = u16::from_le_bytes([a, b]);
                    rec.octets_per_frame_max = u16::from_le_bytes([c, d]);
                }
                cap_type::SUPPORTED_MAX_CODEC_FRAMES_PER_SDU => {
                    rec.max_frames_per_sdu = fixed::<1>(v)?[0];
                }
                _ => {}
            }
        }
    }
    Ok(rec)
}

/// Picks an LC3 configuration with 10 ms frames, one codec frame block per SDU,
/// `sampling_hz`, `octets_per_frame` and Audio_Channel_Allocation `audio_locations`
/// (a location the server exposes in its Source Audio Locations, or 0 for mono
/// without a location) if any record of `pac` supports it.
pub fn choose_codec_config(
    pac: &Pac<'_>,
    sampling_hz: u32,
    octets_per_frame: u16,
    audio_locations: u32,
) -> Option<CodecConfig> {
    let codec = CodecConfig {
        sampling_hz,
        frame_duration_us: 10_000,
        octets_per_frame,
        frames_per_sdu: 1,
        audio_locations,
    };
    pac.records().any(|r| r.supports(&codec)).then_some(codec)
}

// ---------------------------------------------------------------------------
// ASE Control Point writes (ASCS v1.0.1 §5), one ASE per operation.
//
// Every builder writes into `out` and returns the length.
// # Panics: if `out` is shorter than the PDU; [`CP_PDU_MAX_LEN`] always suffices.

/// Config Codec (ASCS v1.0.1 Table 5.2) with LC3 and a Codec_Specific_Configuration
/// of all five LTVs: Sampling_Frequency, Frame_Duration, Audio_Channel_Allocation,
/// Octets_Per_Codec_Frame, Codec_Frame_Blocks_Per_SDU. Returns None (nothing
/// written) if `codec` has a sampling frequency or frame duration without a code.
///
/// # Panics
/// If `out` is shorter than [`CP_PDU_MAX_LEN`].
pub fn config_codec(
    out: &mut [u8],
    ase_id: u8,
    target_latency: u8,
    target_phy: u8,
    codec: &CodecConfig,
) -> Option<usize> {
    let mut cc = [0u8; codec_config::ENCODED_LEN];
    let cc_len = codec_config::encode(codec, &mut cc)?;
    let mut w = Writer::new(out);
    w.bytes(&[opcode::CONFIG_CODEC, 1, ase_id, target_latency, target_phy]);
    w.bytes(&pacs::LC3_CODEC_ID);
    w.u8(cc_len as u8);
    w.bytes(&cc[..cc_len]);
    Some(w.pos)
}

/// Config QoS (ASCS v1.0.1 Table 5.3) for `qos` (CIG_ID and CIS_ID included).
/// 24-bit fields keep only their low 24 bits.
///
/// # Panics
/// If `out` is shorter than 18 octets.
pub fn config_qos(out: &mut [u8], ase_id: u8, qos: &QosConfig) -> usize {
    let mut w = Writer::new(out);
    w.bytes(&[opcode::CONFIG_QOS, 1, ase_id, qos.cig_id, qos.cis_id]);
    w.le24(qos.sdu_interval_us);
    w.u8(qos.framing);
    w.u8(qos.phy);
    w.le16(qos.max_sdu);
    w.u8(qos.retransmission_number);
    w.le16(qos.max_transport_latency_ms);
    w.le24(qos.presentation_delay_us);
    w.pos
}

/// Enable (ASCS v1.0.1 Table 5.4) with a Streaming_Audio_Contexts metadata LTV (a
/// [`context`](crate::context) bitfield; it has to be nonzero and a subset of the
/// server's Available Source Contexts).
///
/// # Panics
/// If `out` is shorter than 8 octets.
pub fn enable(out: &mut [u8], ase_id: u8, streaming_contexts: u16) -> usize {
    let mut w = Writer::new(out);
    w.bytes(&[opcode::ENABLE, 1, ase_id, 4]);
    w.bytes(&[3, metadata_type::STREAMING_AUDIO_CONTEXTS]);
    w.le16(streaming_contexts);
    w.pos
}

/// Receiver Start Ready (ASCS v1.0.1 Table 5.5): the client is ready to receive on
/// the Source ASE's CIS.
///
/// # Panics
/// If `out` is shorter than 3 octets.
pub fn receiver_start_ready(out: &mut [u8], ase_id: u8) -> usize {
    ase_only(out, opcode::RECEIVER_START_READY, ase_id)
}

/// Disable (ASCS v1.0.1 Table 5.6).
///
/// # Panics
/// If `out` is shorter than 3 octets.
pub fn disable(out: &mut [u8], ase_id: u8) -> usize {
    ase_only(out, opcode::DISABLE, ase_id)
}

/// Receiver Stop Ready (ASCS v1.0.1 Table 5.7): completes Disabling of a Source ASE.
///
/// # Panics
/// If `out` is shorter than 3 octets.
pub fn receiver_stop_ready(out: &mut [u8], ase_id: u8) -> usize {
    ase_only(out, opcode::RECEIVER_STOP_READY, ase_id)
}

/// Release (ASCS v1.0.1 Table 5.9).
///
/// # Panics
/// If `out` is shorter than 3 octets.
pub fn release(out: &mut [u8], ase_id: u8) -> usize {
    ase_only(out, opcode::RELEASE, ase_id)
}

fn ase_only(out: &mut [u8], op: u8, ase_id: u8) -> usize {
    out[..3].copy_from_slice(&[op, 1, ase_id]);
    3
}

// ---------------------------------------------------------------------------
// ASE characteristic values

/// A parsed Sink or Source ASE characteristic value (ASCS v1.0.1 Table 4.2).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct AseValue<'a> {
    pub ase_id: u8,
    pub status: AseStatus<'a>,
}

/// ASE_State with its Additional_ASE_Parameters (ASCS v1.0.1 Tables 4.3-4.5).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AseStatus<'a> {
    Idle,
    CodecConfigured(CodecConfigured<'a>),
    /// CIG_ID, CIS_ID and the QoS configuration in effect.
    QosConfigured(QosConfig),
    Enabling(StreamParams<'a>),
    Streaming(StreamParams<'a>),
    Disabling(StreamParams<'a>),
    Releasing,
}

/// Codec Configured parameters (ASCS v1.0.1 Table 4.3).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CodecConfigured<'a> {
    pub preferences: QosPreferences,
    pub codec_id: [u8; 5],
    /// Raw Codec_Specific_Configuration.
    pub codec_config: &'a [u8],
    /// Decoded configuration for LC3 (`None` for other codecs).
    pub codec: Option<CodecConfig>,
}

/// Server QoS preferences in the Codec Configured state (ASCS v1.0.1 Table 4.3).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct QosPreferences {
    /// [`framing::UNFRAMED`]: unframed supported; [`framing::FRAMED`]: unframed not
    /// supported.
    pub framing: u8,
    /// Preferred_PHY bitfield ([`phy`]); 0 = no preference.
    pub phy: u8,
    pub retransmission_number: u8,
    pub max_transport_latency_ms: u16,
    pub presentation_delay_min_us: u32,
    pub presentation_delay_max_us: u32,
    /// 0 = no preference.
    pub preferred_presentation_delay_min_us: u32,
    /// 0 = no preference.
    pub preferred_presentation_delay_max_us: u32,
}

/// Enabling / Streaming / Disabling parameters (ASCS v1.0.1 Table 4.5).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct StreamParams<'a> {
    pub cig_id: u8,
    pub cis_id: u8,
    /// Raw Metadata LTVs.
    pub metadata: &'a [u8],
    /// Streaming_Audio_Contexts metadata, if present.
    pub streaming_contexts: Option<u16>,
}

impl AseStatus<'_> {
    pub fn state(&self) -> AseState {
        match self {
            Self::Idle => AseState::Idle,
            Self::CodecConfigured(_) => AseState::CodecConfigured,
            Self::QosConfigured(_) => AseState::QosConfigured,
            Self::Enabling(_) => AseState::Enabling,
            Self::Streaming(_) => AseState::Streaming,
            Self::Disabling(_) => AseState::Disabling,
            Self::Releasing => AseState::Releasing,
        }
    }
}

/// Parses an ASE characteristic value (read or notified).
pub fn parse_ase(value: &[u8]) -> Result<AseValue<'_>, ParseError> {
    let mut r = Reader::new(value);
    let ase_id = r.u8()?;
    let status = match r.u8()? {
        0x00 => AseStatus::Idle,
        0x01 => {
            let preferences = QosPreferences {
                framing: r.u8()?,
                phy: r.u8()?,
                retransmission_number: r.u8()?,
                max_transport_latency_ms: r.le16()?,
                presentation_delay_min_us: r.le24()?,
                presentation_delay_max_us: r.le24()?,
                preferred_presentation_delay_min_us: r.le24()?,
                preferred_presentation_delay_max_us: r.le24()?,
            };
            let codec_id = r.array::<5>()?;
            let codec_config = r.block()?;
            let codec = if codec_id == pacs::LC3_CODEC_ID {
                Some(codec_config::decode(codec_config)?)
            } else {
                None
            };
            AseStatus::CodecConfigured(CodecConfigured {
                preferences,
                codec_id,
                codec_config,
                codec,
            })
        }
        0x02 => AseStatus::QosConfigured(QosConfig {
            cig_id: r.u8()?,
            cis_id: r.u8()?,
            sdu_interval_us: r.le24()?,
            framing: r.u8()?,
            phy: r.u8()?,
            max_sdu: r.le16()?,
            retransmission_number: r.u8()?,
            max_transport_latency_ms: r.le16()?,
            presentation_delay_us: r.le24()?,
        }),
        state @ 0x03..=0x05 => {
            let cig_id = r.u8()?;
            let cis_id = r.u8()?;
            let metadata = r.block()?;
            let params = StreamParams {
                cig_id,
                cis_id,
                metadata,
                streaming_contexts: metadata_contexts(
                    metadata,
                    metadata_type::STREAMING_AUDIO_CONTEXTS,
                )?,
            };
            match state {
                0x03 => AseStatus::Enabling(params),
                0x04 => AseStatus::Streaming(params),
                _ => AseStatus::Disabling(params),
            }
        }
        0x06 => AseStatus::Releasing,
        _ => return Err(ParseError::InvalidValue),
    };
    r.finish()?;
    Ok(AseValue { ase_id, status })
}

// ---------------------------------------------------------------------------
// ASE Control Point notifications

/// A parsed ASE Control Point notification (ASCS v1.0.1 Table 4.7).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CpResponse<'a> {
    /// Opcode of the operation answered (0x00 for an empty write).
    pub opcode: u8,
    operation_error: bool,
    entries: &'a [u8],
}

/// Response for one ASE (ASE_ID 0x00 for an operation-level error).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CpEntry {
    pub ase_id: u8,
    /// [`ascs::response`] value.
    pub response_code: u8,
    /// [`ascs::reason`] value, or the Metadata Type for metadata response codes.
    pub reason: u8,
}

impl CpEntry {
    pub fn is_success(&self) -> bool {
        self.response_code == ascs::response::SUCCESS
    }
}

/// Parses an ASE Control Point notification: Opcode, Number_of_ASEs, then
/// (ASE_ID, Response_Code, Reason) per ASE. Number_of_ASEs 0xFF (Unsupported Opcode
/// or Invalid Length) carries exactly one entry, with ASE_ID 0x00.
pub fn parse_cp_response(value: &[u8]) -> Result<CpResponse<'_>, ParseError> {
    let mut r = Reader::new(value);
    let opcode = r.u8()?;
    let num = r.u8()?;
    let operation_error = num == 0xFF;
    let entries = r.bytes(if operation_error {
        3
    } else {
        3 * usize::from(num)
    })?;
    r.finish()?;
    Ok(CpResponse {
        opcode,
        operation_error,
        entries,
    })
}

impl<'a> CpResponse<'a> {
    /// True for Number_of_ASEs 0xFF: the whole operation was refused.
    pub fn is_operation_error(&self) -> bool {
        self.operation_error
    }

    pub fn entries(&self) -> impl ExactSizeIterator<Item = CpEntry> + 'a {
        let (entries, _) = self.entries.as_chunks::<3>();
        entries
            .iter()
            .map(|&[ase_id, response_code, reason]| CpEntry {
                ase_id,
                response_code,
                reason,
            })
    }

    /// The entry for `ase_id`; for an operation error, the single entry regardless
    /// of `ase_id`.
    pub fn entry(&self, ase_id: u8) -> Option<CpEntry> {
        let mut entries = self.entries();
        if self.operation_error {
            entries.next()
        } else {
            entries.find(|e| e.ase_id == ase_id)
        }
    }
}

// ---------------------------------------------------------------------------
// QoS

/// QoS configuration for a Codec Configured ASE from its server preferences:
/// * SDU_Interval: one SDU per Codec_Frame_Blocks_Per_SDU frames (10 000 µs for
///   10 ms frames and one block);
/// * Framing: unframed, unless the server does not support unframed PDUs;
/// * PHY: LE 2M if preferred (or no preference), else LE 1M, else LE Coded;
/// * Max_SDU: octets per frame x channels x blocks;
/// * Retransmission_Number and Max_Transport_Latency: the server's preferences
///   (latency clamped to the valid 5..=4000 ms);
/// * Presentation_Delay: the server's preferred minimum if it has one, else its
///   minimum, kept within its supported range.
pub fn derive_qos(
    prefs: &QosPreferences,
    codec: &CodecConfig,
    cig_id: u8,
    cis_id: u8,
) -> QosConfig {
    let chosen_framing = if prefs.framing == framing::FRAMED {
        framing::FRAMED
    } else {
        framing::UNFRAMED
    };
    let chosen_phy = if prefs.phy == 0 || prefs.phy & phy::LE_2M != 0 {
        phy::LE_2M
    } else if prefs.phy & phy::LE_1M != 0 {
        phy::LE_1M
    } else {
        phy::LE_CODED
    };
    let max_sdu = u32::from(codec.octets_per_frame)
        * codec_config::channel_count(codec.audio_locations)
        * u32::from(codec.frames_per_sdu);
    let pd_min = prefs.presentation_delay_min_us;
    let pd_max = prefs.presentation_delay_max_us.max(pd_min);
    let pd = match prefs.preferred_presentation_delay_min_us {
        0 => pd_min,
        preferred => preferred,
    };
    QosConfig {
        cig_id,
        cis_id,
        sdu_interval_us: codec.frame_duration_us * u32::from(codec.frames_per_sdu),
        framing: chosen_framing,
        phy: chosen_phy,
        max_sdu: max_sdu.min(u32::from(u16::MAX)) as u16,
        retransmission_number: prefs.retransmission_number,
        max_transport_latency_ms: prefs.max_transport_latency_ms.clamp(5, 4000),
        presentation_delay_us: pd.clamp(pd_min, pd_max),
    }
}

// ---------------------------------------------------------------------------

/// The 2-octet context bitfield of metadata LTV type `ty`, if present.
fn metadata_contexts(md: &[u8], ty: u8) -> Result<Option<u16>, ParseError> {
    let mut found = None;
    for item in ltv::iter(md) {
        let (t, v) = item.map_err(|_| ParseError::MalformedLtv)?;
        if t == ty {
            found = Some(u16::from_le_bytes(fixed(v)?));
        }
    }
    Ok(found)
}

fn fixed<const N: usize>(v: &[u8]) -> Result<[u8; N], ParseError> {
    v.try_into().map_err(|_| ParseError::MalformedLtv)
}
