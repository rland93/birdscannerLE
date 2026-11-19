//! Audio Stream Control Service (ASCS v1.0.1) for a server exposing only Source ASEs.
//!
//! [`Ascs`] holds the ASE state machines. GATT/HCI events are fed in; every call
//! appends its consequences to an [`Effects`]: ASEs whose characteristic value changed
//! (to be notified) and [`Action`]s for the integrator to perform.
//!
//! Integration contract:
//! * ASE Control Point write: call [`Ascs::control_point`]; notify the returned
//!   response on the ASE Control Point first, then notify [`Ascs::ase_value`] for
//!   every id in [`Effects::changed_ases`] (ASCS v1.0.1 §5 requires that order), then
//!   perform [`Effects::actions`] in order.
//! * If [`Effects::has_deferred`] is set, call [`Ascs::complete_deferred`] after those
//!   notifications were queued, and handle its `Effects` the same way. This lets the
//!   client observe Releasing before the ASE moves on to Codec Configured.
//! * HCI LE CIS Request: accept iff [`Ascs::cis_request`] returns true.
//! * HCI LE CIS Established with status 0: [`Ascs::cis_established`]. The LE CIS
//!   Established event (v1) carries no CIG_ID/CIS_ID, so remember them per CIS
//!   handle from the LE CIS Request. A failed establishment needs no call.
//! * HCI Disconnection Complete for a CIS handle: [`Ascs::cis_disconnected`].
//! * ACL Disconnection Complete: [`Ascs::acl_disconnected`].
//! * Clear the `Effects` (or use a fresh one) after handling each call; at most
//!   [`MAX_ACTIONS`] actions can be held and one call never emits more than that.
//!
//! All multi-octet fields are little-endian.

use crate::{ltv, pacs};

/// Maximum number of Source ASEs (`N`) supported by [`Ascs`]. Bounded so that one call
/// never produces more than [`MAX_ACTIONS`] actions (at most two per ASE).
pub const MAX_ASES: usize = 4;
/// Capacity of [`Effects::actions`].
pub const MAX_ACTIONS: usize = 8;
/// Largest Codec_Specific_Configuration stored per ASE. A full LC3 configuration
/// (all five LTVs) is 19 octets; longer requests are answered with Insufficient
/// Resources.
pub const MAX_CODEC_CONFIG_LEN: usize = 32;
/// Largest Metadata stored per ASE; longer requests are answered with Insufficient
/// Resources.
pub const MAX_METADATA_LEN: usize = 32;

/// Additional_ASE_Parameters in Codec Configured state, excluding the
/// Codec_Specific_Configuration (ASCS v1.0.1 Table 4.3): 1+1+1+2+3+3+3+3+5+1.
const CODEC_CONFIGURED_FIXED_LEN: usize = 23;
/// Upper bound of an ASE characteristic value (Codec Configured state with the
/// largest stored configuration: 2 + 23 + 32 = 57). This fits a notification at the
/// ATT_MTU of 64 that BAP v1.0 §3.5.1 requires a Unicast Server to support; with the
/// default ATT_MTU of 23 the client has to use Read Long.
pub const ASE_VALUE_MAX_LEN: usize = 2 + CODEC_CONFIGURED_FIXED_LEN + MAX_CODEC_CONFIG_LEN;
/// Upper bound of an ASE Control Point notification: Opcode, Number_of_ASEs and one
/// 3-octet entry per ASE (requests naming more than `N` ASEs are rejected as Invalid
/// Length, as Zephyr does, because the response could not cover them).
pub const CP_RESPONSE_MAX_LEN: usize = 2 + 3 * MAX_ASES;

/// Server QoS preferences and limits exposed in the Codec Configured state
/// (ASCS v1.0.1 Table 4.3).
pub mod qos_pref {
    /// 0x00: unframed ISOAL PDUs supported (framed is accepted too).
    pub const FRAMING: u8 = 0x00;
    /// Preferred_PHY bitfield: LE 1M (bit 0) | LE 2M (bit 1).
    pub const PHY: u8 = 0x03;
    /// Preferred_Retransmission_Number: 2, the BAP v1.0 Table 5.2 low-latency value for
    /// 16_2_1, 24_2_1 and 32_2_1.
    pub const RETRANSMISSION_NUMBER: u8 = 2;
    /// Max_Transport_Latency (ms): 10, the BAP v1.0 Table 5.2 low-latency value for the
    /// 10 ms settings. ASCS defines this as the maximum the server supports; Config QoS
    /// is still not rejected above it (the encoder is unaffected by transport latency),
    /// so clients insisting on high-reliability settings keep working.
    pub const MAX_TRANSPORT_LATENCY_MS: u16 = 10;
    /// Presentation_Delay_Min (µs). For a Source ASE the presentation delay is the time
    /// from capture to the SDU reference point; a 10 ms LC3 frame cannot be delivered
    /// sooner than one frame after its first sample was captured.
    pub const PRESENTATION_DELAY_MIN_US: u32 = 10_000;
    /// Presentation_Delay_Max (µs): includes the 40 ms that all BAP v1.0 Table 5.2 QoS
    /// settings use (mandatory only for Sink ASEs, footnote 4, but clients commonly
    /// request it for sources as well).
    pub const PRESENTATION_DELAY_MAX_US: u32 = 40_000;
    /// Preferred_Presentation_Delay_Min: 0 = no preference.
    pub const PREFERRED_PRESENTATION_DELAY_MIN_US: u32 = 0;
    /// Preferred_Presentation_Delay_Max: 0 = no preference.
    pub const PREFERRED_PRESENTATION_DELAY_MAX_US: u32 = 0;
}

/// ASE Control Point opcodes (ASCS v1.0.1 Table 4.6).
pub mod opcode {
    pub const CONFIG_CODEC: u8 = 0x01;
    pub const CONFIG_QOS: u8 = 0x02;
    pub const ENABLE: u8 = 0x03;
    pub const RECEIVER_START_READY: u8 = 0x04;
    pub const DISABLE: u8 = 0x05;
    pub const RECEIVER_STOP_READY: u8 = 0x06;
    pub const UPDATE_METADATA: u8 = 0x07;
    pub const RELEASE: u8 = 0x08;
}

/// ASE Control Point Response_Code values (ASCS v1.0.1 Table 5.1).
pub mod response {
    pub const SUCCESS: u8 = 0x00;
    pub const UNSUPPORTED_OPCODE: u8 = 0x01;
    pub const INVALID_LENGTH: u8 = 0x02;
    pub const INVALID_ASE_ID: u8 = 0x03;
    pub const INVALID_ASE_STATE_MACHINE_TRANSITION: u8 = 0x04;
    pub const INVALID_ASE_DIRECTION: u8 = 0x05;
    pub const UNSUPPORTED_AUDIO_CAPABILITIES: u8 = 0x06;
    pub const UNSUPPORTED_CONFIGURATION_PARAMETER_VALUE: u8 = 0x07;
    pub const REJECTED_CONFIGURATION_PARAMETER_VALUE: u8 = 0x08;
    pub const INVALID_CONFIGURATION_PARAMETER_VALUE: u8 = 0x09;
    pub const UNSUPPORTED_METADATA: u8 = 0x0A;
    pub const REJECTED_METADATA: u8 = 0x0B;
    pub const INVALID_METADATA: u8 = 0x0C;
    pub const INSUFFICIENT_RESOURCES: u8 = 0x0D;
    pub const UNSPECIFIED_ERROR: u8 = 0x0E;
}

/// Reason values for the configuration-parameter Response_Codes 0x07..=0x09
/// (ASCS v1.0.1 Table 5.1). For the metadata Response_Codes 0x0A..=0x0C the Reason is
/// the Metadata Type in error instead.
pub mod reason {
    pub const NONE: u8 = 0x00;
    pub const CODEC_ID: u8 = 0x01;
    pub const CODEC_SPECIFIC_CONFIGURATION: u8 = 0x02;
    pub const SDU_INTERVAL: u8 = 0x03;
    pub const FRAMING: u8 = 0x04;
    pub const PHY: u8 = 0x05;
    pub const MAXIMUM_SDU_SIZE: u8 = 0x06;
    pub const RETRANSMISSION_NUMBER: u8 = 0x07;
    pub const MAX_TRANSPORT_LATENCY: u8 = 0x08;
    pub const PRESENTATION_DELAY: u8 = 0x09;
    pub const INVALID_ASE_CIS_MAPPING: u8 = 0x0A;
}

/// Codec_Specific_Configuration LTV types (Assigned Numbers, Generic Audio).
mod cc_type {
    pub const SAMPLING_FREQUENCY: u8 = 0x01;
    pub const FRAME_DURATION: u8 = 0x02;
    pub const AUDIO_CHANNEL_ALLOCATION: u8 = 0x03;
    pub const OCTETS_PER_CODEC_FRAME: u8 = 0x04;
    pub const CODEC_FRAME_BLOCKS_PER_SDU: u8 = 0x05;
}

/// Metadata LTV types (Assigned Numbers, Generic Audio) validated by this server; any
/// other type is accepted and stored verbatim.
mod metadata_type {
    pub const PREFERRED_AUDIO_CONTEXTS: u8 = 0x01;
    pub const STREAMING_AUDIO_CONTEXTS: u8 = 0x02;
}

use response::*;

/// ASE_State values (ASCS v1.0.1 Table 3.1); the discriminant is the on-air value.
#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AseState {
    Idle = 0x00,
    CodecConfigured = 0x01,
    QosConfigured = 0x02,
    Enabling = 0x03,
    Streaming = 0x04,
    Disabling = 0x05,
    Releasing = 0x06,
}

/// Decoded Codec_Specific_Configuration accepted by Config Codec.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CodecConfig {
    pub sampling_hz: u32,
    pub frame_duration_us: u32,
    pub octets_per_frame: u16,
    /// Codec_Frame_Blocks_Per_SDU (1 when absent).
    pub frames_per_sdu: u8,
    /// Audio_Channel_Allocation (0 = mono without location, also used when absent).
    pub audio_locations: u32,
}

impl CodecConfig {
    const EMPTY: Self = Self {
        sampling_hz: 0,
        frame_duration_us: 0,
        octets_per_frame: 0,
        frames_per_sdu: 0,
        audio_locations: 0,
    };
}

/// Parameters accepted by Config QoS (ASCS v1.0.1 Table 5.3).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct QosConfig {
    pub cig_id: u8,
    pub cis_id: u8,
    pub sdu_interval_us: u32,
    /// 0x00 unframed, 0x01 framed.
    pub framing: u8,
    /// PHY bitfield: bit 0 LE 1M, bit 1 LE 2M, bit 2 LE Coded.
    pub phy: u8,
    pub max_sdu: u16,
    pub retransmission_number: u8,
    pub max_transport_latency_ms: u16,
    pub presentation_delay_us: u32,
}

impl QosConfig {
    const EMPTY: Self = Self {
        cig_id: 0,
        cis_id: 0,
        sdu_interval_us: 0,
        framing: 0,
        phy: 0,
        max_sdu: 0,
        retransmission_number: 0,
        max_transport_latency_ms: 0,
        presentation_delay_us: 0,
    };
}

/// Work for the integrator, in the order it is to be performed.
///
/// Accepting an LE CIS Request needs no action: accept it directly when
/// [`Ascs::cis_request`] returns true.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Action {
    /// A CIS was established for an ASE: send HCI LE Setup ISO Data Path for this
    /// handle (data path direction input / Host to Controller, data path ID 0x00 = HCI,
    /// codec ID transparent 0x03). Must be done before the next ASE Control Point
    /// write is handled, because Receiver Start Ready relies on it. The data path stays
    /// in place across StopStream/StartStream until the CIS goes away.
    SetupIsoDataPath { cis_handle: u16 },
    /// Start producing one LC3 SDU per `qos.sdu_interval_us` on this CIS.
    StartStream {
        ase_id: u8,
        cis_handle: u16,
        codec: CodecConfig,
        qos: QosConfig,
    },
    /// Stop producing SDUs for this ASE.
    StopStream { ase_id: u8 },
    /// Server-initiated CIS teardown: send HCI Disconnect for this CIS handle
    /// (reason 0x13 Remote User Terminated Connection is appropriate).
    DisconnectCis { cis_handle: u16 },
}

/// Consequences of one [`Ascs`] call.
pub struct Effects {
    /// Bit `i` set: ASE `i` changed and must be notified.
    changed: u32,
    actions: [Action; MAX_ACTIONS],
    len: usize,
    deferred: bool,
}

impl Default for Effects {
    fn default() -> Self {
        Self::new()
    }
}

impl Effects {
    pub const fn new() -> Self {
        Self {
            changed: 0,
            actions: [Action::StopStream { ase_id: 0 }; MAX_ACTIONS],
            len: 0,
            deferred: false,
        }
    }

    /// ASE ids whose characteristic value changed, ascending.
    pub fn changed_ases(&self) -> impl Iterator<Item = u8> + '_ {
        (1..=MAX_ASES as u8).filter(move |id| self.changed & (1 << id) != 0)
    }

    /// Actions to perform, in order.
    pub fn actions(&self) -> &[Action] {
        &self.actions[..self.len]
    }

    /// True if [`Ascs::complete_deferred`] has to be called once the notifications for
    /// these effects have been queued.
    pub fn has_deferred(&self) -> bool {
        self.deferred
    }

    pub fn clear(&mut self) {
        *self = Self::new();
    }

    fn mark(&mut self, ase_id: u8) {
        self.changed |= 1 << ase_id;
    }

    fn push(&mut self, action: Action) {
        assert!(
            self.len < MAX_ACTIONS,
            "bap::ascs::Effects is full: clear it after handling each call"
        );
        self.actions[self.len] = action;
        self.len += 1;
    }
}

/// Per-ASE state. Which fields are meaningful depends on `state`:
/// codec in Codec Configured..=Disabling and Releasing (cached for Released),
/// qos in QoS Configured..=Disabling, metadata in Enabling..=Disabling.
#[derive(Clone, Copy)]
struct Ase {
    state: AseState,
    codec: CodecConfig,
    cc: [u8; MAX_CODEC_CONFIG_LEN],
    cc_len: u8,
    qos: QosConfig,
    metadata: [u8; MAX_METADATA_LEN],
    metadata_len: u8,
    /// Established CIS coupled to this ASE.
    cis_handle: Option<u16>,
    /// Releasing without a CIS; finished by `complete_deferred`.
    release_pending: bool,
}

impl Ase {
    const IDLE: Self = Self {
        state: AseState::Idle,
        codec: CodecConfig::EMPTY,
        cc: [0; MAX_CODEC_CONFIG_LEN],
        cc_len: 0,
        qos: QosConfig::EMPTY,
        metadata: [0; MAX_METADATA_LEN],
        metadata_len: 0,
        cis_handle: None,
        release_pending: false,
    };

    fn has_qos(&self) -> bool {
        matches!(
            self.state,
            AseState::QosConfigured
                | AseState::Enabling
                | AseState::Streaming
                | AseState::Disabling
        )
    }

    fn cis_ids(&self) -> (u8, u8) {
        (self.qos.cig_id, self.qos.cis_id)
    }
}

/// Response_Code and Reason for one ASE entry.
type Rsp = (u8, u8);
const OK: Rsp = (SUCCESS, reason::NONE);
const BAD_TRANSITION: Rsp = (INVALID_ASE_STATE_MACHINE_TRANSITION, reason::NONE);

/// `N` Source ASEs with ASE_IDs `1..=N` (`1 <= N <= MAX_ASES`).
pub struct Ascs<const N: usize> {
    ases: [Ase; N],
}

impl<const N: usize> Default for Ascs<N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const N: usize> Ascs<N> {
    const VALID_N: () = assert!(
        N >= 1 && N <= MAX_ASES,
        "Ascs<N>: N must be in 1..=MAX_ASES"
    );

    pub const fn new() -> Self {
        let () = Self::VALID_N;
        Self {
            ases: [Ase::IDLE; N],
        }
    }

    /// ASE_IDs of the Source ASE characteristics to expose, one characteristic each.
    pub fn ase_ids() -> impl Iterator<Item = u8> {
        1..=N as u8
    }

    pub fn state(&self, ase_id: u8) -> Option<AseState> {
        self.index(ase_id).map(|i| self.ases[i].state)
    }

    /// Source ASE characteristic value (read and notify), ASCS v1.0.1 Table 4.2 with the
    /// state-specific Additional_ASE_Parameters (Tables 4.3-4.5). Returns bytes written,
    /// 0 for an unknown `ase_id`.
    ///
    /// # Panics
    /// If `out` is shorter than the value; [`ASE_VALUE_MAX_LEN`] always suffices.
    pub fn ase_value(&self, ase_id: u8, out: &mut [u8]) -> usize {
        let Some(i) = self.index(ase_id) else {
            return 0;
        };
        let ase = &self.ases[i];
        let mut w = Writer { buf: out, pos: 0 };
        w.u8(ase_id);
        w.u8(ase.state as u8);
        match ase.state {
            AseState::Idle | AseState::Releasing => {}
            AseState::CodecConfigured => {
                w.u8(qos_pref::FRAMING);
                w.u8(qos_pref::PHY);
                w.u8(qos_pref::RETRANSMISSION_NUMBER);
                w.le16(qos_pref::MAX_TRANSPORT_LATENCY_MS);
                w.le24(qos_pref::PRESENTATION_DELAY_MIN_US);
                w.le24(qos_pref::PRESENTATION_DELAY_MAX_US);
                w.le24(qos_pref::PREFERRED_PRESENTATION_DELAY_MIN_US);
                w.le24(qos_pref::PREFERRED_PRESENTATION_DELAY_MAX_US);
                w.bytes(&pacs::LC3_CODEC_ID);
                w.u8(ase.cc_len);
                w.bytes(&ase.cc[..usize::from(ase.cc_len)]);
            }
            AseState::QosConfigured => {
                let q = &ase.qos;
                w.u8(q.cig_id);
                w.u8(q.cis_id);
                w.le24(q.sdu_interval_us);
                w.u8(q.framing);
                w.u8(q.phy);
                w.le16(q.max_sdu);
                w.u8(q.retransmission_number);
                w.le16(q.max_transport_latency_ms);
                w.le24(q.presentation_delay_us);
            }
            AseState::Enabling | AseState::Streaming | AseState::Disabling => {
                w.u8(ase.qos.cig_id);
                w.u8(ase.qos.cis_id);
                w.u8(ase.metadata_len);
                w.bytes(&ase.metadata[..usize::from(ase.metadata_len)]);
            }
        }
        w.pos
    }

    /// Handles an ASE Control Point write. Writes the Control Point notification into
    /// `rsp` and returns its length; there is always a response (ASCS v1.0.1 §5):
    /// * unknown opcode: `[op, 0xFF, 0x00, 0x01, 0x00]`;
    /// * invalid length (including Number_of_ASEs of 0 or above `N`, and an empty
    ///   write, answered with opcode 0x00): `[op, 0xFF, 0x00, 0x02, 0x00]`; no ASE is
    ///   touched;
    /// * otherwise one (ASE_ID, Response_Code, Reason) entry per requested ASE, each
    ///   processed in order.
    ///
    /// # Panics
    /// If `rsp` is shorter than the response; [`CP_RESPONSE_MAX_LEN`] always suffices.
    pub fn control_point(&mut self, data: &[u8], rsp: &mut [u8], fx: &mut Effects) -> usize {
        let Some((&op, params)) = data.split_first() else {
            return op_error(rsp, 0x00, INVALID_LENGTH);
        };
        if !(opcode::CONFIG_CODEC..=opcode::RELEASE).contains(&op) {
            return op_error(rsp, op, UNSUPPORTED_OPCODE);
        }
        let Some((&num, mut entries)) = params.split_first() else {
            return op_error(rsp, op, INVALID_LENGTH);
        };
        if !Self::length_valid(op, num, entries) {
            return op_error(rsp, op, INVALID_LENGTH);
        }

        let mut w = Writer { buf: rsp, pos: 0 };
        w.u8(op);
        w.u8(num);
        for _ in 0..num {
            // Lengths were validated above.
            let (entry, rest) = entries.split_at(entry_len(op, entries).unwrap_or(entries.len()));
            entries = rest;
            let ase_id = entry[0];
            let (code, why) = match self.index(ase_id) {
                None => (INVALID_ASE_ID, reason::NONE),
                Some(i) => self.apply(op, i, &entry[1..], fx),
            };
            w.u8(ase_id);
            w.u8(code);
            w.u8(why);
        }
        w.pos
    }

    /// LE CIS Request: returns true (accept) if an ASE in QoS Configured or Enabling
    /// state without a CIS is configured with these CIG/CIS ids.
    pub fn cis_request(&self, cig_id: u8, cis_id: u8) -> bool {
        self.connectable_ase(cig_id, cis_id).is_some()
    }

    /// LE CIS Established with status 0. Couples the CIS to its ASE and emits
    /// [`Action::SetupIsoDataPath`]. If no ASE wants this CIS any more (e.g. it was
    /// released while the CIS was being set up) emits [`Action::DisconnectCis`].
    pub fn cis_established(&mut self, cig_id: u8, cis_id: u8, cis_handle: u16, fx: &mut Effects) {
        let wanted = self.ases.iter().position(|a| {
            matches!(
                a.state,
                AseState::QosConfigured | AseState::Enabling | AseState::Disabling
            ) && a.cis_handle.is_none()
                && a.cis_ids() == (cig_id, cis_id)
        });
        match wanted {
            Some(i) => {
                self.ases[i].cis_handle = Some(cis_handle);
                fx.push(Action::SetupIsoDataPath { cis_handle });
            }
            None => fx.push(Action::DisconnectCis { cis_handle }),
        }
    }

    /// CIS lost or disconnected (any reason). ASCS v1.0.1 §3.2: Streaming and Disabling
    /// go to QoS Configured; other states keep their state, except Releasing, which
    /// completes (Released) to Codec Configured with the codec configuration cached.
    pub fn cis_disconnected(&mut self, cis_handle: u16, fx: &mut Effects) {
        for i in 0..N {
            if self.ases[i].cis_handle != Some(cis_handle) {
                continue;
            }
            let id = ase_id(i);
            let ase = &mut self.ases[i];
            ase.cis_handle = None;
            match ase.state {
                AseState::Streaming | AseState::Disabling => {
                    if ase.state == AseState::Streaming {
                        fx.push(Action::StopStream { ase_id: id });
                    }
                    ase.state = AseState::QosConfigured;
                    ase.metadata_len = 0;
                    fx.mark(id);
                }
                AseState::Releasing => self.released(i, fx),
                _ => {}
            }
        }
    }

    /// ACL disconnected: every ASE returns to Idle without notifications (none can be
    /// sent). Streaming ASEs emit [`Action::StopStream`]; the controller tears down the
    /// CISes itself.
    pub fn acl_disconnected(&mut self, fx: &mut Effects) {
        for (i, ase) in self.ases.iter_mut().enumerate() {
            if ase.state == AseState::Streaming {
                fx.push(Action::StopStream { ase_id: ase_id(i) });
            }
            *ase = Ase::IDLE;
        }
    }

    /// Completes the Release of ASEs that had no CIS (Releasing -> Codec Configured,
    /// codec configuration cached). Call after the notifications of an `Effects` with
    /// [`Effects::has_deferred`] set; a no-op otherwise.
    pub fn complete_deferred(&mut self, fx: &mut Effects) {
        for i in 0..N {
            if self.ases[i].state == AseState::Releasing && self.ases[i].release_pending {
                self.released(i, fx);
            }
        }
    }

    fn index(&self, ase_id: u8) -> Option<usize> {
        let i = usize::from(ase_id).checked_sub(1)?;
        (i < N).then_some(i)
    }

    fn connectable_ase(&self, cig_id: u8, cis_id: u8) -> Option<usize> {
        self.ases.iter().position(|a| {
            matches!(a.state, AseState::QosConfigured | AseState::Enabling)
                && a.cis_handle.is_none()
                && a.cis_ids() == (cig_id, cis_id)
        })
    }

    /// ASCS v1.0.1 §5: invalid length if Number_of_ASEs is 0 or the parameter arrays do
    /// not exactly fill the write. Number_of_ASEs above `N` is also refused here.
    fn length_valid(op: u8, num: u8, mut entries: &[u8]) -> bool {
        if num == 0 || usize::from(num) > N {
            return false;
        }
        for _ in 0..num {
            let Some(len) = entry_len(op, entries) else {
                return false;
            };
            entries = &entries[len..];
        }
        entries.is_empty()
    }

    /// Applies one operation entry (without its ASE_ID) to ASE index `i`.
    fn apply(&mut self, op: u8, i: usize, p: &[u8], fx: &mut Effects) -> Rsp {
        match op {
            opcode::CONFIG_CODEC => self.config_codec(i, p, fx),
            opcode::CONFIG_QOS => self.config_qos(i, p, fx),
            opcode::ENABLE => self.enable(i, &p[1..], fx),
            opcode::RECEIVER_START_READY => self.receiver_start_ready(i, fx),
            opcode::DISABLE => self.disable(i, fx),
            opcode::RECEIVER_STOP_READY => self.receiver_stop_ready(i, fx),
            opcode::UPDATE_METADATA => self.update_metadata(i, &p[1..], fx),
            opcode::RELEASE => self.release(i, fx),
            _ => unreachable!("opcode range checked by control_point"),
        }
    }

    /// Config Codec (ASCS v1.0.1 §5.1). `p`: Target_Latency, Target_PHY, Codec_ID[5],
    /// Codec_Specific_Configuration_Length, Codec_Specific_Configuration.
    fn config_codec(&mut self, i: usize, p: &[u8], fx: &mut Effects) -> Rsp {
        if !matches!(
            self.ases[i].state,
            AseState::Idle | AseState::CodecConfigured | AseState::QosConfigured
        ) {
            return BAD_TRANSITION;
        }
        // Errors are reported for the first parameter in error (ASCS v1.0.1 §5).
        // Target_Latency 0x01..=0x03 and Target_PHY 0x01..=0x03 (Table 5.2); Zephyr
        // reports an invalid Target_Latency with reason Max_Transport_Latency.
        if !(0x01..=0x03).contains(&p[0]) {
            return (
                INVALID_CONFIGURATION_PARAMETER_VALUE,
                reason::MAX_TRANSPORT_LATENCY,
            );
        }
        if !(0x01..=0x03).contains(&p[1]) {
            return (INVALID_CONFIGURATION_PARAMETER_VALUE, reason::PHY);
        }
        let codec_id = &p[2..7];
        if codec_id != pacs::LC3_CODEC_ID {
            // Company/vendor ids shall be 0 unless Coding_Format is 0xFF (Table 5.2).
            let code = if codec_id[0] != 0xFF && codec_id[1..] != [0; 4] {
                INVALID_CONFIGURATION_PARAMETER_VALUE
            } else {
                UNSUPPORTED_CONFIGURATION_PARAMETER_VALUE
            };
            return (code, reason::CODEC_ID);
        }
        let cc = &p[8..];
        let codec = match parse_codec_config(cc) {
            Ok(codec) => codec,
            Err(code) => return (code, reason::CODEC_SPECIFIC_CONFIGURATION),
        };
        if cc.len() > MAX_CODEC_CONFIG_LEN {
            return (INSUFFICIENT_RESOURCES, reason::NONE);
        }

        let ase = &mut self.ases[i];
        ase.state = AseState::CodecConfigured;
        ase.codec = codec;
        ase.cc[..cc.len()].copy_from_slice(cc);
        ase.cc_len = cc.len() as u8;
        // Leaving QoS Configured drops the CIS coupling; the client owns its CIS.
        ase.cis_handle = None;
        fx.mark(ase_id(i));
        OK
    }

    /// Config QoS (ASCS v1.0.1 §5.2). `p`: CIG_ID, CIS_ID, SDU_Interval[3], Framing,
    /// PHY, Max_SDU[2], Retransmission_Number, Max_Transport_Latency[2],
    /// Presentation_Delay[3].
    fn config_qos(&mut self, i: usize, p: &[u8], fx: &mut Effects) -> Rsp {
        if !matches!(
            self.ases[i].state,
            AseState::CodecConfigured | AseState::QosConfigured
        ) {
            return BAD_TRANSITION;
        }
        let qos = QosConfig {
            cig_id: p[0],
            cis_id: p[1],
            sdu_interval_us: le24(&p[2..5]),
            framing: p[5],
            phy: p[6],
            max_sdu: le16(&p[7..9]),
            retransmission_number: p[9],
            max_transport_latency_ms: le16(&p[10..12]),
            presentation_delay_us: le24(&p[12..15]),
        };
        let codec = self.ases[i].codec;

        // Checks follow the parameter order (first parameter in error wins).
        // CIG_ID/CIS_ID range 0x00..=0xEF (Core Vol 4 Part E §7.8.97).
        if qos.cig_id > 0xEF || qos.cis_id > 0xEF {
            return (
                INVALID_CONFIGURATION_PARAMETER_VALUE,
                reason::INVALID_ASE_CIS_MAPPING,
            );
        }
        // Two Source ASEs must not share a CIS (ASCS v1.0.1 §5.2).
        let mapped_elsewhere = self
            .ases
            .iter()
            .enumerate()
            .any(|(j, a)| j != i && a.has_qos() && a.cis_ids() == (qos.cig_id, qos.cis_id));
        if mapped_elsewhere {
            return (
                INVALID_CONFIGURATION_PARAMETER_VALUE,
                reason::INVALID_ASE_CIS_MAPPING,
            );
        }
        // SDU_Interval range 0x0000FF..=0x0FFFFF (Core Vol 4 Part E §7.8.97). The
        // encoder emits one SDU per Codec_Frame_Blocks_Per_SDU frames, so any other
        // interval is unsupported.
        if !(0x0000FF..=0x0FFFFF).contains(&qos.sdu_interval_us) {
            return (INVALID_CONFIGURATION_PARAMETER_VALUE, reason::SDU_INTERVAL);
        }
        if qos.sdu_interval_us != codec.frame_duration_us * u32::from(codec.frames_per_sdu) {
            return (
                UNSUPPORTED_CONFIGURATION_PARAMETER_VALUE,
                reason::SDU_INTERVAL,
            );
        }
        if qos.framing > 0x01 {
            return (INVALID_CONFIGURATION_PARAMETER_VALUE, reason::FRAMING);
        }
        if qos.phy == 0 || qos.phy & !0x07 != 0 {
            return (INVALID_CONFIGURATION_PARAMETER_VALUE, reason::PHY);
        }
        // Max_SDU range 0x000..=0xFFF (ASCS v1.0.1 Table 5.3); it must also hold one SDU
        // of the configured codec (single channel).
        let sdu_len = u32::from(codec.octets_per_frame) * u32::from(codec.frames_per_sdu);
        if qos.max_sdu > 0x0FFF || u32::from(qos.max_sdu) < sdu_len {
            return (
                INVALID_CONFIGURATION_PARAMETER_VALUE,
                reason::MAXIMUM_SDU_SIZE,
            );
        }
        // Retransmission_Number: all values 0x00..=0xFF are valid.
        if !(0x0005..=0x0FA0).contains(&qos.max_transport_latency_ms) {
            return (
                INVALID_CONFIGURATION_PARAMETER_VALUE,
                reason::MAX_TRANSPORT_LATENCY,
            );
        }
        // Presentation_Delay must lie within the range exposed in Codec Configured.
        if !(qos_pref::PRESENTATION_DELAY_MIN_US..=qos_pref::PRESENTATION_DELAY_MAX_US)
            .contains(&qos.presentation_delay_us)
        {
            return (
                UNSUPPORTED_CONFIGURATION_PARAMETER_VALUE,
                reason::PRESENTATION_DELAY,
            );
        }

        let ase = &mut self.ases[i];
        if ase.state == AseState::QosConfigured && ase.cis_ids() != (qos.cig_id, qos.cis_id) {
            // Remapped to another CIS: the established one no longer belongs to it.
            ase.cis_handle = None;
        }
        ase.state = AseState::QosConfigured;
        ase.qos = qos;
        fx.mark(ase_id(i));
        OK
    }

    /// Enable (ASCS v1.0.1 §5.3). `md`: Metadata.
    fn enable(&mut self, i: usize, md: &[u8], fx: &mut Effects) -> Rsp {
        if self.ases[i].state != AseState::QosConfigured {
            return BAD_TRANSITION;
        }
        if let Err(rsp) = self.store_metadata(i, md) {
            return rsp;
        }
        self.ases[i].state = AseState::Enabling;
        fx.mark(ase_id(i));
        OK
    }

    /// Receiver Start Ready (ASCS v1.0.1 §5.4), written by the client (Audio Sink) for a
    /// Source ASE. Streaming requires an established CIS with its data path; before that
    /// the request is refused with Invalid ASE State Machine Transition (as Zephyr does).
    fn receiver_start_ready(&mut self, i: usize, fx: &mut Effects) -> Rsp {
        let ase = &mut self.ases[i];
        if ase.state != AseState::Enabling {
            return BAD_TRANSITION;
        }
        let Some(cis_handle) = ase.cis_handle else {
            return BAD_TRANSITION;
        };
        ase.state = AseState::Streaming;
        let id = ase_id(i);
        fx.push(Action::StartStream {
            ase_id: id,
            cis_handle,
            codec: ase.codec,
            qos: ase.qos,
        });
        fx.mark(id);
        OK
    }

    /// Disable (ASCS v1.0.1 §5.5): a Source ASE goes to Disabling and waits for
    /// Receiver Stop Ready.
    fn disable(&mut self, i: usize, fx: &mut Effects) -> Rsp {
        let id = ase_id(i);
        let ase = &mut self.ases[i];
        match ase.state {
            AseState::Streaming => fx.push(Action::StopStream { ase_id: id }),
            AseState::Enabling => {}
            _ => return BAD_TRANSITION,
        }
        ase.state = AseState::Disabling;
        fx.mark(id);
        OK
    }

    /// Receiver Stop Ready (ASCS v1.0.1 §5.6): Disabling -> QoS Configured.
    fn receiver_stop_ready(&mut self, i: usize, fx: &mut Effects) -> Rsp {
        let ase = &mut self.ases[i];
        if ase.state != AseState::Disabling {
            return BAD_TRANSITION;
        }
        ase.state = AseState::QosConfigured;
        ase.metadata_len = 0;
        fx.mark(ase_id(i));
        OK
    }

    /// Update Metadata (ASCS v1.0.1 §5.7). `md`: Metadata.
    fn update_metadata(&mut self, i: usize, md: &[u8], fx: &mut Effects) -> Rsp {
        if !matches!(self.ases[i].state, AseState::Enabling | AseState::Streaming) {
            return BAD_TRANSITION;
        }
        if let Err(rsp) = self.store_metadata(i, md) {
            return rsp;
        }
        fx.mark(ase_id(i));
        OK
    }

    /// Release (ASCS v1.0.1 §5.8). With a CIS, the Released operation (§5.9) follows
    /// its disconnection; without one it is deferred to `complete_deferred` so the
    /// Releasing state is notified first.
    fn release(&mut self, i: usize, fx: &mut Effects) -> Rsp {
        let id = ase_id(i);
        let ase = &mut self.ases[i];
        match ase.state {
            AseState::Idle | AseState::Releasing => return BAD_TRANSITION,
            AseState::Streaming => fx.push(Action::StopStream { ase_id: id }),
            _ => {}
        }
        ase.state = AseState::Releasing;
        ase.qos = QosConfig::EMPTY;
        ase.metadata_len = 0;
        match ase.cis_handle {
            Some(cis_handle) => fx.push(Action::DisconnectCis { cis_handle }),
            None => {
                ase.release_pending = true;
                fx.deferred = true;
            }
        }
        fx.mark(id);
        OK
    }

    /// Released (ASCS v1.0.1 §5.9): Releasing -> Codec Configured, caching the codec
    /// configuration last applied by the client.
    fn released(&mut self, i: usize, fx: &mut Effects) {
        let ase = &mut self.ases[i];
        ase.state = AseState::CodecConfigured;
        ase.cis_handle = None;
        ase.release_pending = false;
        fx.mark(ase_id(i));
    }

    fn store_metadata(&mut self, i: usize, md: &[u8]) -> Result<(), Rsp> {
        validate_metadata(md)?;
        if md.len() > MAX_METADATA_LEN {
            return Err((INSUFFICIENT_RESOURCES, reason::NONE));
        }
        let ase = &mut self.ases[i];
        ase.metadata[..md.len()].copy_from_slice(md);
        ase.metadata_len = md.len() as u8;
        Ok(())
    }
}

fn ase_id(index: usize) -> u8 {
    index as u8 + 1
}

/// Length of the next parameter entry (including ASE_ID) of operation `op`, or None if
/// `rest` is too short (ASCS v1.0.1 Tables 5.2-5.9).
fn entry_len(op: u8, rest: &[u8]) -> Option<usize> {
    let len = match op {
        // ASE_ID, Target_Latency, Target_PHY, Codec_ID[5], CC_Length, CC.
        opcode::CONFIG_CODEC => 9 + usize::from(*rest.get(8)?),
        // ASE_ID + 15 octets of QoS parameters.
        opcode::CONFIG_QOS => 16,
        // ASE_ID, Metadata_Length, Metadata.
        opcode::ENABLE | opcode::UPDATE_METADATA => 2 + usize::from(*rest.get(1)?),
        // ASE_ID only.
        _ => 1,
    };
    (rest.len() >= len).then_some(len)
}

fn op_error(rsp: &mut [u8], op: u8, code: u8) -> usize {
    // ASCS v1.0.1 Table 4.7: Number_of_ASEs 0xFF, ASE_ID 0, Reason 0.
    rsp[..5].copy_from_slice(&[op, 0xFF, 0x00, code, reason::NONE]);
    5
}

/// Decodes and checks a Codec_Specific_Configuration against the Source PAC record.
/// Returns the Response_Code on error (Reason is always Codec_Specific_Configuration).
fn parse_codec_config(cc: &[u8]) -> Result<CodecConfig, u8> {
    const INVALID: u8 = INVALID_CONFIGURATION_PARAMETER_VALUE;
    const UNSUPPORTED: u8 = UNSUPPORTED_CONFIGURATION_PARAMETER_VALUE;

    let mut sampling = None;
    let mut duration = None;
    let mut allocation = 0u32;
    let mut octets = None;
    let mut blocks = 1u8;
    for item in ltv::iter(cc) {
        let (ty, v) = item.map_err(|_| INVALID)?;
        match ty {
            cc_type::SAMPLING_FREQUENCY => sampling = Some(*one(v).ok_or(INVALID)?),
            cc_type::FRAME_DURATION => duration = Some(*one(v).ok_or(INVALID)?),
            cc_type::AUDIO_CHANNEL_ALLOCATION => {
                let v: [u8; 4] = v.try_into().map_err(|_| INVALID)?;
                allocation = u32::from_le_bytes(v);
            }
            cc_type::OCTETS_PER_CODEC_FRAME => {
                let v: [u8; 2] = v.try_into().map_err(|_| INVALID)?;
                octets = Some(u16::from_le_bytes(v));
            }
            cc_type::CODEC_FRAME_BLOCKS_PER_SDU => blocks = *one(v).ok_or(INVALID)?,
            // Unknown types are ignored for forward compatibility.
            _ => {}
        }
    }
    // Sampling_Frequency, Frame_Duration and Octets_Per_Codec_Frame are mandatory
    // (BAP v1.0 §4.3.2); the other two have defaults (mono / one block).
    let (Some(sampling), Some(duration), Some(octets)) = (sampling, duration, octets) else {
        return Err(INVALID);
    };

    // Sampling_Frequency codes (Assigned Numbers): 0x01 8 kHz, 0x02 11.025, 0x03 16,
    // 0x04 22.05, 0x05 24, 0x06 32, 0x07 44.1, 0x08 48, 0x09 88.2, 0x0A 96,
    // 0x0B 176.4, 0x0C 192, 0x0D 384 kHz.
    let sampling_hz = match sampling {
        0x03 => 16_000,
        0x05 => 24_000,
        0x06 => 32_000,
        0x01..=0x0D => return Err(UNSUPPORTED),
        _ => return Err(INVALID),
    };
    // Frame_Duration: 0x00 7.5 ms, 0x01 10 ms.
    let frame_duration_us = match duration {
        0x01 => pacs::FRAME_DURATION_US,
        0x00 => return Err(UNSUPPORTED),
        _ => return Err(INVALID),
    };
    // Only exposed Source Audio Locations; with a single location bit this also caps
    // the channel count at 1.
    if allocation & !pacs::SOURCE_AUDIO_LOCATIONS != 0 {
        return Err(UNSUPPORTED);
    }
    if !(pacs::MIN_OCTETS_PER_FRAME..=pacs::MAX_OCTETS_PER_FRAME).contains(&octets) {
        return Err(UNSUPPORTED);
    }
    if blocks == 0 {
        return Err(INVALID);
    }
    if blocks > pacs::MAX_FRAMES_PER_SDU {
        return Err(UNSUPPORTED);
    }
    Ok(CodecConfig {
        sampling_hz,
        frame_duration_us,
        octets_per_frame: octets,
        frames_per_sdu: blocks,
        audio_locations: allocation,
    })
}

/// Validates Enable/Update Metadata LTVs (ASCS v1.0.1 Table 5.1: Reason = Metadata
/// Type in error). Context metadata must be 2 octets and nonzero (Invalid Metadata);
/// Streaming_Audio_Contexts must be a subset of the available Source contexts
/// (Rejected Metadata), as Zephyr enforces for a CAP Acceptor. Other types are
/// accepted unchecked.
fn validate_metadata(md: &[u8]) -> Result<(), Rsp> {
    for item in ltv::iter(md) {
        let (ty, v) = item.map_err(|ty| (INVALID_METADATA, ty))?;
        if ty == metadata_type::PREFERRED_AUDIO_CONTEXTS
            || ty == metadata_type::STREAMING_AUDIO_CONTEXTS
        {
            let v: [u8; 2] = v.try_into().map_err(|_| (INVALID_METADATA, ty))?;
            let contexts = u16::from_le_bytes(v);
            if contexts == 0 {
                return Err((INVALID_METADATA, ty));
            }
            if ty == metadata_type::STREAMING_AUDIO_CONTEXTS
                && contexts & !pacs::SOURCE_CONTEXTS != 0
            {
                return Err((REJECTED_METADATA, ty));
            }
        }
    }
    Ok(())
}

fn one(v: &[u8]) -> Option<&u8> {
    match v {
        [b] => Some(b),
        _ => None,
    }
}

fn le16(b: &[u8]) -> u16 {
    u16::from_le_bytes([b[0], b[1]])
}

fn le24(b: &[u8]) -> u32 {
    u32::from_le_bytes([b[0], b[1], b[2], 0])
}

/// Sequential writer that panics (slice index) when the buffer is too small.
struct Writer<'a> {
    buf: &'a mut [u8],
    pos: usize,
}

impl Writer<'_> {
    fn u8(&mut self, v: u8) {
        self.buf[self.pos] = v;
        self.pos += 1;
    }

    fn bytes(&mut self, v: &[u8]) {
        self.buf[self.pos..self.pos + v.len()].copy_from_slice(v);
        self.pos += v.len();
    }

    fn le16(&mut self, v: u16) {
        self.bytes(&v.to_le_bytes());
    }

    fn le24(&mut self, v: u32) {
        self.bytes(&v.to_le_bytes()[..3]);
    }
}
