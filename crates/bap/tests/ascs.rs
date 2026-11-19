use bap::ascs::{
    ASE_VALUE_MAX_LEN, Action, Ascs, AseState, CP_RESPONSE_MAX_LEN, CodecConfig, Effects, QosConfig,
};

/// 16_2: 16 kHz, 10 ms, Front Left, 40 octets.
const CC_16_2: [u8; 16] = [
    2, 0x01, 0x03, // Sampling_Frequency 16 kHz
    2, 0x02, 0x01, // Frame_Duration 10 ms
    5, 0x03, 0x01, 0x00, 0x00, 0x00, // Audio_Channel_Allocation Front Left
    3, 0x04, 40, 0, // Octets_Per_Codec_Frame 40
];

const CIS_HANDLE: u16 = 0x0060;

fn cp<const N: usize>(ascs: &mut Ascs<N>, data: &[u8], fx: &mut Effects) -> Vec<u8> {
    let mut rsp = [0u8; CP_RESPONSE_MAX_LEN];
    let n = ascs.control_point(data, &mut rsp, fx);
    rsp[..n].to_vec()
}

fn value<const N: usize>(ascs: &Ascs<N>, ase_id: u8) -> Vec<u8> {
    let mut out = [0u8; ASE_VALUE_MAX_LEN];
    let n = ascs.ase_value(ase_id, &mut out);
    out[..n].to_vec()
}

fn changed(fx: &Effects) -> Vec<u8> {
    fx.changed_ases().collect()
}

/// Config Codec parameter array for one ASE: low latency, 2M PHY, LC3.
fn codec_entry(ase_id: u8, cc: &[u8]) -> Vec<u8> {
    let mut v = vec![ase_id, 0x01, 0x02, 0x06, 0, 0, 0, 0, cc.len() as u8];
    v.extend_from_slice(cc);
    v
}

fn config_codec(entries: &[Vec<u8>]) -> Vec<u8> {
    let mut v = vec![0x01, entries.len() as u8];
    for e in entries {
        v.extend_from_slice(e);
    }
    v
}

/// Config QoS parameter array: SDU interval 10 ms, unframed, 2M, Max_SDU 40, RTN 2,
/// 10 ms latency, 40 ms presentation delay.
fn qos_entry(ase_id: u8, cig: u8, cis: u8) -> Vec<u8> {
    vec![
        ase_id, cig, cis, 0x10, 0x27, 0x00, 0x00, 0x02, 40, 0, 2, 10, 0, 0x40, 0x9C, 0x00,
    ]
}

fn config_qos(entries: &[Vec<u8>]) -> Vec<u8> {
    let mut v = vec![0x02, entries.len() as u8];
    for e in entries {
        v.extend_from_slice(e);
    }
    v
}

fn codec_configured_value(ase_id: u8, cc: &[u8]) -> Vec<u8> {
    let mut v = vec![
        ase_id,
        0x01, // Codec Configured
        0x00, // Framing: unframed supported
        0x03, // Preferred_PHY: 1M | 2M
        0x02, // Preferred RTN
        0x0A,
        0x00, // Max_Transport_Latency 10 ms
        0x10,
        0x27,
        0x00, // Presentation_Delay_Min 10000 us
        0x40,
        0x9C,
        0x00, // Presentation_Delay_Max 40000 us
        0x00,
        0x00,
        0x00, // Preferred_Presentation_Delay_Min: none
        0x00,
        0x00,
        0x00, // Preferred_Presentation_Delay_Max: none
        0x06,
        0x00,
        0x00,
        0x00,
        0x00, // LC3
        cc.len() as u8,
    ];
    v.extend_from_slice(cc);
    v
}

fn qos_configured_value(ase_id: u8, cig: u8, cis: u8) -> Vec<u8> {
    vec![
        ase_id, 0x02, cig, cis, 0x10, 0x27, 0x00, 0x00, 0x02, 40, 0, 2, 10, 0, 0x40, 0x9C, 0x00,
    ]
}

const CODEC_16_2: CodecConfig = CodecConfig {
    sampling_hz: 16_000,
    frame_duration_us: 10_000,
    octets_per_frame: 40,
    frames_per_sdu: 1,
    audio_locations: 1,
};

const QOS: QosConfig = QosConfig {
    cig_id: 0,
    cis_id: 0,
    sdu_interval_us: 10_000,
    framing: 0,
    phy: 2,
    max_sdu: 40,
    retransmission_number: 2,
    max_transport_latency_ms: 10,
    presentation_delay_us: 40_000,
};

/// Streaming_Audio_Contexts = Conversational.
const MD_CONVERSATIONAL: [u8; 4] = [3, 0x02, 0x02, 0x00];

/// Drives ASE 1 of `ascs` to Enabling with CIG 0 / CIS 0.
fn to_enabling<const N: usize>(ascs: &mut Ascs<N>) {
    let mut fx = Effects::default();
    assert_eq!(
        cp(ascs, &config_codec(&[codec_entry(1, &CC_16_2)]), &mut fx),
        [1, 1, 1, 0, 0]
    );
    assert_eq!(
        cp(ascs, &config_qos(&[qos_entry(1, 0, 0)]), &mut fx),
        [2, 1, 1, 0, 0]
    );
    let mut enable = vec![0x03, 1, 1, 4];
    enable.extend_from_slice(&MD_CONVERSATIONAL);
    assert_eq!(cp(ascs, &enable, &mut fx), [3, 1, 1, 0, 0]);
    assert_eq!(ascs.state(1), Some(AseState::Enabling));
}

/// Drives ASE 1 to Streaming on CIS_HANDLE.
fn to_streaming<const N: usize>(ascs: &mut Ascs<N>) {
    to_enabling(ascs);
    let mut fx = Effects::default();
    assert!(ascs.cis_request(0, 0));
    ascs.cis_established(0, 0, CIS_HANDLE, &mut fx);
    assert_eq!(cp(ascs, &[0x04, 1, 1], &mut fx), [4, 1, 1, 0, 0]);
    assert_eq!(ascs.state(1), Some(AseState::Streaming));
}

#[test]
fn initial_state() {
    let ascs = Ascs::<1>::new();
    assert_eq!(ascs.state(1), Some(AseState::Idle));
    assert_eq!(ascs.state(0), None);
    assert_eq!(ascs.state(2), None);
    assert_eq!(value(&ascs, 1), [0x01, 0x00]);
    assert_eq!(value(&ascs, 2), Vec::<u8>::new());
    assert_eq!(Ascs::<2>::ase_ids().collect::<Vec<_>>(), [1, 2]);
}

#[test]
fn happy_path() {
    let mut ascs = Ascs::<1>::new();
    let mut fx = Effects::default();

    // Config Codec 16_2.
    let rsp = cp(
        &mut ascs,
        &config_codec(&[codec_entry(1, &CC_16_2)]),
        &mut fx,
    );
    assert_eq!(rsp, [0x01, 0x01, 0x01, 0x00, 0x00]);
    assert_eq!(changed(&fx), [1]);
    assert!(fx.actions().is_empty());
    let codec_configured = codec_configured_value(1, &CC_16_2);
    assert_eq!(codec_configured.len(), 41);
    assert_eq!(value(&ascs, 1), codec_configured);
    fx.clear();

    // Config QoS.
    let rsp = cp(&mut ascs, &config_qos(&[qos_entry(1, 0, 0)]), &mut fx);
    assert_eq!(rsp, [0x02, 0x01, 0x01, 0x00, 0x00]);
    assert_eq!(changed(&fx), [1]);
    assert_eq!(value(&ascs, 1), qos_configured_value(1, 0, 0));
    fx.clear();

    // CIS request before Enable is allowed in QoS Configured; not for unknown ids.
    assert!(ascs.cis_request(0, 0));
    assert!(!ascs.cis_request(0, 1));

    // Enable with Streaming_Audio_Contexts = Conversational.
    let rsp = cp(
        &mut ascs,
        &[0x03, 0x01, 0x01, 0x04, 0x03, 0x02, 0x02, 0x00],
        &mut fx,
    );
    assert_eq!(rsp, [0x03, 0x01, 0x01, 0x00, 0x00]);
    assert_eq!(changed(&fx), [1]);
    assert_eq!(
        value(&ascs, 1),
        [0x01, 0x03, 0x00, 0x00, 0x04, 0x03, 0x02, 0x02, 0x00]
    );
    fx.clear();

    // CIS request + established -> set up the ISO data path; no ASE change.
    assert!(ascs.cis_request(0, 0));
    ascs.cis_established(0, 0, CIS_HANDLE, &mut fx);
    assert_eq!(
        fx.actions(),
        [Action::SetupIsoDataPath {
            cis_handle: CIS_HANDLE
        }]
    );
    assert_eq!(changed(&fx), Vec::<u8>::new());
    assert_eq!(ascs.state(1), Some(AseState::Enabling));
    // The CIS is taken now.
    assert!(!ascs.cis_request(0, 0));
    fx.clear();

    // Receiver Start Ready -> Streaming + StartStream.
    let rsp = cp(&mut ascs, &[0x04, 0x01, 0x01], &mut fx);
    assert_eq!(rsp, [0x04, 0x01, 0x01, 0x00, 0x00]);
    assert_eq!(changed(&fx), [1]);
    assert_eq!(
        fx.actions(),
        [Action::StartStream {
            ase_id: 1,
            cis_handle: CIS_HANDLE,
            codec: CODEC_16_2,
            qos: QOS
        }]
    );
    assert_eq!(
        value(&ascs, 1),
        [0x01, 0x04, 0x00, 0x00, 0x04, 0x03, 0x02, 0x02, 0x00]
    );
    fx.clear();

    // Disable -> Disabling + StopStream.
    let rsp = cp(&mut ascs, &[0x05, 0x01, 0x01], &mut fx);
    assert_eq!(rsp, [0x05, 0x01, 0x01, 0x00, 0x00]);
    assert_eq!(changed(&fx), [1]);
    assert_eq!(fx.actions(), [Action::StopStream { ase_id: 1 }]);
    assert_eq!(
        value(&ascs, 1),
        [0x01, 0x05, 0x00, 0x00, 0x04, 0x03, 0x02, 0x02, 0x00]
    );
    fx.clear();

    // Receiver Stop Ready -> QoS Configured.
    let rsp = cp(&mut ascs, &[0x06, 0x01, 0x01], &mut fx);
    assert_eq!(rsp, [0x06, 0x01, 0x01, 0x00, 0x00]);
    assert_eq!(changed(&fx), [1]);
    assert!(fx.actions().is_empty());
    assert_eq!(value(&ascs, 1), qos_configured_value(1, 0, 0));
    fx.clear();

    // Release -> Releasing + DisconnectCis; nothing deferred because a CIS exists.
    let rsp = cp(&mut ascs, &[0x08, 0x01, 0x01], &mut fx);
    assert_eq!(rsp, [0x08, 0x01, 0x01, 0x00, 0x00]);
    assert_eq!(changed(&fx), [1]);
    assert_eq!(
        fx.actions(),
        [Action::DisconnectCis {
            cis_handle: CIS_HANDLE
        }]
    );
    assert!(!fx.has_deferred());
    assert_eq!(value(&ascs, 1), [0x01, 0x06]);
    fx.clear();

    // CIS gone -> Released -> Codec Configured with the cached codec configuration.
    ascs.cis_disconnected(CIS_HANDLE, &mut fx);
    assert_eq!(changed(&fx), [1]);
    assert!(fx.actions().is_empty());
    assert_eq!(ascs.state(1), Some(AseState::CodecConfigured));
    assert_eq!(value(&ascs, 1), codec_configured);
}

#[test]
fn restart_after_stop_reuses_cis() {
    let mut ascs = Ascs::<1>::new();
    to_streaming(&mut ascs);
    let mut fx = Effects::default();
    cp(&mut ascs, &[0x05, 1, 1], &mut fx);
    cp(&mut ascs, &[0x06, 1, 1], &mut fx);
    fx.clear();
    assert_eq!(cp(&mut ascs, &[0x03, 1, 1, 0], &mut fx), [3, 1, 1, 0, 0]);
    assert_eq!(value(&ascs, 1), [1, 3, 0, 0, 0]);
    fx.clear();
    assert_eq!(cp(&mut ascs, &[0x04, 1, 1], &mut fx), [4, 1, 1, 0, 0]);
    assert!(matches!(
        fx.actions(),
        [Action::StartStream {
            cis_handle: CIS_HANDLE,
            ..
        }]
    ));
}

#[test]
fn unsupported_opcode() {
    let mut ascs = Ascs::<1>::new();
    let mut fx = Effects::default();
    assert_eq!(
        cp(&mut ascs, &[0x09, 0x01, 0x01], &mut fx),
        [0x09, 0xFF, 0x00, 0x01, 0x00]
    );
    assert_eq!(
        cp(&mut ascs, &[0x00], &mut fx),
        [0x00, 0xFF, 0x00, 0x01, 0x00]
    );
    assert_eq!(
        cp(&mut ascs, &[0xFF, 1, 2, 3], &mut fx),
        [0xFF, 0xFF, 0x00, 0x01, 0x00]
    );
    assert!(changed(&fx).is_empty());
}

#[test]
fn invalid_length() {
    let mut ascs = Ascs::<1>::new();
    let mut fx = Effects::default();
    let invalid = |op: u8| vec![op, 0xFF, 0x00, 0x02, 0x00];

    // Empty write.
    assert_eq!(cp(&mut ascs, &[], &mut fx), invalid(0x00));
    // Missing Number_of_ASEs.
    assert_eq!(cp(&mut ascs, &[0x08], &mut fx), invalid(0x08));
    // Number_of_ASEs = 0.
    assert_eq!(cp(&mut ascs, &[0x08, 0x00], &mut fx), invalid(0x08));
    // Number_of_ASEs does not match the arrays.
    assert_eq!(cp(&mut ascs, &[0x08, 0x01], &mut fx), invalid(0x08));
    // Trailing octet.
    assert_eq!(
        cp(&mut ascs, &[0x08, 0x01, 0x01, 0x00], &mut fx),
        invalid(0x08)
    );
    // More ASEs than exist.
    assert_eq!(
        cp(&mut ascs, &[0x08, 0x02, 0x01, 0x01], &mut fx),
        invalid(0x08)
    );

    // Truncated Config Codec: Codec_Specific_Configuration shorter than its length.
    let mut data = config_codec(&[codec_entry(1, &CC_16_2)]);
    data.pop();
    assert_eq!(cp(&mut ascs, &data, &mut fx), invalid(0x01));
    // Truncated before the length octet.
    assert_eq!(
        cp(&mut ascs, &[0x01, 0x01, 0x01, 0x01, 0x02, 0x06], &mut fx),
        invalid(0x01)
    );
    // Truncated Config QoS.
    let mut data = config_qos(&[qos_entry(1, 0, 0)]);
    data.pop();
    assert_eq!(cp(&mut ascs, &data, &mut fx), invalid(0x02));
    // Enable with Metadata_Length beyond the data.
    assert_eq!(
        cp(&mut ascs, &[0x03, 0x01, 0x01, 0x04, 0x03, 0x02], &mut fx),
        invalid(0x03)
    );

    assert!(changed(&fx).is_empty());
    assert_eq!(ascs.state(1), Some(AseState::Idle));
}

#[test]
fn invalid_ase_id() {
    let mut ascs = Ascs::<1>::new();
    let mut fx = Effects::default();
    let rsp = cp(
        &mut ascs,
        &config_codec(&[codec_entry(2, &CC_16_2)]),
        &mut fx,
    );
    assert_eq!(rsp, [0x01, 0x01, 0x02, 0x03, 0x00]);
    let rsp = cp(
        &mut ascs,
        &config_codec(&[codec_entry(0, &CC_16_2)]),
        &mut fx,
    );
    assert_eq!(rsp, [0x01, 0x01, 0x00, 0x03, 0x00]);
    assert_eq!(
        cp(&mut ascs, &[0x04, 0x01, 0x07], &mut fx),
        [0x04, 0x01, 0x07, 0x03, 0x00]
    );
    assert!(changed(&fx).is_empty());
}

#[test]
fn invalid_transitions() {
    let mut ascs = Ascs::<1>::new();
    let mut fx = Effects::default();
    // Release, Config QoS, Enable, Disable, Stop Ready, Update Metadata from Idle.
    assert_eq!(cp(&mut ascs, &[0x08, 1, 1], &mut fx), [8, 1, 1, 4, 0]);
    assert_eq!(
        cp(&mut ascs, &config_qos(&[qos_entry(1, 0, 0)]), &mut fx),
        [2, 1, 1, 4, 0]
    );
    assert_eq!(cp(&mut ascs, &[0x03, 1, 1, 0], &mut fx), [3, 1, 1, 4, 0]);
    assert_eq!(cp(&mut ascs, &[0x05, 1, 1], &mut fx), [5, 1, 1, 4, 0]);
    assert_eq!(cp(&mut ascs, &[0x06, 1, 1], &mut fx), [6, 1, 1, 4, 0]);
    assert_eq!(cp(&mut ascs, &[0x07, 1, 1, 0], &mut fx), [7, 1, 1, 4, 0]);
    assert!(changed(&fx).is_empty());

    // Config Codec is not allowed once Enabling.
    to_enabling(&mut ascs);
    let rsp = cp(
        &mut ascs,
        &config_codec(&[codec_entry(1, &CC_16_2)]),
        &mut fx,
    );
    assert_eq!(rsp, [1, 1, 1, 4, 0]);
}

#[test]
fn receiver_start_ready_before_cis() {
    let mut ascs = Ascs::<1>::new();
    to_enabling(&mut ascs);
    let mut fx = Effects::default();
    let rsp = cp(&mut ascs, &[0x04, 0x01, 0x01], &mut fx);
    assert_eq!(rsp, [0x04, 0x01, 0x01, 0x04, 0x00]);
    assert_eq!(ascs.state(1), Some(AseState::Enabling));
    assert!(changed(&fx).is_empty());
    assert!(fx.actions().is_empty());

    // An accepted but not yet established CIS is not enough either.
    assert!(ascs.cis_request(0, 0));
    assert_eq!(
        cp(&mut ascs, &[0x04, 0x01, 0x01], &mut fx),
        [0x04, 0x01, 0x01, 0x04, 0x00]
    );
}

#[test]
fn unsupported_codec_configurations() {
    let mut ascs = Ascs::<1>::new();
    let mut fx = Effects::default();
    let check = |ascs: &mut Ascs<1>, fx: &mut Effects, cc: &[u8], code: u8| {
        let rsp = cp(ascs, &config_codec(&[codec_entry(1, cc)]), fx);
        assert_eq!(rsp, [0x01, 0x01, 0x01, code, 0x02], "cc {cc:02X?}");
        assert_eq!(ascs.state(1), Some(AseState::Idle));
    };

    let with = |ty: u8, val: &[u8]| -> Vec<u8> {
        // Replace one LTV of CC_16_2.
        let mut out = Vec::new();
        let mut rest = &CC_16_2[..];
        while let [len, ..] = rest {
            let (entry, tail) = rest.split_at(1 + *len as usize);
            if entry[1] == ty {
                out.push(val.len() as u8 + 1);
                out.push(ty);
                out.extend_from_slice(val);
            } else {
                out.extend_from_slice(entry);
            }
            rest = tail;
        }
        out
    };

    // 48 kHz (0x08) and 8 kHz (0x01): valid codes, not supported.
    check(&mut ascs, &mut fx, &with(0x01, &[0x08]), 0x07);
    check(&mut ascs, &mut fx, &with(0x01, &[0x01]), 0x07);
    // RFU sampling code.
    check(&mut ascs, &mut fx, &with(0x01, &[0x20]), 0x09);
    // 7.5 ms frames.
    check(&mut ascs, &mut fx, &with(0x02, &[0x00]), 0x07);
    // Front Right / stereo allocation.
    check(&mut ascs, &mut fx, &with(0x03, &[0x02, 0, 0, 0]), 0x07);
    check(&mut ascs, &mut fx, &with(0x03, &[0x03, 0, 0, 0]), 0x07);
    // Octets per frame out of 26..=80.
    check(&mut ascs, &mut fx, &with(0x04, &[25, 0]), 0x07);
    check(&mut ascs, &mut fx, &with(0x04, &[81, 0]), 0x07);
    // Wrong LTV value length.
    check(&mut ascs, &mut fx, &with(0x04, &[40]), 0x09);
    // Two blocks per SDU.
    let mut cc = CC_16_2.to_vec();
    cc.extend_from_slice(&[2, 0x05, 2]);
    check(&mut ascs, &mut fx, &cc, 0x07);
    // Missing Octets_Per_Codec_Frame.
    check(&mut ascs, &mut fx, &CC_16_2[..12], 0x09);
    // LTV overrunning the configuration.
    let mut cc = CC_16_2.to_vec();
    cc.extend_from_slice(&[4, 0x05]);
    check(&mut ascs, &mut fx, &cc, 0x09);

    assert!(changed(&fx).is_empty());
}

#[test]
fn supported_codec_configurations() {
    for (sf, hz, octets) in [
        (0x03u8, 16_000u32, 40u8),
        (0x05, 24_000, 60),
        (0x06, 32_000, 80),
    ] {
        let mut ascs = Ascs::<1>::new();
        let mut fx = Effects::default();
        // No allocation, explicit one block, plus an unknown LTV type that is ignored.
        let cc = [
            2, 0x01, sf, 2, 0x02, 0x01, 3, 0x04, octets, 0, 2, 0x05, 1, 2, 0x77, 0xAA,
        ];
        let rsp = cp(&mut ascs, &config_codec(&[codec_entry(1, &cc)]), &mut fx);
        assert_eq!(rsp, [1, 1, 1, 0, 0]);
        assert_eq!(value(&ascs, 1), codec_configured_value(1, &cc));

        // Drive to streaming to observe the decoded configuration.
        let mut q = qos_entry(1, 3, 4);
        q[8] = octets;
        cp(&mut ascs, &config_qos(&[q]), &mut fx);
        cp(&mut ascs, &[0x03, 1, 1, 0], &mut fx);
        ascs.cis_established(3, 4, 0x42, &mut fx);
        fx.clear();
        cp(&mut ascs, &[0x04, 1, 1], &mut fx);
        let [Action::StartStream { codec, qos, .. }] = fx.actions() else {
            panic!()
        };
        assert_eq!(
            *codec,
            CodecConfig {
                sampling_hz: hz,
                frame_duration_us: 10_000,
                octets_per_frame: octets.into(),
                frames_per_sdu: 1,
                audio_locations: 0
            }
        );
        assert_eq!((qos.cig_id, qos.cis_id, qos.max_sdu), (3, 4, octets.into()));
    }
}

#[test]
fn codec_id_and_target_checks() {
    let mut ascs = Ascs::<1>::new();
    let mut fx = Effects::default();
    let mut entry = codec_entry(1, &CC_16_2);
    entry[3] = 0x02; // CVSD
    assert_eq!(
        cp(&mut ascs, &config_codec(&[entry]), &mut fx),
        [1, 1, 1, 0x07, 0x01]
    );
    let mut entry = codec_entry(1, &CC_16_2);
    entry[4] = 0x01; // company id with a non-vendor coding format
    assert_eq!(
        cp(&mut ascs, &config_codec(&[entry]), &mut fx),
        [1, 1, 1, 0x09, 0x01]
    );
    let mut entry = codec_entry(1, &CC_16_2);
    entry[1] = 0x04; // Target_Latency RFU
    assert_eq!(
        cp(&mut ascs, &config_codec(&[entry]), &mut fx),
        [1, 1, 1, 0x09, 0x08]
    );
    let mut entry = codec_entry(1, &CC_16_2);
    entry[2] = 0x00; // Target_PHY RFU
    assert_eq!(
        cp(&mut ascs, &config_codec(&[entry]), &mut fx),
        [1, 1, 1, 0x09, 0x05]
    );
    // Configuration too long to store.
    let mut cc = CC_16_2.to_vec();
    cc.extend_from_slice(&[18, 0x77]);
    cc.extend_from_slice(&[0; 17]);
    assert_eq!(
        cp(&mut ascs, &config_codec(&[codec_entry(1, &cc)]), &mut fx),
        [1, 1, 1, 0x0D, 0x00]
    );
    assert_eq!(ascs.state(1), Some(AseState::Idle));
}

#[test]
fn qos_parameter_checks() {
    let mut ascs = Ascs::<1>::new();
    let mut fx = Effects::default();
    cp(
        &mut ascs,
        &config_codec(&[codec_entry(1, &CC_16_2)]),
        &mut fx,
    );
    fx.clear();
    let check =
        |ascs: &mut Ascs<1>, fx: &mut Effects, at: usize, val: &[u8], code: u8, reason: u8| {
            let mut q = qos_entry(1, 0, 0);
            q[at..at + val.len()].copy_from_slice(val);
            assert_eq!(
                cp(ascs, &config_qos(&[q]), fx),
                [2, 1, 1, code, reason],
                "at {at} {val:02X?}"
            );
            assert_eq!(ascs.state(1), Some(AseState::CodecConfigured));
        };
    check(&mut ascs, &mut fx, 1, &[0xF0], 0x09, 0x0A); // CIG_ID out of range
    check(&mut ascs, &mut fx, 3, &[0x4C, 0x1D, 0x00], 0x07, 0x03); // 7.5 ms interval
    check(&mut ascs, &mut fx, 3, &[0x00, 0x00, 0x10], 0x09, 0x03); // interval out of range
    check(&mut ascs, &mut fx, 6, &[0x02], 0x09, 0x04); // framing RFU
    check(&mut ascs, &mut fx, 7, &[0x00], 0x09, 0x05); // no PHY
    check(&mut ascs, &mut fx, 7, &[0x08], 0x09, 0x05); // PHY RFU bit
    check(&mut ascs, &mut fx, 8, &[39, 0], 0x09, 0x06); // Max_SDU < 40 octets
    check(&mut ascs, &mut fx, 8, &[0x00, 0x10], 0x09, 0x06); // Max_SDU > 0xFFF
    check(&mut ascs, &mut fx, 11, &[4, 0], 0x09, 0x08); // latency < 5 ms
    check(&mut ascs, &mut fx, 13, &[0x0F, 0x27, 0x00], 0x07, 0x09); // PD 9999 us
    check(&mut ascs, &mut fx, 13, &[0x41, 0x9C, 0x00], 0x07, 0x09); // PD 40001 us
    // Several errors: the first parameter in error is reported (interval before PD).
    let mut q = qos_entry(1, 0, 0);
    q[3..6].copy_from_slice(&[0x4C, 0x1D, 0x00]);
    q[13..16].copy_from_slice(&[0, 0, 0]);
    assert_eq!(
        cp(&mut ascs, &config_qos(&[q]), &mut fx),
        [2, 1, 1, 0x07, 0x03]
    );
    assert!(changed(&fx).is_empty());

    // High-reliability latency, framed, 1M and RTN 13 are accepted.
    let mut q = qos_entry(1, 0, 0);
    q[6] = 0x01;
    q[7] = 0x01;
    q[10] = 13;
    q[11] = 95;
    assert_eq!(cp(&mut ascs, &config_qos(&[q]), &mut fx), [2, 1, 1, 0, 0]);
    assert_eq!(
        value(&ascs, 1),
        [
            1, 2, 0, 0, 0x10, 0x27, 0x00, 0x01, 0x01, 40, 0, 13, 95, 0, 0x40, 0x9C, 0x00
        ]
    );
}

#[test]
fn cis_mapping_conflict() {
    let mut ascs = Ascs::<2>::new();
    let mut fx = Effects::default();
    let data = config_codec(&[codec_entry(1, &CC_16_2), codec_entry(2, &CC_16_2)]);
    assert_eq!(cp(&mut ascs, &data, &mut fx), [1, 2, 1, 0, 0, 2, 0, 0]);
    let data = config_qos(&[qos_entry(1, 0, 0), qos_entry(2, 0, 0)]);
    assert_eq!(
        cp(&mut ascs, &data, &mut fx),
        [2, 2, 1, 0, 0, 2, 0x09, 0x0A]
    );
    assert_eq!(ascs.state(2), Some(AseState::CodecConfigured));
    // Reconfiguring ASE 1 onto its own CIS again is fine; ASE 2 on another CIS too.
    let data = config_qos(&[qos_entry(1, 0, 0), qos_entry(2, 0, 1)]);
    assert_eq!(cp(&mut ascs, &data, &mut fx), [2, 2, 1, 0, 0, 2, 0, 0]);
}

#[test]
fn multiple_ases_in_one_write() {
    let mut ascs = Ascs::<2>::new();
    let mut fx = Effects::default();
    let mut cc_24 = CC_16_2;
    cc_24[2] = 0x05;
    cc_24[14] = 60;
    let data = config_codec(&[codec_entry(1, &CC_16_2), codec_entry(2, &cc_24)]);
    assert_eq!(
        cp(&mut ascs, &data, &mut fx),
        [0x01, 0x02, 0x01, 0x00, 0x00, 0x02, 0x00, 0x00]
    );
    assert_eq!(changed(&fx), [1, 2]);
    assert_eq!(value(&ascs, 1), codec_configured_value(1, &CC_16_2));
    assert_eq!(value(&ascs, 2), codec_configured_value(2, &cc_24));
    fx.clear();

    let mut q2 = qos_entry(2, 0, 1);
    q2[8] = 60;
    let data = config_qos(&[qos_entry(1, 0, 0), q2]);
    assert_eq!(
        cp(&mut ascs, &data, &mut fx),
        [0x02, 0x02, 0x01, 0x00, 0x00, 0x02, 0x00, 0x00]
    );
    fx.clear();

    // Enable both with different metadata lengths.
    let data = [0x03, 0x02, 0x01, 0x04, 3, 0x02, 0x04, 0x00, 0x02, 0x00];
    assert_eq!(
        cp(&mut ascs, &data, &mut fx),
        [0x03, 0x02, 0x01, 0x00, 0x00, 0x02, 0x00, 0x00]
    );
    assert_eq!(value(&ascs, 1), [1, 3, 0, 0, 4, 3, 0x02, 0x04, 0x00]);
    assert_eq!(value(&ascs, 2), [2, 3, 0, 1, 0]);
    fx.clear();

    // Only ASE 1 gets its CIS; start both: ASE 2 fails, ASE 1 streams.
    ascs.cis_established(0, 0, 0x10, &mut fx);
    fx.clear();
    let rsp = cp(&mut ascs, &[0x04, 0x02, 0x02, 0x01], &mut fx);
    assert_eq!(rsp, [0x04, 0x02, 0x02, 0x04, 0x00, 0x01, 0x00, 0x00]);
    assert_eq!(changed(&fx), [1]);
    assert_eq!(fx.actions().len(), 1);
    fx.clear();

    // Release both: ASE 1 has a CIS, ASE 2 is deferred.
    assert_eq!(
        cp(&mut ascs, &[0x08, 0x02, 0x01, 0x02], &mut fx),
        [8, 2, 1, 0, 0, 2, 0, 0]
    );
    assert_eq!(changed(&fx), [1, 2]);
    assert_eq!(
        fx.actions(),
        [
            Action::StopStream { ase_id: 1 },
            Action::DisconnectCis { cis_handle: 0x10 }
        ]
    );
    assert!(fx.has_deferred());
    fx.clear();
    ascs.complete_deferred(&mut fx);
    assert_eq!(changed(&fx), [2]);
    assert_eq!(ascs.state(1), Some(AseState::Releasing));
    assert_eq!(ascs.state(2), Some(AseState::CodecConfigured));
    assert_eq!(value(&ascs, 2), codec_configured_value(2, &cc_24));
}

#[test]
fn release_without_cis_is_deferred() {
    let mut ascs = Ascs::<1>::new();
    let mut fx = Effects::default();
    cp(
        &mut ascs,
        &config_codec(&[codec_entry(1, &CC_16_2)]),
        &mut fx,
    );
    fx.clear();
    assert_eq!(cp(&mut ascs, &[0x08, 1, 1], &mut fx), [8, 1, 1, 0, 0]);
    assert_eq!(value(&ascs, 1), [1, 6]);
    assert!(fx.has_deferred());
    assert!(fx.actions().is_empty());
    // Released is not reachable through the control point.
    let mut fx2 = Effects::default();
    assert_eq!(cp(&mut ascs, &[0x08, 1, 1], &mut fx2), [8, 1, 1, 4, 0]);
    fx.clear();
    ascs.complete_deferred(&mut fx);
    assert_eq!(changed(&fx), [1]);
    assert!(!fx.has_deferred());
    assert_eq!(value(&ascs, 1), codec_configured_value(1, &CC_16_2));
    // Idempotent.
    fx.clear();
    ascs.complete_deferred(&mut fx);
    assert!(changed(&fx).is_empty());
}

#[test]
fn release_while_streaming() {
    let mut ascs = Ascs::<1>::new();
    to_streaming(&mut ascs);
    let mut fx = Effects::default();
    assert_eq!(cp(&mut ascs, &[0x08, 1, 1], &mut fx), [8, 1, 1, 0, 0]);
    assert_eq!(
        fx.actions(),
        [
            Action::StopStream { ase_id: 1 },
            Action::DisconnectCis {
                cis_handle: CIS_HANDLE
            }
        ]
    );
    fx.clear();
    // Unrelated handle: nothing happens.
    ascs.cis_disconnected(0x99, &mut fx);
    assert!(changed(&fx).is_empty());
    ascs.cis_disconnected(CIS_HANDLE, &mut fx);
    assert_eq!(changed(&fx), [1]);
    assert_eq!(ascs.state(1), Some(AseState::CodecConfigured));
}

#[test]
fn cis_loss() {
    // Streaming -> QoS Configured with StopStream.
    let mut ascs = Ascs::<1>::new();
    to_streaming(&mut ascs);
    let mut fx = Effects::default();
    ascs.cis_disconnected(CIS_HANDLE, &mut fx);
    assert_eq!(changed(&fx), [1]);
    assert_eq!(fx.actions(), [Action::StopStream { ase_id: 1 }]);
    assert_eq!(value(&ascs, 1), qos_configured_value(1, 0, 0));

    // Disabling -> QoS Configured.
    let mut ascs = Ascs::<1>::new();
    to_streaming(&mut ascs);
    cp(&mut ascs, &[0x05, 1, 1], &mut fx);
    fx.clear();
    ascs.cis_disconnected(CIS_HANDLE, &mut fx);
    assert_eq!(changed(&fx), [1]);
    assert!(fx.actions().is_empty());
    assert_eq!(ascs.state(1), Some(AseState::QosConfigured));

    // Enabling stays Enabling; the CIS can be re-established.
    let mut ascs = Ascs::<1>::new();
    to_enabling(&mut ascs);
    ascs.cis_established(0, 0, CIS_HANDLE, &mut fx);
    fx.clear();
    ascs.cis_disconnected(CIS_HANDLE, &mut fx);
    assert!(changed(&fx).is_empty());
    assert_eq!(ascs.state(1), Some(AseState::Enabling));
    assert!(ascs.cis_request(0, 0));
    assert_eq!(cp(&mut ascs, &[0x04, 1, 1], &mut fx), [4, 1, 1, 4, 0]);
}

#[test]
fn unmatched_cis_established_is_disconnected() {
    let mut ascs = Ascs::<1>::new();
    let mut fx = Effects::default();
    ascs.cis_established(0, 0, 0x20, &mut fx);
    assert_eq!(fx.actions(), [Action::DisconnectCis { cis_handle: 0x20 }]);

    // Released while the CIS was being set up.
    let mut ascs = Ascs::<1>::new();
    to_enabling(&mut ascs);
    assert!(ascs.cis_request(0, 0));
    fx.clear();
    cp(&mut ascs, &[0x08, 1, 1], &mut fx);
    ascs.complete_deferred(&mut fx);
    fx.clear();
    ascs.cis_established(0, 0, 0x21, &mut fx);
    assert_eq!(fx.actions(), [Action::DisconnectCis { cis_handle: 0x21 }]);
    assert!(changed(&fx).is_empty());
}

#[test]
fn metadata_checks() {
    let mut ascs = Ascs::<1>::new();
    let mut fx = Effects::default();
    cp(
        &mut ascs,
        &config_codec(&[codec_entry(1, &CC_16_2)]),
        &mut fx,
    );
    cp(&mut ascs, &config_qos(&[qos_entry(1, 0, 0)]), &mut fx);
    fx.clear();
    // Sound Effects is not an available source context -> Rejected Metadata, type 0x02.
    assert_eq!(
        cp(&mut ascs, &[3, 1, 1, 4, 3, 0x02, 0x80, 0x00], &mut fx),
        [3, 1, 1, 0x0B, 0x02]
    );
    // Zero context -> Invalid Metadata.
    assert_eq!(
        cp(&mut ascs, &[3, 1, 1, 4, 3, 0x02, 0x00, 0x00], &mut fx),
        [3, 1, 1, 0x0C, 0x02]
    );
    // Wrong context length -> Invalid Metadata.
    assert_eq!(
        cp(&mut ascs, &[3, 1, 1, 3, 2, 0x01, 0x02], &mut fx),
        [3, 1, 1, 0x0C, 0x01]
    );
    // LTV overrun -> Invalid Metadata with the type in error.
    assert_eq!(
        cp(&mut ascs, &[3, 1, 1, 3, 5, 0x05, 0x01], &mut fx),
        [3, 1, 1, 0x0C, 0x05]
    );
    // Too long to store.
    let mut data = vec![3, 1, 1, 40, 39, 0xFF];
    data.extend_from_slice(&[0; 38]);
    assert_eq!(cp(&mut ascs, &data, &mut fx), [3, 1, 1, 0x0D, 0x00]);
    assert!(changed(&fx).is_empty());
    assert_eq!(ascs.state(1), Some(AseState::QosConfigured));

    // Vendor-specific metadata plus a valid context is accepted verbatim.
    let data = [3, 1, 1, 8, 3, 0x02, 0x20, 0x00, 3, 0xFF, 0xAB, 0xCD];
    assert_eq!(cp(&mut ascs, &data, &mut fx), [3, 1, 1, 0, 0]);
    assert_eq!(
        value(&ascs, 1),
        [1, 3, 0, 0, 8, 3, 0x02, 0x20, 0x00, 3, 0xFF, 0xAB, 0xCD]
    );
    fx.clear();

    // Update Metadata while Enabling replaces it.
    assert_eq!(
        cp(&mut ascs, &[7, 1, 1, 4, 3, 0x02, 0x04, 0x00], &mut fx),
        [7, 1, 1, 0, 0]
    );
    assert_eq!(changed(&fx), [1]);
    assert_eq!(value(&ascs, 1), [1, 3, 0, 0, 4, 3, 0x02, 0x04, 0x00]);
    // Rejected update leaves it untouched.
    assert_eq!(
        cp(&mut ascs, &[7, 1, 1, 4, 3, 0x02, 0x00, 0x01], &mut fx),
        [7, 1, 1, 0x0B, 0x02]
    );
    assert_eq!(value(&ascs, 1), [1, 3, 0, 0, 4, 3, 0x02, 0x04, 0x00]);
}

#[test]
fn update_metadata_while_streaming() {
    let mut ascs = Ascs::<1>::new();
    to_streaming(&mut ascs);
    let mut fx = Effects::default();
    assert_eq!(cp(&mut ascs, &[7, 1, 1, 0], &mut fx), [7, 1, 1, 0, 0]);
    assert_eq!(changed(&fx), [1]);
    assert!(fx.actions().is_empty());
    assert_eq!(value(&ascs, 1), [1, 4, 0, 0, 0]);
    // Not in Disabling.
    cp(&mut ascs, &[0x05, 1, 1], &mut fx);
    assert_eq!(cp(&mut ascs, &[7, 1, 1, 0], &mut fx), [7, 1, 1, 4, 0]);
}

#[test]
fn disable_from_enabling() {
    let mut ascs = Ascs::<1>::new();
    to_enabling(&mut ascs);
    let mut fx = Effects::default();
    assert_eq!(cp(&mut ascs, &[0x05, 1, 1], &mut fx), [5, 1, 1, 0, 0]);
    assert!(fx.actions().is_empty());
    assert_eq!(ascs.state(1), Some(AseState::Disabling));
    assert_eq!(cp(&mut ascs, &[0x06, 1, 1], &mut fx), [6, 1, 1, 0, 0]);
    assert_eq!(ascs.state(1), Some(AseState::QosConfigured));
}

#[test]
fn reconfigure_codec_from_qos_configured() {
    let mut ascs = Ascs::<1>::new();
    let mut fx = Effects::default();
    cp(
        &mut ascs,
        &config_codec(&[codec_entry(1, &CC_16_2)]),
        &mut fx,
    );
    cp(&mut ascs, &config_qos(&[qos_entry(1, 0, 0)]), &mut fx);
    let mut cc_32 = CC_16_2;
    cc_32[2] = 0x06;
    cc_32[14] = 80;
    assert_eq!(
        cp(&mut ascs, &config_codec(&[codec_entry(1, &cc_32)]), &mut fx),
        [1, 1, 1, 0, 0]
    );
    assert_eq!(value(&ascs, 1), codec_configured_value(1, &cc_32));
    // Max_SDU 40 no longer holds an 80-octet frame.
    assert_eq!(
        cp(&mut ascs, &config_qos(&[qos_entry(1, 0, 0)]), &mut fx),
        [2, 1, 1, 0x09, 0x06]
    );
}

#[test]
fn acl_disconnect_resets() {
    let mut ascs = Ascs::<2>::new();
    let mut fx = Effects::default();
    to_streaming(&mut ascs);
    cp(
        &mut ascs,
        &config_codec(&[codec_entry(2, &CC_16_2)]),
        &mut fx,
    );
    fx.clear();
    ascs.acl_disconnected(&mut fx);
    assert_eq!(fx.actions(), [Action::StopStream { ase_id: 1 }]);
    assert!(changed(&fx).is_empty());
    assert_eq!(ascs.state(1), Some(AseState::Idle));
    assert_eq!(ascs.state(2), Some(AseState::Idle));
    assert_eq!(value(&ascs, 1), [1, 0]);
    assert_eq!(value(&ascs, 2), [2, 0]);
    // The old CIS is forgotten.
    fx.clear();
    ascs.cis_disconnected(CIS_HANDLE, &mut fx);
    assert!(changed(&fx).is_empty() && fx.actions().is_empty());
    // A fresh session works.
    to_streaming(&mut ascs);
}

#[test]
fn effects_clear() {
    let mut ascs = Ascs::<1>::new();
    let mut fx = Effects::default();
    to_enabling(&mut ascs);
    ascs.cis_established(0, 0, 1, &mut fx);
    cp(&mut ascs, &[0x08, 1, 1], &mut fx);
    assert!(!fx.actions().is_empty());
    fx.clear();
    assert!(fx.actions().is_empty());
    assert!(changed(&fx).is_empty());
    assert!(!fx.has_deferred());
}

#[test]
fn const_construction() {
    const _A: Ascs<1> = Ascs::new();
    const _B: Ascs<4> = Ascs::new();
    assert_eq!(ASE_VALUE_MAX_LEN, 57);
    assert_eq!(CP_RESPONSE_MAX_LEN, 14);
}
