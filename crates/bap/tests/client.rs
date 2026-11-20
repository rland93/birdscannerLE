use bap::ascs::{
    ASE_VALUE_MAX_LEN, Action, Ascs, AseState, CP_RESPONSE_MAX_LEN, CodecConfig, Effects,
    QosConfig, opcode, reason, response,
};
use bap::client::{
    self, AseStatus, CP_PDU_MAX_LEN, CpEntry, ParseError, QosPreferences, StreamParams,
    target_latency, target_phy,
};
use bap::{context, pacs};

const fn lc3(sampling_hz: u32, octets_per_frame: u16, audio_locations: u32) -> CodecConfig {
    CodecConfig {
        sampling_hz,
        frame_duration_us: 10_000,
        octets_per_frame,
        frames_per_sdu: 1,
        audio_locations,
    }
}

const CODEC_16_2: CodecConfig = lc3(16_000, 40, 1);

/// Server preferences of `bap::ascs` (see `ascs::qos_pref`).
const SERVER_PREFS: QosPreferences = QosPreferences {
    framing: 0x00,
    phy: 0x03,
    retransmission_number: 2,
    max_transport_latency_ms: 10,
    presentation_delay_min_us: 10_000,
    presentation_delay_max_us: 40_000,
    preferred_presentation_delay_min_us: 0,
    preferred_presentation_delay_max_us: 0,
};

/// LC3 16_2 Front Left, all five LTVs, as built by `client::config_codec`.
const CC_16_2: [u8; 19] = [
    2, 0x01, 0x03, // Sampling_Frequency 16 kHz
    2, 0x02, 0x01, // Frame_Duration 10 ms
    5, 0x03, 0x01, 0x00, 0x00, 0x00, // Audio_Channel_Allocation Front Left
    3, 0x04, 40, 0, // Octets_Per_Codec_Frame 40
    2, 0x05, 0x01, // Codec_Frame_Blocks_Per_SDU 1
];

fn server_pac() -> Vec<u8> {
    let mut out = [0u8; pacs::SOURCE_PAC_MAX_LEN];
    let n = pacs::source_pac(&mut out);
    out[..n].to_vec()
}

// ---------------------------------------------------------------------------
// PAC

#[test]
fn parse_server_source_pac() {
    let value = server_pac();
    let pac = client::parse_pac(&value).unwrap();
    assert_eq!(pac.len(), 1);
    assert!(!pac.is_empty());
    let records: Vec<_> = pac.records().collect();
    assert_eq!(records.len(), 1);
    let r = &records[0];
    assert!(r.is_lc3());
    assert_eq!(r.codec_id, [0x06, 0, 0, 0, 0]);
    assert_eq!(r.codec_specific_capabilities, &value[7..26]);
    assert_eq!(r.metadata, &[] as &[u8]);
    assert_eq!(r.sampling_frequencies, 0x0034);
    assert_eq!(r.frame_durations, 0x02);
    assert_eq!(r.channel_counts, 0x01);
    assert_eq!(r.octets_per_frame_min, 26);
    assert_eq!(r.octets_per_frame_max, 80);
    assert_eq!(r.max_frames_per_sdu, 1);
    assert_eq!(r.preferred_contexts, None);
}

/// Vendor record, then an LC3 record with metadata and every capability LTV, then
/// an LC3 record with only the mandatory LTVs (defaults apply).
const MULTI_PAC: [u8; 64] = [
    0x03, // Number_of_PAC_records
    // Record 1: vendor codec, opaque capabilities, no metadata.
    0xFF, 0x59, 0x00, 0x01, 0x00, // Codec_ID: vendor 0x0059, codec 0x0001
    0x02, 0xAA, 0xBB, // capabilities (not LTV)
    0x00, // Metadata_Length
    // Record 2: LC3.
    0x06, 0x00, 0x00, 0x00, 0x00, // LC3
    0x17, // Codec_Specific_Capabilities_Length = 23
    0x03, 0x01, 0x84, 0x00, // 16 kHz | 48 kHz (bits 2, 7)
    0x02, 0x02, 0x23, // 7.5 ms | 10 ms | 10 ms preferred
    0x02, 0x03, 0x03, // 1 or 2 channels
    0x05, 0x04, 0x1E, 0x00, 0x9B, 0x00, // 30..=155 octets
    0x02, 0x05, 0x02, // max 2 frames per SDU
    0x03, 0xF0, 0x01, 0x02, // unknown type 0xF0, ignored
    0x04, // Metadata_Length
    0x03, 0x01, 0x06, 0x00, // Preferred_Audio_Contexts: Conversational | Media
    // Record 3: LC3, mandatory LTVs only.
    0x06, 0x00, 0x00, 0x00, 0x00, // LC3
    0x0D, // Codec_Specific_Capabilities_Length = 13
    0x03, 0x01, 0x80, 0x00, // 48 kHz
    0x02, 0x02, 0x02, // 10 ms
    0x05, 0x04, 0x64, 0x00, 0x78, 0x00, // 100..=120 octets
    0x00, // Metadata_Length
];

#[test]
fn parse_multi_record_pac() {
    let pac = client::parse_pac(&MULTI_PAC).unwrap();
    assert_eq!(pac.len(), 3);
    let r: Vec<_> = pac.records().collect();
    assert_eq!(r.len(), 3);

    assert!(!r[0].is_lc3());
    assert_eq!(r[0].codec_id, [0xFF, 0x59, 0x00, 0x01, 0x00]);
    assert_eq!(r[0].codec_specific_capabilities, &[0xAA, 0xBB]);
    assert_eq!(r[0].sampling_frequencies, 0);
    assert_eq!(r[0].frame_durations, 0);
    assert_eq!(r[0].octets_per_frame_max, 0);
    assert_eq!(r[0].preferred_contexts, None);

    assert!(r[1].is_lc3());
    assert_eq!(r[1].sampling_frequencies, 0x0084);
    assert_eq!(r[1].frame_durations, 0x23);
    assert_eq!(r[1].channel_counts, 0x03);
    assert_eq!(r[1].octets_per_frame_min, 30);
    assert_eq!(r[1].octets_per_frame_max, 155);
    assert_eq!(r[1].max_frames_per_sdu, 2);
    assert_eq!(r[1].metadata, &[0x03, 0x01, 0x06, 0x00]);
    assert_eq!(
        r[1].preferred_contexts,
        Some(context::CONVERSATIONAL | context::MEDIA)
    );

    assert!(r[2].is_lc3());
    assert_eq!(r[2].sampling_frequencies, 0x0080);
    assert_eq!(r[2].frame_durations, 0x02);
    assert_eq!(r[2].channel_counts, 0x01, "default: 1 channel");
    assert_eq!(r[2].octets_per_frame_min, 100);
    assert_eq!(r[2].octets_per_frame_max, 120);
    assert_eq!(r[2].max_frames_per_sdu, 1, "default: 1 frame");
    assert_eq!(r[2].preferred_contexts, None);

    // Iterating again yields the same records.
    assert_eq!(pac.records().collect::<Vec<_>>(), r);
}

#[test]
fn parse_pac_errors() {
    assert_eq!(client::parse_pac(&[]), Err(ParseError::Truncated));
    // Two records announced, one present.
    let mut v = server_pac();
    v[0] = 2;
    assert_eq!(client::parse_pac(&v), Err(ParseError::Truncated));
    // Record cut inside the capabilities.
    assert_eq!(
        client::parse_pac(&server_pac()[..20]),
        Err(ParseError::Truncated)
    );
    // Octet after the last record.
    let mut v = server_pac();
    v.push(0);
    assert_eq!(client::parse_pac(&v), Err(ParseError::TrailingData));
    // Zero records is valid.
    let empty = client::parse_pac(&[0x00]).unwrap();
    assert!(empty.is_empty());
    assert_eq!(empty.records().count(), 0);
    // Capability LTV overrunning its block.
    let v = [
        0x01, 0x06, 0, 0, 0, 0, //
        0x03, 0x03, 0x01, 0x34, // LTV claims 3 octets, 2 present
        0x00,
    ];
    assert_eq!(client::parse_pac(&v), Err(ParseError::MalformedLtv));
    // Supported_Sampling_Frequencies with one octet.
    let v = [0x01, 0x06, 0, 0, 0, 0, 0x03, 0x02, 0x01, 0x34, 0x00];
    assert_eq!(client::parse_pac(&v), Err(ParseError::MalformedLtv));
    // Metadata LTV overrunning its block.
    let v = [0x01, 0x06, 0, 0, 0, 0, 0x00, 0x03, 0x03, 0x01, 0x06];
    assert_eq!(client::parse_pac(&v), Err(ParseError::MalformedLtv));
    // Preferred_Audio_Contexts with three octets.
    let v = [
        0x01, 0x06, 0, 0, 0, 0, 0x00, 0x05, 0x04, 0x01, 0x06, 0x00, 0x00,
    ];
    assert_eq!(client::parse_pac(&v), Err(ParseError::MalformedLtv));
}

#[test]
fn choose_from_server_pac() {
    let value = server_pac();
    let pac = client::parse_pac(&value).unwrap();
    for (hz, octets) in [(16_000, 40), (24_000, 60), (32_000, 80)] {
        assert_eq!(
            client::choose_codec_config(&pac, hz, octets, 1),
            Some(lc3(hz, octets, 1)),
            "{hz} Hz / {octets} octets"
        );
        // Mono without a location is fine too.
        assert_eq!(
            client::choose_codec_config(&pac, hz, octets, 0),
            Some(lc3(hz, octets, 0))
        );
    }
    // Unsupported rate, rate without a code, octets outside 26..=80, two channels.
    assert_eq!(client::choose_codec_config(&pac, 48_000, 80, 1), None);
    assert_eq!(client::choose_codec_config(&pac, 12_345, 40, 1), None);
    assert_eq!(client::choose_codec_config(&pac, 16_000, 25, 1), None);
    assert_eq!(client::choose_codec_config(&pac, 32_000, 81, 1), None);
    assert_eq!(client::choose_codec_config(&pac, 16_000, 40, 0x3), None);
}

#[test]
fn choose_from_multi_record_pac() {
    let pac = client::parse_pac(&MULTI_PAC).unwrap();
    // Record 2: 48 kHz stereo (2 frames per SDU allowed), 155 octets.
    assert_eq!(
        client::choose_codec_config(&pac, 48_000, 155, 0x3),
        Some(lc3(48_000, 155, 0x3))
    );
    // Record 3 does not support 16 kHz but record 2 does.
    assert_eq!(
        client::choose_codec_config(&pac, 16_000, 30, 1),
        Some(lc3(16_000, 30, 1))
    );
    // 24 kHz: no record.
    assert_eq!(client::choose_codec_config(&pac, 24_000, 60, 1), None);
    // Three channels: no record supports them.
    assert_eq!(client::choose_codec_config(&pac, 48_000, 100, 0x7), None);

    let r: Vec<_> = pac.records().collect();
    // 7.5 ms is supported by record 2 only.
    let short = CodecConfig {
        frame_duration_us: 7_500,
        ..lc3(48_000, 100, 1)
    };
    assert!(r[1].supports(&short));
    assert!(!r[2].supports(&short));
    // Two blocks of stereo = 4 frames > 2.
    let blocks = CodecConfig {
        frames_per_sdu: 2,
        ..lc3(48_000, 100, 0x3)
    };
    assert!(!r[1].supports(&blocks));
    assert!(r[1].supports(&CodecConfig {
        audio_locations: 1,
        ..blocks
    }));
    assert!(!r[1].supports(&CodecConfig {
        frames_per_sdu: 0,
        ..blocks
    }));
    // The vendor record supports no LC3 configuration.
    assert!(!r[0].supports(&lc3(16_000, 40, 1)));
}

// ---------------------------------------------------------------------------
// Control Point PDUs

#[test]
fn build_config_codec() {
    let mut out = [0xEEu8; CP_PDU_MAX_LEN];
    let n = client::config_codec(
        &mut out,
        1,
        target_latency::LOW_LATENCY,
        target_phy::LE_2M,
        &CODEC_16_2,
    )
    .unwrap();
    let mut expected = vec![
        0x01, // Config Codec
        0x01, // Number_of_ASEs
        0x01, // ASE_ID
        0x01, // Target_Latency: low latency
        0x02, // Target_PHY: LE 2M
        0x06, 0x00, 0x00, 0x00, 0x00, // LC3
        19,   // Codec_Specific_Configuration_Length
    ];
    expected.extend_from_slice(&CC_16_2);
    assert_eq!(n, 30);
    assert_eq!(n, CP_PDU_MAX_LEN);
    assert_eq!(&out[..n], expected);

    // 32_2, Front Right, 7.5 ms, other target values.
    let codec = CodecConfig {
        frame_duration_us: 7_500,
        ..lc3(32_000, 80, 0x2)
    };
    let n = client::config_codec(
        &mut out,
        3,
        target_latency::HIGH_RELIABILITY,
        target_phy::LE_CODED,
        &codec,
    )
    .unwrap();
    assert_eq!(
        &out[..n],
        [
            0x01, 0x01, 0x03, 0x03, 0x03, 0x06, 0, 0, 0, 0, 19, //
            2, 0x01, 0x06, //
            2, 0x02, 0x00, //
            5, 0x03, 0x02, 0, 0, 0, //
            3, 0x04, 80, 0, //
            2, 0x05, 0x01,
        ]
    );
    assert_eq!(target_latency::BALANCED, 0x02);
    assert_eq!(target_phy::LE_1M, 0x01);

    // No code for these: nothing written.
    let mut out = [0xEEu8; CP_PDU_MAX_LEN];
    assert_eq!(
        client::config_codec(&mut out, 1, 1, 2, &lc3(12_000, 40, 1)),
        None
    );
    let bad_duration = CodecConfig {
        frame_duration_us: 5_000,
        ..CODEC_16_2
    };
    assert_eq!(client::config_codec(&mut out, 1, 1, 2, &bad_duration), None);
    assert_eq!(out, [0xEE; CP_PDU_MAX_LEN]);
}

#[test]
fn build_config_qos() {
    let qos = QosConfig {
        cig_id: 0x05,
        cis_id: 0x07,
        sdu_interval_us: 10_000,
        framing: 0x00,
        phy: 0x02,
        max_sdu: 40,
        retransmission_number: 2,
        max_transport_latency_ms: 10,
        presentation_delay_us: 40_000,
    };
    let mut out = [0xEEu8; CP_PDU_MAX_LEN];
    let n = client::config_qos(&mut out, 1, &qos);
    assert_eq!(
        &out[..n],
        [
            0x02, // Config QoS
            0x01, // Number_of_ASEs
            0x01, // ASE_ID
            0x05, // CIG_ID
            0x07, // CIS_ID
            0x10, 0x27, 0x00, // SDU_Interval 10000
            0x00, // Framing unframed
            0x02, // PHY 2M
            0x28, 0x00, // Max_SDU 40
            0x02, // RTN
            0x0A, 0x00, // Max_Transport_Latency 10
            0x40, 0x9C, 0x00, // Presentation_Delay 40000
        ]
    );
    let qos = QosConfig {
        sdu_interval_us: 0x0F_FFFF,
        max_sdu: 0x0123,
        max_transport_latency_ms: 0x0FA0,
        presentation_delay_us: 0x12_3456,
        framing: 0x01,
        phy: 0x04,
        retransmission_number: 0xFF,
        ..qos
    };
    let n = client::config_qos(&mut out, 0x42, &qos);
    assert_eq!(
        &out[..n],
        [
            0x02, 0x01, 0x42, 0x05, 0x07, 0xFF, 0xFF, 0x0F, 0x01, 0x04, 0x23, 0x01, 0xFF, 0xA0,
            0x0F, 0x56, 0x34, 0x12,
        ]
    );
}

#[test]
fn build_simple_operations() {
    let mut out = [0xEEu8; CP_PDU_MAX_LEN];
    let n = client::enable(&mut out, 1, context::LIVE);
    assert_eq!(
        &out[..n],
        [
            0x03, // Enable
            0x01, // Number_of_ASEs
            0x01, // ASE_ID
            0x04, // Metadata_Length
            0x03, 0x02, 0x40, 0x00, // Streaming_Audio_Contexts: Live
        ]
    );
    let n = client::enable(&mut out, 2, context::CONVERSATIONAL | context::MEDIA);
    assert_eq!(&out[..n], [0x03, 0x01, 0x02, 0x04, 0x03, 0x02, 0x06, 0x00]);
    let n = client::receiver_start_ready(&mut out, 1);
    assert_eq!(&out[..n], [0x04, 0x01, 0x01]);
    let n = client::disable(&mut out, 2);
    assert_eq!(&out[..n], [0x05, 0x01, 0x02]);
    let n = client::receiver_stop_ready(&mut out, 3);
    assert_eq!(&out[..n], [0x06, 0x01, 0x03]);
    let n = client::release(&mut out, 4);
    assert_eq!(&out[..n], [0x08, 0x01, 0x04]);
}

// ---------------------------------------------------------------------------
// ASE values

#[test]
fn parse_ase_idle_and_releasing() {
    let v = client::parse_ase(&[0x01, 0x00]).unwrap();
    assert_eq!(v.ase_id, 1);
    assert_eq!(v.status, AseStatus::Idle);
    assert_eq!(v.status.state(), AseState::Idle);
    let v = client::parse_ase(&[0x02, 0x06]).unwrap();
    assert_eq!(v.ase_id, 2);
    assert_eq!(v.status, AseStatus::Releasing);
    assert_eq!(v.status.state(), AseState::Releasing);
}

/// Codec Configured value with the bap::ascs preferences and 16_2.
fn codec_configured_bytes() -> Vec<u8> {
    let mut v = vec![
        0x01, // ASE_ID
        0x01, // Codec Configured
        0x00, // Framing: unframed supported
        0x03, // Preferred_PHY: 1M | 2M
        0x02, // Preferred_Retransmission_Number
        0x0A, 0x00, // Max_Transport_Latency 10 ms
        0x10, 0x27, 0x00, // Presentation_Delay_Min 10000
        0x40, 0x9C, 0x00, // Presentation_Delay_Max 40000
        0x00, 0x00, 0x00, // Preferred_Presentation_Delay_Min: none
        0x00, 0x00, 0x00, // Preferred_Presentation_Delay_Max: none
        0x06, 0x00, 0x00, 0x00, 0x00, // LC3
        19,
    ];
    v.extend_from_slice(&CC_16_2);
    v
}

#[test]
fn parse_ase_codec_configured() {
    let bytes = codec_configured_bytes();
    let v = client::parse_ase(&bytes).unwrap();
    assert_eq!(v.ase_id, 1);
    assert_eq!(v.status.state(), AseState::CodecConfigured);
    let AseStatus::CodecConfigured(cc) = v.status else {
        panic!("{v:?}");
    };
    assert_eq!(cc.preferences, SERVER_PREFS);
    assert_eq!(cc.codec_id, pacs::LC3_CODEC_ID);
    assert_eq!(cc.codec_config, CC_16_2);
    assert_eq!(cc.codec, Some(CODEC_16_2));

    // Distinct preference values land in the right fields.
    let v = client::parse_ase(&[
        0x05, 0x01, 0x01, 0x04, 0x0D, 0x34, 0x12, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
        0x09, 0x0A, 0x0B, 0x0C, 0x06, 0, 0, 0, 0, 0x0A, //
        2, 0x01, 0x08, // 48 kHz
        2, 0x02, 0x00, // 7.5 ms
        3, 0x04, 0x9B, 0x00, // 155 octets (no allocation, no blocks: defaults)
    ])
    .unwrap();
    assert_eq!(v.ase_id, 5);
    let AseStatus::CodecConfigured(cc) = v.status else {
        panic!("{v:?}");
    };
    assert_eq!(
        cc.preferences,
        QosPreferences {
            framing: 0x01,
            phy: 0x04,
            retransmission_number: 0x0D,
            max_transport_latency_ms: 0x1234,
            presentation_delay_min_us: 0x03_0201,
            presentation_delay_max_us: 0x06_0504,
            preferred_presentation_delay_min_us: 0x09_0807,
            preferred_presentation_delay_max_us: 0x0C_0B0A,
        }
    );
    assert_eq!(
        cc.codec,
        Some(CodecConfig {
            sampling_hz: 48_000,
            frame_duration_us: 7_500,
            octets_per_frame: 155,
            frames_per_sdu: 1,
            audio_locations: 0,
        })
    );

    // Vendor codec: configuration not decoded.
    let mut vendor = codec_configured_bytes();
    vendor[19..24].copy_from_slice(&[0xFF, 0x59, 0x00, 0x01, 0x00]);
    let AseStatus::CodecConfigured(cc) = client::parse_ase(&vendor).unwrap().status else {
        panic!();
    };
    assert_eq!(cc.codec, None);
    assert_eq!(cc.codec_config, CC_16_2);
}

#[test]
fn parse_ase_qos_configured() {
    let v = client::parse_ase(&[
        0x01, 0x02, // ASE 1, QoS Configured
        0x03, 0x04, // CIG_ID, CIS_ID
        0x10, 0x27, 0x00, // SDU_Interval 10000
        0x01, // framed
        0x02, // 2M
        0x3C, 0x00, // Max_SDU 60
        0x05, // RTN
        0x14, 0x00, // 20 ms
        0x20, 0x4E, 0x00, // 20000 us
    ])
    .unwrap();
    assert_eq!(v.ase_id, 1);
    assert_eq!(
        v.status,
        AseStatus::QosConfigured(QosConfig {
            cig_id: 3,
            cis_id: 4,
            sdu_interval_us: 10_000,
            framing: 1,
            phy: 2,
            max_sdu: 60,
            retransmission_number: 5,
            max_transport_latency_ms: 20,
            presentation_delay_us: 20_000,
        })
    );
    assert_eq!(v.status.state(), AseState::QosConfigured);
}

#[test]
fn parse_ase_enabling_streaming_disabling() {
    let md = [0x03, 0x02, 0x40, 0x00, 0x02, 0xF0, 0x01];
    let mut bytes = vec![0x02, 0x03, 0x01, 0x02, md.len() as u8];
    bytes.extend_from_slice(&md);
    let expected = StreamParams {
        cig_id: 1,
        cis_id: 2,
        metadata: &md,
        streaming_contexts: Some(context::LIVE),
    };
    assert_eq!(
        client::parse_ase(&bytes).unwrap().status,
        AseStatus::Enabling(expected)
    );
    bytes[1] = 0x04;
    let v = client::parse_ase(&bytes).unwrap();
    assert_eq!(v.ase_id, 2);
    assert_eq!(v.status, AseStatus::Streaming(expected));
    assert_eq!(v.status.state(), AseState::Streaming);
    bytes[1] = 0x05;
    assert_eq!(
        client::parse_ase(&bytes).unwrap().status,
        AseStatus::Disabling(expected)
    );

    // No metadata.
    assert_eq!(
        client::parse_ase(&[0x01, 0x04, 0x00, 0x00, 0x00])
            .unwrap()
            .status,
        AseStatus::Streaming(StreamParams {
            cig_id: 0,
            cis_id: 0,
            metadata: &[],
            streaming_contexts: None,
        })
    );
}

#[test]
fn parse_ase_errors() {
    assert_eq!(client::parse_ase(&[]), Err(ParseError::Truncated));
    assert_eq!(client::parse_ase(&[0x01]), Err(ParseError::Truncated));
    assert_eq!(
        client::parse_ase(&[0x01, 0x07]),
        Err(ParseError::InvalidValue)
    );
    assert_eq!(
        client::parse_ase(&[0x01, 0x00, 0x00]),
        Err(ParseError::TrailingData)
    );
    assert_eq!(
        client::parse_ase(&[0x01, 0x06, 0x00]),
        Err(ParseError::TrailingData)
    );
    // Notification truncated at the default ATT_MTU (20 octets).
    assert_eq!(
        client::parse_ase(&codec_configured_bytes()[..20]),
        Err(ParseError::Truncated)
    );
    // QoS Configured one octet short.
    assert_eq!(
        client::parse_ase(&[
            0x01, 0x02, 0, 0, 0x10, 0x27, 0, 0, 2, 40, 0, 2, 10, 0, 0x40, 0x9C
        ]),
        Err(ParseError::Truncated)
    );
    // Metadata_Length beyond the value.
    assert_eq!(
        client::parse_ase(&[0x01, 0x03, 0, 0, 0x05, 0x03, 0x02]),
        Err(ParseError::Truncated)
    );
    // Metadata LTV overrun inside its block.
    assert_eq!(
        client::parse_ase(&[0x01, 0x03, 0, 0, 0x02, 0x03, 0x02]),
        Err(ParseError::MalformedLtv)
    );
    // Streaming_Audio_Contexts of one octet.
    assert_eq!(
        client::parse_ase(&[0x01, 0x03, 0, 0, 0x03, 0x02, 0x02, 0x40]),
        Err(ParseError::MalformedLtv)
    );

    // LC3 configuration errors.
    let with_cc = |cc: &[u8]| {
        let mut v = codec_configured_bytes()[..24].to_vec();
        v.push(cc.len() as u8);
        v.extend_from_slice(cc);
        client::parse_ase(&v).map(|_| ())
    };
    assert!(with_cc(&CC_16_2).is_ok());
    // Octets_Per_Codec_Frame missing.
    assert_eq!(with_cc(&CC_16_2[..12]), Err(ParseError::MissingLtv));
    // Sampling_Frequency code 0x0E undefined.
    assert_eq!(
        with_cc(&[2, 0x01, 0x0E, 2, 0x02, 0x01, 3, 0x04, 40, 0]),
        Err(ParseError::InvalidValue)
    );
    // Frame_Duration code 0x02 undefined.
    assert_eq!(
        with_cc(&[2, 0x01, 0x03, 2, 0x02, 0x02, 3, 0x04, 40, 0]),
        Err(ParseError::InvalidValue)
    );
    // Octets_Per_Codec_Frame of one octet.
    assert_eq!(
        with_cc(&[2, 0x01, 0x03, 2, 0x02, 0x01, 2, 0x04, 40]),
        Err(ParseError::MalformedLtv)
    );
    // LTV overrun.
    assert_eq!(
        with_cc(&[2, 0x01, 0x03, 4, 0x02]),
        Err(ParseError::MalformedLtv)
    );
}

// ---------------------------------------------------------------------------
// Control Point notifications

#[test]
fn parse_cp_responses() {
    let rsp = client::parse_cp_response(&[0x01, 0x01, 0x01, 0x00, 0x00]).unwrap();
    assert_eq!(rsp.opcode, opcode::CONFIG_CODEC);
    assert!(!rsp.is_operation_error());
    let ok = CpEntry {
        ase_id: 1,
        response_code: response::SUCCESS,
        reason: reason::NONE,
    };
    assert_eq!(rsp.entries().collect::<Vec<_>>(), [ok]);
    assert_eq!(rsp.entry(1), Some(ok));
    assert!(ok.is_success());
    assert_eq!(rsp.entry(2), None);

    let rsp = client::parse_cp_response(&[0x02, 0x02, 0x01, 0x00, 0x00, 0x02, 0x09, 0x0A]).unwrap();
    assert_eq!(rsp.opcode, opcode::CONFIG_QOS);
    assert_eq!(rsp.entries().len(), 2);
    let second = rsp.entry(2).unwrap();
    assert_eq!(
        second,
        CpEntry {
            ase_id: 2,
            response_code: response::INVALID_CONFIGURATION_PARAMETER_VALUE,
            reason: reason::INVALID_ASE_CIS_MAPPING,
        }
    );
    assert!(!second.is_success());

    // Operation-level error.
    let rsp = client::parse_cp_response(&[0x09, 0xFF, 0x00, 0x01, 0x00]).unwrap();
    assert_eq!(rsp.opcode, 0x09);
    assert!(rsp.is_operation_error());
    let err = CpEntry {
        ase_id: 0,
        response_code: response::UNSUPPORTED_OPCODE,
        reason: 0,
    };
    assert_eq!(rsp.entry(7), Some(err));
    assert_eq!(rsp.entries().collect::<Vec<_>>(), [err]);

    assert_eq!(client::parse_cp_response(&[]), Err(ParseError::Truncated));
    assert_eq!(
        client::parse_cp_response(&[0x01]),
        Err(ParseError::Truncated)
    );
    assert_eq!(
        client::parse_cp_response(&[0x01, 0x02, 0x01, 0x00, 0x00]),
        Err(ParseError::Truncated)
    );
    assert_eq!(
        client::parse_cp_response(&[0x01, 0xFF, 0x00, 0x02]),
        Err(ParseError::Truncated)
    );
    assert_eq!(
        client::parse_cp_response(&[0x01, 0x01, 0x01, 0x00, 0x00, 0x00]),
        Err(ParseError::TrailingData)
    );
}

// ---------------------------------------------------------------------------
// QoS derivation

#[test]
fn derive_qos_from_server_prefs() {
    assert_eq!(
        client::derive_qos(&SERVER_PREFS, &CODEC_16_2, 0, 1),
        QosConfig {
            cig_id: 0,
            cis_id: 1,
            sdu_interval_us: 10_000,
            framing: 0x00,
            phy: 0x02,
            max_sdu: 40,
            retransmission_number: 2,
            max_transport_latency_ms: 10,
            presentation_delay_us: 10_000,
        }
    );
    assert_eq!(
        client::derive_qos(&SERVER_PREFS, &lc3(32_000, 80, 1), 3, 4).max_sdu,
        80
    );
}

#[test]
fn derive_qos_variants() {
    // Framed only, 1M only, latency below the valid minimum, preferred delay in range.
    let prefs = QosPreferences {
        framing: 0x01,
        phy: 0x01,
        retransmission_number: 13,
        max_transport_latency_ms: 0,
        presentation_delay_min_us: 20_000,
        presentation_delay_max_us: 40_000,
        preferred_presentation_delay_min_us: 25_000,
        preferred_presentation_delay_max_us: 30_000,
    };
    let q = client::derive_qos(&prefs, &lc3(48_000, 100, 0x3), 1, 2);
    assert_eq!(
        q,
        QosConfig {
            cig_id: 1,
            cis_id: 2,
            sdu_interval_us: 10_000,
            framing: 0x01,
            phy: 0x01,
            max_sdu: 200, // two channels
            retransmission_number: 13,
            max_transport_latency_ms: 5,
            presentation_delay_us: 25_000,
        }
    );
    // Coded only; latency above the valid maximum; preferred delay above the range.
    let q = client::derive_qos(
        &QosPreferences {
            phy: 0x04,
            max_transport_latency_ms: 5000,
            preferred_presentation_delay_min_us: 50_000,
            ..prefs
        },
        &CODEC_16_2,
        0,
        0,
    );
    assert_eq!(q.phy, 0x04);
    assert_eq!(q.max_transport_latency_ms, 4000);
    assert_eq!(q.presentation_delay_us, 40_000);
    // No PHY preference -> 2M; inverted range -> minimum.
    let q = client::derive_qos(
        &QosPreferences {
            phy: 0x00,
            presentation_delay_max_us: 0,
            preferred_presentation_delay_min_us: 0,
            ..prefs
        },
        &CODEC_16_2,
        0,
        0,
    );
    assert_eq!(q.phy, 0x02);
    assert_eq!(q.presentation_delay_us, 20_000);
    // Two blocks of 7.5 ms frames.
    let q = client::derive_qos(
        &SERVER_PREFS,
        &CodecConfig {
            frame_duration_us: 7_500,
            frames_per_sdu: 2,
            ..CODEC_16_2
        },
        0,
        0,
    );
    assert_eq!(q.sdu_interval_us, 15_000);
    assert_eq!(q.max_sdu, 80);
}

// ---------------------------------------------------------------------------
// End to end against the server state machine

struct Link {
    server: Ascs<1>,
    fx: Effects,
}

impl Link {
    /// Writes a client-built PDU to the server's Control Point. Returns the parsed
    /// response entry for ASE 1 and the notified ASE values, after handling deferred
    /// work as the integration contract requires.
    fn write(&mut self, pdu: &[u8]) -> (CpEntry, Vec<Vec<u8>>) {
        self.fx.clear();
        let mut rsp = [0u8; CP_RESPONSE_MAX_LEN];
        let n = self.server.control_point(pdu, &mut rsp, &mut self.fx);
        let parsed = client::parse_cp_response(&rsp[..n]).unwrap();
        assert_eq!(parsed.opcode, pdu[0]);
        assert!(!parsed.is_operation_error());
        assert_eq!(parsed.entries().len(), 1);
        let entry = parsed.entry(1).unwrap();
        let mut values = self.notifications();
        if self.fx.has_deferred() {
            let mut fx = Effects::new();
            self.server.complete_deferred(&mut fx);
            self.fx = fx;
            values.extend(self.notifications());
        }
        (entry, values)
    }

    /// Values of the ASEs changed in `self.fx`.
    fn notifications(&self) -> Vec<Vec<u8>> {
        self.fx
            .changed_ases()
            .map(|id| {
                let mut out = [0u8; ASE_VALUE_MAX_LEN];
                let n = self.server.ase_value(id, &mut out);
                out[..n].to_vec()
            })
            .collect()
    }
}

/// Parses ASE 1 values with the client parser.
fn parse_all(values: &[Vec<u8>]) -> Vec<AseStatus<'_>> {
    values
        .iter()
        .map(|v| {
            let v = client::parse_ase(v).unwrap();
            assert_eq!(v.ase_id, 1);
            v.status
        })
        .collect()
}

fn ok() -> CpEntry {
    CpEntry {
        ase_id: 1,
        response_code: response::SUCCESS,
        reason: reason::NONE,
    }
}

fn end_to_end(sampling_hz: u32, octets: u16) {
    const CIG: u8 = 0;
    const CIS: u8 = 1;
    const CIS_HANDLE: u16 = 0x0061;
    let mut link = Link {
        server: Ascs::new(),
        fx: Effects::new(),
    };
    let mut pdu = [0u8; CP_PDU_MAX_LEN];

    // Capabilities -> codec configuration.
    let pac_value = server_pac();
    let pac = client::parse_pac(&pac_value).unwrap();
    let codec = client::choose_codec_config(&pac, sampling_hz, octets, 1).unwrap();
    assert_eq!(codec, lc3(sampling_hz, octets, 1));

    // Config Codec.
    let n = client::config_codec(
        &mut pdu,
        1,
        target_latency::LOW_LATENCY,
        target_phy::LE_2M,
        &codec,
    )
    .unwrap();
    let (entry, values) = link.write(&pdu[..n]);
    let notified = parse_all(&values);
    assert_eq!(entry, ok());
    let [AseStatus::CodecConfigured(cc)] = notified[..] else {
        panic!("{notified:?}");
    };
    assert_eq!(cc.codec, Some(codec));
    assert_eq!(cc.codec_id, pacs::LC3_CODEC_ID);
    assert_eq!(cc.preferences, SERVER_PREFS);
    assert_eq!(link.server.state(1), Some(AseState::CodecConfigured));

    // Config QoS from the server's preferences.
    let qos = client::derive_qos(&cc.preferences, &codec, CIG, CIS);
    assert_eq!(qos.max_sdu, octets);
    assert_eq!(qos.sdu_interval_us, 10_000);
    let n = client::config_qos(&mut pdu, 1, &qos);
    let (entry, values) = link.write(&pdu[..n]);
    let notified = parse_all(&values);
    assert_eq!(entry, ok());
    assert_eq!(notified, [AseStatus::QosConfigured(qos)]);

    // Receiver Start Ready is refused before Enable.
    let n = client::receiver_start_ready(&mut pdu, 1);
    let (entry, values) = link.write(&pdu[..n]);
    let notified = parse_all(&values);
    assert_eq!(
        entry.response_code,
        response::INVALID_ASE_STATE_MACHINE_TRANSITION
    );
    assert!(notified.is_empty());

    // Enable.
    let n = client::enable(&mut pdu, 1, context::LIVE);
    let (entry, values) = link.write(&pdu[..n]);
    let notified = parse_all(&values);
    assert_eq!(entry, ok());
    let [AseStatus::Enabling(params)] = notified[..] else {
        panic!("{notified:?}");
    };
    assert_eq!((params.cig_id, params.cis_id), (CIG, CIS));
    assert_eq!(params.streaming_contexts, Some(context::LIVE));
    assert_eq!(params.metadata, [0x03, 0x02, 0x40, 0x00]);

    // CIS setup by the client; the server accepts it.
    assert!(link.server.cis_request(CIG, CIS));
    link.fx.clear();
    link.server
        .cis_established(CIG, CIS, CIS_HANDLE, &mut link.fx);
    assert_eq!(
        link.fx.actions(),
        [Action::SetupIsoDataPath {
            cis_handle: CIS_HANDLE
        }]
    );
    assert!(link.notifications().is_empty());

    // Receiver Start Ready -> Streaming.
    let n = client::receiver_start_ready(&mut pdu, 1);
    let (entry, values) = link.write(&pdu[..n]);
    let notified = parse_all(&values);
    assert_eq!(entry, ok());
    assert_eq!(notified, [AseStatus::Streaming(params)]);
    assert_eq!(
        link.fx.actions(),
        [Action::StartStream {
            ase_id: 1,
            cis_handle: CIS_HANDLE,
            codec,
            qos,
        }]
    );

    // Disable -> Disabling.
    let n = client::disable(&mut pdu, 1);
    let (entry, values) = link.write(&pdu[..n]);
    let notified = parse_all(&values);
    assert_eq!(entry, ok());
    assert_eq!(notified, [AseStatus::Disabling(params)]);
    assert_eq!(link.fx.actions(), [Action::StopStream { ase_id: 1 }]);

    // Receiver Stop Ready -> QoS Configured.
    let n = client::receiver_stop_ready(&mut pdu, 1);
    let (entry, values) = link.write(&pdu[..n]);
    let notified = parse_all(&values);
    assert_eq!(entry, ok());
    assert_eq!(notified, [AseStatus::QosConfigured(qos)]);

    // Release -> Releasing; the server disconnects the CIS, then Codec Configured
    // with the cached configuration.
    let n = client::release(&mut pdu, 1);
    let (entry, values) = link.write(&pdu[..n]);
    let notified = parse_all(&values);
    assert_eq!(entry, ok());
    assert_eq!(notified, [AseStatus::Releasing]);
    assert_eq!(
        link.fx.actions(),
        [Action::DisconnectCis {
            cis_handle: CIS_HANDLE
        }]
    );
    link.fx.clear();
    link.server.cis_disconnected(CIS_HANDLE, &mut link.fx);
    let values = link.notifications();
    let notified = parse_all(&values);
    let [AseStatus::CodecConfigured(cached)] = notified[..] else {
        panic!("{notified:?}");
    };
    assert_eq!(cached.codec, Some(codec));
    assert_eq!(link.server.state(1), Some(AseState::CodecConfigured));

    // A second Release without a CIS goes Releasing -> Codec Configured (deferred).
    let n = client::release(&mut pdu, 1);
    let (entry, values) = link.write(&pdu[..n]);
    let notified = parse_all(&values);
    assert_eq!(entry, ok());
    assert_eq!(notified.len(), 2);
    assert_eq!(notified[0], AseStatus::Releasing);
    assert_eq!(notified[1].state(), AseState::CodecConfigured);
}

#[test]
fn end_to_end_16_2() {
    end_to_end(16_000, 40);
}

#[test]
fn end_to_end_24_2() {
    end_to_end(24_000, 60);
}

#[test]
fn end_to_end_32_2() {
    end_to_end(32_000, 80);
}

#[test]
fn server_rejections_parse() {
    let mut link = Link {
        server: Ascs::new(),
        fx: Effects::new(),
    };
    let mut pdu = [0u8; CP_PDU_MAX_LEN];
    // 48 kHz is not supported by the server.
    let n = client::config_codec(&mut pdu, 1, 1, 2, &lc3(48_000, 100, 1)).unwrap();
    let (entry, values) = link.write(&pdu[..n]);
    let notified = parse_all(&values);
    assert_eq!(
        entry,
        CpEntry {
            ase_id: 1,
            response_code: response::UNSUPPORTED_CONFIGURATION_PARAMETER_VALUE,
            reason: reason::CODEC_SPECIFIC_CONFIGURATION,
        }
    );
    assert!(notified.is_empty());
    // Streaming context outside the server's available source contexts.
    let n = client::config_codec(&mut pdu, 1, 1, 2, &CODEC_16_2).unwrap();
    assert_eq!(link.write(&pdu[..n]).0, ok());
    let qos = client::derive_qos(&SERVER_PREFS, &CODEC_16_2, 0, 0);
    let n = client::config_qos(&mut pdu, 1, &qos);
    assert_eq!(link.write(&pdu[..n]).0, ok());
    let n = client::enable(&mut pdu, 1, context::RINGTONE);
    let (entry, _) = link.write(&pdu[..n]);
    assert_eq!(
        entry,
        CpEntry {
            ase_id: 1,
            response_code: response::REJECTED_METADATA,
            reason: 0x02, // Streaming_Audio_Contexts
        }
    );
}
