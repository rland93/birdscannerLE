use bap::{adv, pacs, uuid};

#[test]
fn uuids() {
    assert_eq!(uuid::PACS, 0x1850);
    assert_eq!(uuid::ASCS, 0x184E);
    assert_eq!(uuid::CAS, 0x1853);
    assert_eq!(uuid::SINK_ASE, 0x2BC4);
    assert_eq!(uuid::SOURCE_ASE, 0x2BC5);
    assert_eq!(uuid::ASE_CONTROL_POINT, 0x2BC6);
    assert_eq!(uuid::SINK_PAC, 0x2BC9);
    assert_eq!(uuid::SINK_AUDIO_LOCATIONS, 0x2BCA);
    assert_eq!(uuid::SOURCE_PAC, 0x2BCB);
    assert_eq!(uuid::SOURCE_AUDIO_LOCATIONS, 0x2BCC);
    assert_eq!(uuid::AVAILABLE_AUDIO_CONTEXTS, 0x2BCD);
    assert_eq!(uuid::SUPPORTED_AUDIO_CONTEXTS, 0x2BCE);
}

#[test]
fn source_pac_bytes() {
    let mut out = [0xEEu8; 64];
    let n = pacs::source_pac(&mut out);
    let expected: [u8; 27] = [
        0x01, // one PAC record
        0x06, 0x00, 0x00, 0x00, 0x00, // LC3
        19,   // capabilities length
        3, 0x01, 0x34, 0x00, // 16|24|32 kHz = bits 2,4,5
        2, 0x02, 0x02, // 10 ms
        2, 0x03, 0x01, // 1 channel
        5, 0x04, 26, 0, 80, 0, // 26..=80 octets
        2, 0x05, 1, // 1 frame per SDU
        0, // no metadata
    ];
    assert_eq!(n, expected.len());
    assert_eq!(n, pacs::SOURCE_PAC_MAX_LEN);
    assert_eq!(&out[..n], &expected);
    assert_eq!(out[n], 0xEE);
}

#[test]
fn locations_and_contexts() {
    assert_eq!(pacs::source_audio_locations(), [0x01, 0x00, 0x00, 0x00]);
    // sink 0x0000, source 0x006F = Unspecified|Conversational|Media|Game|VoiceAssistants|Live
    assert_eq!(pacs::available_audio_contexts(), [0x00, 0x00, 0x6F, 0x00]);
    assert_eq!(pacs::supported_audio_contexts(), [0x00, 0x00, 0x6F, 0x00]);
}

#[test]
fn service_data() {
    assert_eq!(
        adv::ascs_service_data(),
        [0x4E, 0x18, 0x00, 0x00, 0x00, 0x6F, 0x00, 0x00]
    );
    assert_eq!(adv::cas_service_data(), [0x53, 0x18, 0x00]);
}
