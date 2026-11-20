//! The host's protocol handling, driven by byte streams encoded with
//! garden-link exactly as the dongle sends them.

use std::io::{self, Read, Write};
use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use chrono::{NaiveDate, NaiveDateTime};
use garden_host::config::{Config, MicConfig, Paths};
use garden_host::engine::{Change, Engine};
use garden_host::link::{FrameReader, serve};
use garden_link::postcard::accumulator::{CobsAccumulator, FeedResult};
use garden_link::{
    Addr, DongleToHost, Failure, HostToDongle, MAX_FRAME, MicId, MicLink, PROTOCOL_VERSION, Quality,
};

const A: Addr = Addr {
    kind: 1,
    bytes: [0xDE, 0x10, 0xEB, 0xCE, 0x60, 0x9E],
};
const B: Addr = Addr {
    kind: 0,
    bytes: [0x11, 0x22, 0x33, 0x44, 0x55, 0x66],
};
const SLOT: MicId = MicId(2);

fn start_time() -> NaiveDateTime {
    NaiveDate::from_ymd_opt(2026, 6, 21)
        .unwrap()
        .and_hms_opt(5, 30, 0)
        .unwrap()
}

fn engine(dir: &Path, mics: Vec<MicConfig>) -> Engine {
    let clock = Arc::new(Mutex::new(start_time()));
    Engine::new(
        Config { mics },
        Paths::in_dir(dir),
        Box::new(move || *clock.lock().unwrap()),
    )
}

fn mic(addr: Addr, name: &str, quality: Quality, record: bool) -> MicConfig {
    MicConfig {
        addr,
        name: name.into(),
        quality,
        stream: false,
        record,
    }
}

/// Encodes messages the way the dongle puts them on the wire.
fn wire(msgs: &[DongleToHost]) -> Vec<u8> {
    let mut out = Vec::new();
    let mut buf = [0u8; MAX_FRAME];
    for m in msgs {
        out.extend_from_slice(garden_link::encode(m, &mut buf).unwrap());
    }
    out
}

/// Feeds `bytes` to the engine in awkward 7-byte pieces.
fn feed(e: &mut Engine, bytes: &[u8], now: Instant) {
    let mut r = FrameReader::default();
    for chunk in bytes.chunks(7) {
        r.feed(chunk, |m| e.handle(&m, now));
    }
    assert_eq!(r.bad_frames, 0);
}

fn hello() -> DongleToHost<'static> {
    DongleToHost::Hello {
        protocol: PROTOCOL_VERSION,
        firmware: "0.1.0-test",
        max_mics: 4,
    }
}

fn status(addr: Addr, link: MicLink) -> DongleToHost<'static> {
    DongleToHost::MicStatus {
        mic: SLOT,
        addr,
        link,
    }
}

/// `n` LC3 frames of a 440 Hz tone at `hz`, and the PCM they were made from.
fn lc3_tone(hz: u32, octets: usize, n: usize) -> (Vec<Vec<u8>>, Vec<i16>) {
    let mut enc = lc3::Encoder::new(10_000, hz, hz).unwrap();
    let ns = enc.frame_samples();
    let pcm: Vec<i16> = (0..ns * n)
        .map(|i| {
            (10_000.0 * (2.0 * std::f64::consts::PI * 440.0 * i as f64 / f64::from(hz)).sin())
                as i16
        })
        .collect();
    let frames = pcm
        .chunks(ns)
        .map(|c| {
            let mut f = vec![0u8; octets];
            enc.encode(c, &mut f).unwrap();
            f
        })
        .collect();
    (frames, pcm)
}

fn wav_files(dir: &Path) -> Vec<PathBuf> {
    let mut out = Vec::new();
    let mut stack = vec![dir.to_path_buf()];
    while let Some(d) = stack.pop() {
        for e in std::fs::read_dir(&d).unwrap().flatten() {
            let p = e.path();
            if p.is_dir() {
                stack.push(p);
            } else if p.extension().is_some_and(|x| x == "wav") {
                out.push(p);
            }
        }
    }
    out.sort();
    out
}

fn rms(x: &[i16]) -> f64 {
    (x.iter().map(|&s| f64::from(s) * f64::from(s)).sum::<f64>() / x.len() as f64).sqrt()
}

#[test]
fn mic_lifecycle_audio_with_gaps_and_recording() {
    let dir = tempfile::tempdir().unwrap();
    let mut e = engine(dir.path(), vec![mic(A, "Apple tree", Quality::Low, true)]);
    let mut changes = e.subscribe();
    let t0 = Instant::now();

    e.link_up("mock", t0);
    assert_eq!(e.take_outbox(), [HostToDongle::Hello]);
    feed(&mut e, &wire(&[hello()]), t0);
    assert_eq!(e.receiver.info.as_ref().unwrap().firmware, "0.1.0-test");

    // The dongle gets a moment to report mics it already holds.
    e.tick(t0 + Duration::from_millis(500));
    assert_eq!(e.take_outbox(), []);
    e.tick(t0 + Duration::from_millis(1100));
    assert_eq!(e.take_outbox(), [HostToDongle::Connect { addr: A }]);
    e.tick(t0 + Duration::from_secs(5));
    assert_eq!(
        e.take_outbox(),
        [],
        "no second Connect while the first is pending"
    );

    let t1 = t0 + Duration::from_secs(6);
    feed(
        &mut e,
        &wire(&[
            status(A, MicLink::Connecting),
            status(A, MicLink::Discovering),
            status(A, MicLink::Ready),
            DongleToHost::Rssi {
                mic: SLOT,
                rssi: -61,
            },
        ]),
        t1,
    );
    let m = e.mic(&A).unwrap();
    assert_eq!(
        (m.slot, m.link, m.rssi),
        (Some(SLOT), Some(MicLink::Ready), Some(-61))
    );
    // Recording is wanted, so the host asks for the stream at the mic's quality.
    assert_eq!(
        e.take_outbox(),
        [HostToDongle::StartStream {
            mic: SLOT,
            quality: Quality::Low
        }]
    );

    // 12 frames: seq 100..=111, with 105 never arriving and 109 reported lost.
    let (frames, _) = lc3_tone(16_000, 40, 12);
    let mut msgs = vec![
        status(A, MicLink::Starting),
        DongleToHost::StreamConfig {
            mic: SLOT,
            sampling_hz: 16_000,
            frame_duration_us: 10_000,
            octets_per_frame: 40,
        },
        status(A, MicLink::Streaming),
    ];
    for (i, f) in frames.iter().enumerate() {
        let seq = 100 + i as u16;
        match seq {
            105 => {}
            109 => msgs.push(DongleToHost::Audio {
                mic: SLOT,
                seq,
                frame: &[],
            }),
            _ => msgs.push(DongleToHost::Audio {
                mic: SLOT,
                seq,
                frame: f,
            }),
        }
    }
    feed(&mut e, &wire(&msgs), t1);
    assert_eq!(e.take_outbox(), []);
    let m = e.mic(&A).unwrap();
    assert_eq!(m.link, Some(MicLink::Streaming));
    assert_eq!(m.stream_rate(), Some(16_000));
    assert_eq!(
        (
            m.stats.frames_decoded,
            m.stats.frames_concealed,
            m.stats.frames_dropped
        ),
        (10, 2, 0)
    );
    assert_eq!(m.stats.samples, 12 * 160);
    let recording = m.recording_path().unwrap().to_path_buf();
    assert_eq!(
        recording,
        dir.path()
            .join("recordings/Apple tree/2026-06-21/05-30-00.wav")
    );
    assert!(m.level.is_some(), "120 ms of audio gives a meter reading");

    // The link drops: the file is completed, a reconnect is scheduled.
    let t2 = t1 + Duration::from_secs(1);
    feed(
        &mut e,
        &wire(&[status(A, MicLink::Gone(Failure::Disconnected(0x08)))]),
        t2,
    );
    let m = e.mic(&A).unwrap();
    assert_eq!(
        (m.slot, m.link, m.last_failure),
        (None, None, Some(Failure::Disconnected(0x08)))
    );
    assert!(m.recording_path().is_none());
    let wav = hound::WavReader::open(&recording).unwrap();
    assert_eq!(
        wav.spec(),
        hound::WavSpec {
            channels: 1,
            sample_rate: 16_000,
            bits_per_sample: 16,
            sample_format: hound::SampleFormat::Int
        }
    );
    assert_eq!(wav.len(), 12 * 160);

    e.tick(t2 + Duration::from_millis(1900));
    assert_eq!(e.take_outbox(), [], "backoff");
    e.tick(t2 + Duration::from_millis(2100));
    assert_eq!(e.take_outbox(), [HostToDongle::Connect { addr: A }]);
    // A second failure doubles the wait.
    let t3 = t2 + Duration::from_secs(3);
    feed(
        &mut e,
        &wire(&[
            status(A, MicLink::Connecting),
            status(A, MicLink::Gone(Failure::ConnectTimeout)),
        ]),
        t3,
    );
    e.tick(t3 + Duration::from_millis(3900));
    assert_eq!(e.take_outbox(), []);
    e.tick(t3 + Duration::from_millis(4100));
    assert_eq!(e.take_outbox(), [HostToDongle::Connect { addr: A }]);

    let mut seen = Vec::new();
    while let Ok(c) = changes.try_recv() {
        seen.push(c);
    }
    assert!(seen.contains(&Change::Receiver));
    assert!(seen.contains(&Change::Rssi(A, -61)));
    assert!(
        seen.iter()
            .any(|c| matches!(c, Change::Meter(a, _) if *a == A))
    );
}

/// An in-memory serial port: hands out the input in small pieces, then
/// reports the port closed; collects what the host writes.
struct MockPort {
    input: io::Cursor<Vec<u8>>,
    output: Vec<u8>,
}

impl Read for MockPort {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        let n = buf.len().min(13);
        self.input.read(&mut buf[..n])
    }
}

impl Write for MockPort {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        self.output.extend_from_slice(buf);
        Ok(buf.len())
    }
    fn flush(&mut self) -> io::Result<()> {
        Ok(())
    }
}

fn host_messages(bytes: &[u8]) -> Vec<HostToDongle> {
    let mut acc: CobsAccumulator<MAX_FRAME> = CobsAccumulator::new();
    let mut out = Vec::new();
    let mut window = bytes;
    while !window.is_empty() {
        window = match acc.feed::<HostToDongle>(window) {
            FeedResult::Consumed => break,
            FeedResult::Success { data, remaining } => {
                out.push(data);
                remaining
            }
            _ => panic!("host sent a bad frame"),
        };
    }
    out
}

#[test]
fn serial_session_on_an_in_memory_port() {
    let dir = tempfile::tempdir().unwrap();
    let e = Mutex::new(engine(
        dir.path(),
        vec![mic(A, "Hedge", Quality::Medium, true)],
    ));
    // The host restarted while the dongle kept the mic connected: it reports
    // the slot right after Hello, and the host starts the stream without a
    // Connect.
    let (frames, _) = lc3_tone(24_000, 60, 23);
    let mut msgs = vec![
        hello(),
        status(A, MicLink::Ready),
        DongleToHost::Log {
            text: "hello from the dongle",
        },
        status(A, MicLink::Starting),
        DongleToHost::StreamConfig {
            mic: SLOT,
            sampling_hz: 24_000,
            frame_duration_us: 10_000,
            octets_per_frame: 60,
        },
        status(A, MicLink::Streaming),
    ];
    // Sequence numbers wrap, and 0x0001..=0x0003 are missing.
    for (i, f) in frames.iter().enumerate() {
        let seq = 0xFFF0u16.wrapping_add(i as u16);
        if !(1..=3).contains(&seq) {
            msgs.push(DongleToHost::Audio {
                mic: SLOT,
                seq,
                frame: f,
            });
        }
    }
    let mut bytes = wire(&msgs);
    // Line noise before the first frame (e.g. a frame cut by the port opening).
    bytes.splice(0..0, [0x13, 0x37, 0x00]);
    let mut port = MockPort {
        input: io::Cursor::new(bytes),
        output: Vec::new(),
    };

    let err = serve(&e, &mut port, "mock");
    assert_eq!(err.kind(), io::ErrorKind::UnexpectedEof);
    assert_eq!(
        host_messages(&port.output),
        [
            HostToDongle::Hello,
            HostToDongle::StartStream {
                mic: SLOT,
                quality: Quality::Medium
            }
        ]
    );

    let mut eng = e.lock().unwrap();
    assert_eq!(eng.receiver.port.as_deref(), Some("mock"));
    assert_eq!(eng.receiver.bad_frames, 1);
    let m = eng.mic(&A).unwrap();
    assert_eq!((m.stats.frames_decoded, m.stats.frames_concealed), (20, 3));
    assert_eq!(m.stats.samples, 23 * 240);
    let path = m.recording_path().unwrap().to_path_buf();

    // Unplugged: everything goes offline and the recording is completed.
    eng.link_down(Some("unplugged".into()));
    let m = eng.mic(&A).unwrap();
    assert_eq!((m.slot, m.link), (None, None));
    assert!(m.recording_path().is_none());
    assert_eq!(
        eng.take_outbox(),
        [],
        "nothing is queued while disconnected"
    );
    let wav = hound::WavReader::open(&path).unwrap();
    assert_eq!((wav.spec().sample_rate, wav.len()), (24_000, 23 * 240));
}

#[test]
fn lc3_round_trip_produces_a_correct_wav() {
    let dir = tempfile::tempdir().unwrap();
    let mut e = engine(dir.path(), vec![mic(A, "Pond", Quality::Medium, true)]);
    let t = Instant::now();
    e.link_up("mock", t);
    feed(&mut e, &wire(&[hello(), status(A, MicLink::Starting)]), t);
    // One second of 24 kHz audio at 48 kbit/s (BAP 24_2).
    let (frames, pcm) = lc3_tone(24_000, 60, 100);
    let mut msgs = vec![DongleToHost::StreamConfig {
        mic: SLOT,
        sampling_hz: 24_000,
        frame_duration_us: 10_000,
        octets_per_frame: 60,
    }];
    msgs.extend(frames.iter().enumerate().map(|(i, f)| DongleToHost::Audio {
        mic: SLOT,
        seq: i as u16,
        frame: f,
    }));
    msgs.push(status(A, MicLink::Streaming));
    msgs.push(status(A, MicLink::Stopping));
    feed(&mut e, &wire(&msgs), t);

    let files = wav_files(&dir.path().join("recordings"));
    assert_eq!(files.len(), 1);
    let raw = std::fs::read(&files[0]).unwrap();
    let data_len = 100 * 240 * 2;
    assert_eq!(raw.len(), 44 + data_len);
    let u16_at = |o: usize| u16::from_le_bytes([raw[o], raw[o + 1]]);
    let u32_at = |o: usize| u32::from_le_bytes(raw[o..o + 4].try_into().unwrap());
    assert_eq!(&raw[0..4], b"RIFF");
    assert_eq!(u32_at(4) as usize, 36 + data_len);
    assert_eq!(&raw[8..16], b"WAVEfmt ");
    assert_eq!(u32_at(16), 16, "plain PCM fmt chunk");
    assert_eq!(u16_at(20), 1, "PCM");
    assert_eq!(u16_at(22), 1, "mono");
    assert_eq!(u32_at(24), 24_000);
    assert_eq!(u32_at(28), 48_000, "byte rate");
    assert_eq!(u16_at(32), 2, "block align");
    assert_eq!(u16_at(34), 16, "bits per sample");
    assert_eq!(&raw[36..40], b"data");
    assert_eq!(u32_at(40) as usize, data_len);

    // The decoded tone has the input's level (LC3 adds delay, not loss).
    let decoded: Vec<i16> = raw[44..]
        .chunks(2)
        .map(|b| i16::from_le_bytes([b[0], b[1]]))
        .collect();
    let (a, b) = (rms(&pcm[2400..]), rms(&decoded[2400..]));
    assert!(
        (20.0 * (b / a).log10()).abs() < 1.0,
        "input rms {a}, decoded rms {b}"
    );
}

#[test]
fn stream_restart_starts_a_new_file() {
    let dir = tempfile::tempdir().unwrap();
    let mut e = engine(dir.path(), vec![mic(A, "Hedge", Quality::Low, true)]);
    let t = Instant::now();
    e.link_up("mock", t);
    let (frames, _) = lc3_tone(16_000, 40, 3);
    let config = DongleToHost::StreamConfig {
        mic: SLOT,
        sampling_hz: 16_000,
        frame_duration_us: 10_000,
        octets_per_frame: 40,
    };
    let audio = |seq: u16| DongleToHost::Audio {
        mic: SLOT,
        seq,
        frame: &frames[seq as usize % 3],
    };
    feed(
        &mut e,
        &wire(&[
            hello(),
            status(A, MicLink::Starting),
            config.clone(),
            audio(0),
            audio(1),
        ]),
        t,
    );
    feed(&mut e, &wire(&[config, audio(0)]), t);
    // Same second: the second file gets a suffix.
    let day = dir.path().join("recordings/Hedge/2026-06-21");
    let first = day.join("05-30-00.wav");
    let second = day.join("05-30-00-1.wav");
    assert_eq!(
        wav_files(&dir.path().join("recordings")),
        [second.clone(), first.clone()]
    );
    assert_eq!(hound::WavReader::open(&first).unwrap().len(), 320);
    assert_eq!(e.mic(&A).unwrap().recording_path(), Some(second.as_path()));
    e.link_down(None);
    assert_eq!(hound::WavReader::open(&second).unwrap().len(), 160);
}

#[test]
fn user_demand_drives_streams() {
    let dir = tempfile::tempdir().unwrap();
    let mut e = engine(dir.path(), vec![mic(A, "Hedge", Quality::Low, false)]);
    let t = Instant::now();
    e.link_up("mock", t);
    e.take_outbox();
    feed(&mut e, &wire(&[hello(), status(A, MicLink::Ready)]), t);
    assert_eq!(e.take_outbox(), [], "nobody wants audio");

    // A listener joins: stream starts; audio reaches the listener.
    let (mut rx, rate) = e.add_listener(&A, t).unwrap();
    assert_eq!(rate, None);
    assert_eq!(
        e.take_outbox(),
        [HostToDongle::StartStream {
            mic: SLOT,
            quality: Quality::Low
        }]
    );
    let (frames, _) = lc3_tone(16_000, 40, 2);
    feed(
        &mut e,
        &wire(&[
            status(A, MicLink::Starting),
            DongleToHost::StreamConfig {
                mic: SLOT,
                sampling_hz: 16_000,
                frame_duration_us: 10_000,
                octets_per_frame: 40,
            },
            status(A, MicLink::Streaming),
            DongleToHost::Audio {
                mic: SLOT,
                seq: 7,
                frame: &frames[0],
            },
        ]),
        t,
    );
    let pcm = rx.try_recv().unwrap();
    assert_eq!((pcm.rate, pcm.samples.len()), (16_000, 160));

    // Quality change while streaming: stop, then start again at the new one.
    assert!(e.configure_mic(&A, "Hedge", Quality::High, t));
    assert_eq!(e.take_outbox(), [HostToDongle::StopStream { mic: SLOT }]);
    feed(
        &mut e,
        &wire(&[status(A, MicLink::Stopping), status(A, MicLink::Ready)]),
        t,
    );
    assert_eq!(
        e.take_outbox(),
        [HostToDongle::StartStream {
            mic: SLOT,
            quality: Quality::High
        }]
    );
    feed(
        &mut e,
        &wire(&[
            status(A, MicLink::Starting),
            DongleToHost::StreamConfig {
                mic: SLOT,
                sampling_hz: 32_000,
                frame_duration_us: 10_000,
                octets_per_frame: 80,
            },
            status(A, MicLink::Streaming),
        ]),
        t,
    );
    assert_eq!(e.take_outbox(), []);

    // The listener leaves: nobody wants the stream any more.
    e.remove_listener(&A, t);
    assert_eq!(e.take_outbox(), [HostToDongle::StopStream { mic: SLOT }]);
    // An unanswered request is repeated after a while, not on every tick.
    e.tick(t + Duration::from_secs(1));
    assert_eq!(e.take_outbox(), []);
    e.tick(t + Duration::from_secs(11));
    assert_eq!(e.take_outbox(), [HostToDongle::StopStream { mic: SLOT }]);

    // Start stream pins it; Stop stream releases it and recording with it.
    feed(&mut e, &wire(&[status(A, MicLink::Ready)]), t);
    assert!(e.set_recording(&A, true, t));
    assert_eq!(
        e.take_outbox(),
        [HostToDongle::StartStream {
            mic: SLOT,
            quality: Quality::High
        }]
    );
    assert!(e.set_streaming(&A, false, t));
    assert!(!e.mic(&A).unwrap().cfg.record);

    // Settings are persisted.
    let saved = Config::load(&dir.path().join("config.json")).unwrap();
    assert_eq!(saved.mics[0].quality, Quality::High);
    assert!(!saved.mics[0].record);
}

#[test]
fn scanning_adding_and_forgetting() {
    let dir = tempfile::tempdir().unwrap();
    let mut e = engine(dir.path(), vec![]);
    let t = Instant::now();
    e.link_up("mock", t);
    feed(&mut e, &wire(&[hello()]), t);
    e.take_outbox();

    // Adverts are ignored unless someone is looking.
    feed(
        &mut e,
        &wire(&[DongleToHost::Advert {
            addr: A,
            rssi: -70,
            name: "Garden mic",
        }]),
        t,
    );
    assert!(e.adverts().is_empty());
    e.scan_watch(true, t);
    assert_eq!(e.take_outbox(), [HostToDongle::Scan { enable: true }]);
    feed(
        &mut e,
        &wire(&[
            DongleToHost::Advert {
                addr: A,
                rssi: -70,
                name: "Garden mic",
            },
            DongleToHost::Advert {
                addr: B,
                rssi: -50,
                name: "Garden mic",
            },
        ]),
        t,
    );
    let ads = e.adverts();
    assert_eq!(
        ads.iter().map(|(a, ad)| (*a, ad.rssi)).collect::<Vec<_>>(),
        [(B, -50), (A, -70)]
    );

    // Adding connects at once (the Hello grace has passed).
    let t1 = t + Duration::from_secs(2);
    assert!(e.add_mic(A, "Garden mic", t1));
    assert!(e.add_mic(B, "Garden mic", t1));
    assert!(!e.add_mic(A, "again", t1));
    assert_eq!(
        e.take_outbox(),
        [
            HostToDongle::Connect { addr: A },
            HostToDongle::Connect { addr: B }
        ]
    );
    let names: Vec<_> = e.mics().iter().map(|m| m.cfg.name.clone()).collect();
    assert_eq!(names, ["Garden mic", "Garden mic 2"]);

    e.scan_watch(false, t1);
    assert_eq!(e.take_outbox(), [HostToDongle::Scan { enable: false }]);
    assert!(e.adverts().is_empty());

    assert!(e.forget_mic(&A));
    assert_eq!(e.take_outbox(), []);
    let saved = Config::load(&dir.path().join("config.json")).unwrap();
    assert_eq!(saved.mics.len(), 1);
    assert_eq!(saved.mics[0].addr, B);
}

#[test]
fn forgetting_a_connected_mic_disconnects_it() {
    let dir = tempfile::tempdir().unwrap();
    let mut e = engine(dir.path(), vec![mic(A, "Hedge", Quality::Low, false)]);
    let t = Instant::now();
    e.link_up("mock", t);
    e.take_outbox();
    feed(&mut e, &wire(&[hello(), status(A, MicLink::Ready)]), t);
    e.take_outbox();
    assert_eq!(e.mic(&A).unwrap().slot, Some(SLOT));

    assert!(e.forget_mic(&A));
    assert_eq!(e.take_outbox(), [HostToDongle::Disconnect { mic: SLOT }]);
    assert!(e.mic(&A).is_none());
    let saved = Config::load(&dir.path().join("config.json")).unwrap();
    assert!(saved.mics.is_empty());
}

#[test]
fn unknown_mics_are_released_and_old_protocols_refused() {
    let dir = tempfile::tempdir().unwrap();
    let mut e = engine(dir.path(), vec![mic(A, "Hedge", Quality::Low, false)]);
    let t = Instant::now();
    e.link_up("mock", t);
    e.take_outbox();
    feed(
        &mut e,
        &wire(&[
            hello(),
            DongleToHost::MicStatus {
                mic: MicId(0),
                addr: B,
                link: MicLink::Ready,
            },
        ]),
        t,
    );
    assert_eq!(
        e.take_outbox(),
        [HostToDongle::Disconnect { mic: MicId(0) }]
    );

    let mut e = engine(dir.path(), vec![mic(A, "Hedge", Quality::Low, false)]);
    e.link_up("mock", t);
    e.take_outbox();
    feed(
        &mut e,
        &wire(&[DongleToHost::Hello {
            protocol: PROTOCOL_VERSION + 1,
            firmware: "9.9",
            max_mics: 4,
        }]),
        t,
    );
    e.tick(t + Duration::from_secs(5));
    assert_eq!(e.take_outbox(), [], "no commands to an incompatible dongle");
    assert!(e.receiver.problem.as_deref().unwrap().contains("protocol"));
    assert!(!e.receiver.usable());
}

#[test]
fn a_failed_connect_does_not_disturb_other_slots() {
    let dir = tempfile::tempdir().unwrap();
    let mut e = engine(
        dir.path(),
        vec![
            mic(A, "Hedge", Quality::Low, false),
            mic(B, "Pond", Quality::Low, false),
        ],
    );
    let t = Instant::now();
    e.link_up("mock", t);
    feed(&mut e, &wire(&[hello(), status(A, MicLink::Ready)]), t);
    // The dongle reports B's failure under A's slot id.
    feed(
        &mut e,
        &wire(&[DongleToHost::MicStatus {
            mic: SLOT,
            addr: B,
            link: MicLink::Gone(Failure::NoSlot),
        }]),
        t,
    );
    let a = e.mic(&A).unwrap();
    assert_eq!((a.slot, a.link), (Some(SLOT), Some(MicLink::Ready)));
    let b = e.mic(&B).unwrap();
    assert_eq!((b.slot, b.last_failure), (None, Some(Failure::NoSlot)));
    assert!(b.next_connect().is_some());
}
