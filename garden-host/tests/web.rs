//! The web layer end to end: pages render, actions redirect, a live listener
//! over WebSocket starts the stream and receives the decoded PCM.

use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use axum::body::Body;
use axum::http::{Request, StatusCode};
use futures_util::StreamExt;
use garden_host::config::{Config, MicConfig, Paths};
use garden_host::engine::Engine;
use garden_host::link::{FrameReader, lock};
use garden_host::web;
use garden_link::{
    Addr, DongleToHost, HostToDongle, MAX_FRAME, MicId, MicLink, PROTOCOL_VERSION, Quality,
};
use http_body_util::BodyExt;
use tokio_tungstenite::tungstenite::Message;
use tower::ServiceExt;

const A: Addr = Addr {
    kind: 1,
    bytes: [0xDE, 0x10, 0xEB, 0xCE, 0x60, 0x9E],
};
const KEY: &str = "de10ebce609e-r";
const SLOT: MicId = MicId(0);

fn feed(engine: &Mutex<Engine>, msgs: &[DongleToHost]) {
    let mut buf = [0u8; MAX_FRAME];
    let mut r = FrameReader::default();
    let mut e = lock(engine);
    for m in msgs {
        let frame = garden_link::encode(m, &mut buf).unwrap();
        r.feed(frame, |m| e.handle(&m, Instant::now()));
    }
}

fn setup(dir: &std::path::Path) -> Arc<Mutex<Engine>> {
    let cfg = Config {
        mics: vec![MicConfig {
            addr: A,
            name: "Hedge".into(),
            quality: Quality::Low,
            stream: false,
            record: false,
        }],
    };
    let engine = Arc::new(Mutex::new(Engine::new(
        cfg,
        Paths::in_dir(dir),
        Box::new(Engine::local_now),
    )));
    lock(&engine).link_up("mock", Instant::now());
    feed(
        &engine,
        &[
            DongleToHost::Hello {
                protocol: PROTOCOL_VERSION,
                firmware: "test",
                max_mics: 2,
            },
            DongleToHost::MicStatus {
                mic: SLOT,
                addr: A,
                link: MicLink::Ready,
            },
        ],
    );
    lock(&engine).take_outbox();
    engine
}

async fn get(app: &axum::Router, uri: &str) -> (StatusCode, String) {
    let res = app
        .clone()
        .oneshot(Request::get(uri).body(Body::empty()).unwrap())
        .await
        .unwrap();
    let status = res.status();
    let body = res.into_body().collect().await.unwrap().to_bytes();
    (status, String::from_utf8_lossy(&body).into_owned())
}

#[tokio::test]
async fn pages_render_and_actions_redirect() {
    let dir = tempfile::tempdir().unwrap();
    let engine = setup(dir.path());
    let app = web::router(engine.clone());

    let (s, body) = get(&app, "/").await;
    assert_eq!(s, StatusCode::OK);
    assert!(body.contains(&format!("id=\"mic-{KEY}\"")));
    assert!(body.contains("Receiver connected on mock"));
    assert!(body.contains(">Ready<"));
    let (s, body) = get(&app, "/add").await;
    assert_eq!(s, StatusCode::OK);
    assert!(body.contains("id=\"adverts\""));
    assert!(lock(&engine).scanning(), "the Add page turns scanning on");
    assert_eq!(
        lock(&engine).take_outbox(),
        [HostToDongle::Scan { enable: true }]
    );
    assert_eq!(get(&app, "/recordings").await.0, StatusCode::OK);
    assert_eq!(get(&app, "/static/app.js").await.0, StatusCode::OK);
    assert_eq!(get(&app, "/nope").await.0, StatusCode::NOT_FOUND);

    let post = |uri: &str, form: &str| {
        Request::post(uri)
            .header("content-type", "application/x-www-form-urlencoded")
            .body(Body::from(form.to_string()))
            .unwrap()
    };
    let res = app
        .clone()
        .oneshot(post(
            &format!("/mics/{KEY}/settings"),
            "name=Old+hedge&quality=High",
        ))
        .await
        .unwrap();
    assert_eq!(res.status(), StatusCode::SEE_OTHER);
    let res = app
        .clone()
        .oneshot(post(&format!("/mics/{KEY}/stream"), "on=true"))
        .await
        .unwrap();
    assert_eq!(res.status(), StatusCode::SEE_OTHER);
    assert_eq!(
        lock(&engine).take_outbox(),
        [HostToDongle::StartStream {
            mic: SLOT,
            quality: Quality::High
        }]
    );
    let res = app
        .clone()
        .oneshot(post("/mics/0000-x/stream", "on=true"))
        .await
        .unwrap();
    assert_eq!(res.status(), StatusCode::NOT_FOUND);
    let res = app
        .clone()
        .oneshot(post("/mics", "addr=11:22:33:44:55:66/public&name=Pond"))
        .await
        .unwrap();
    assert_eq!(res.status(), StatusCode::SEE_OTHER);
    let e = lock(&engine);
    assert_eq!(
        e.mics()
            .iter()
            .map(|m| m.cfg.name.as_str())
            .collect::<Vec<_>>(),
        ["Old hedge", "Pond"]
    );
}

#[tokio::test]
async fn live_listener_gets_pcm_over_websocket() {
    let dir = tempfile::tempdir().unwrap();
    let engine = setup(dir.path());
    let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
    let port = listener.local_addr().unwrap().port();
    tokio::spawn(axum::serve(listener, web::router(engine.clone())).into_future());

    let (mut ws, _) =
        tokio_tungstenite::connect_async(format!("ws://127.0.0.1:{port}/mics/{KEY}/listen"))
            .await
            .unwrap();
    // Listening is a reason to stream.
    let deadline = Instant::now() + Duration::from_secs(5);
    while lock(&engine).mic(&A).unwrap().listeners == 0 {
        assert!(Instant::now() < deadline);
        tokio::time::sleep(Duration::from_millis(10)).await;
    }
    assert_eq!(
        lock(&engine).take_outbox(),
        [HostToDongle::StartStream {
            mic: SLOT,
            quality: Quality::Low
        }]
    );

    let mut enc = lc3::Encoder::new(10_000, 16_000, 16_000).unwrap();
    let pcm: Vec<i16> = (0..160).map(|i| ((i % 32) * 500) as i16).collect();
    let mut frame = [0u8; 40];
    enc.encode(&pcm, &mut frame).unwrap();
    feed(
        &engine,
        &[
            DongleToHost::MicStatus {
                mic: SLOT,
                addr: A,
                link: MicLink::Starting,
            },
            DongleToHost::StreamConfig {
                mic: SLOT,
                sampling_hz: 16_000,
                frame_duration_us: 10_000,
                octets_per_frame: 40,
            },
            DongleToHost::MicStatus {
                mic: SLOT,
                addr: A,
                link: MicLink::Streaming,
            },
            DongleToHost::Audio {
                mic: SLOT,
                seq: 1,
                frame: &frame,
            },
            DongleToHost::Audio {
                mic: SLOT,
                seq: 3,
                frame: &frame,
            },
        ],
    );

    let next = async |ws: &mut tokio_tungstenite::WebSocketStream<_>| {
        tokio::time::timeout(Duration::from_secs(5), ws.next())
            .await
            .unwrap()
            .unwrap()
            .unwrap()
    };
    assert_eq!(
        next(&mut ws).await,
        Message::Text("{\"rate\":16000}".into())
    );
    // seq 1, then 2 concealed, then 3: three 10 ms chunks of 160 samples.
    for _ in 0..3 {
        match next(&mut ws).await {
            Message::Binary(b) => assert_eq!(b.len(), 160 * 2),
            other => panic!("expected PCM, got {other:?}"),
        }
    }

    drop(ws);
    let deadline = Instant::now() + Duration::from_secs(5);
    while lock(&engine).mic(&A).unwrap().listeners != 0 {
        assert!(Instant::now() < deadline);
        tokio::time::sleep(Duration::from_millis(10)).await;
    }
    // Nobody listens any more: the stream is stopped.
    assert_eq!(
        lock(&engine).take_outbox(),
        [HostToDongle::StopStream { mic: SLOT }]
    );
}
