//! The web UI: server-rendered Askama pages styled with Alloy CSS, form posts
//! for actions, Server-Sent Events for live updates (re-rendered fragments,
//! meter and RSSI values) and a WebSocket per live listener carrying PCM.

use std::collections::{HashSet, VecDeque};
use std::convert::Infallible;
use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex};
use std::time::Instant;

use askama::Template;
use axum::Router;
use axum::extract::ws::{Message, WebSocket, WebSocketUpgrade};
use axum::extract::{Form, Path as UrlPath, Query, State};
use axum::http::{StatusCode, header};
use axum::response::sse::{Event, KeepAlive, Sse};
use axum::response::{Html, IntoResponse, Redirect, Response};
use axum::routing::{get, post};
use futures_util::Stream;
use garden_link::{Addr, Failure, MicLink, PROTOCOL_VERSION, Quality, USB_PID, USB_VID};
use serde::Deserialize;
use tokio::sync::broadcast::error::RecvError;
use tower_http::services::ServeDir;

use crate::config;
use crate::engine::{Change, Engine, Mic, Receiver};
use crate::link::lock;

const APP_JS: &str = include_str!("../static/app.js");

#[derive(Clone)]
pub struct AppState {
    engine: Arc<Mutex<Engine>>,
}

pub fn router(engine: Arc<Mutex<Engine>>) -> Router {
    let recordings = lock(&engine).paths().recordings.clone();
    Router::new()
        .route("/", get(dashboard))
        .route("/add", get(add_page))
        .route("/recordings", get(recordings_page))
        .route("/events", get(events))
        .route("/static/app.js", get(app_js))
        .route("/mics", post(add_mic))
        .route("/mics/{key}/stream", post(set_stream))
        .route("/mics/{key}/record", post(set_record))
        .route("/mics/{key}/settings", post(configure))
        .route("/mics/{key}/forget", post(forget))
        .route("/mics/{key}/listen", get(listen))
        .nest_service("/recordings/files", ServeDir::new(recordings))
        .fallback(not_found)
        .with_state(AppState { engine })
}

// ----- views -----

pub struct ReceiverView {
    pub connected: bool,
    pub answered: bool,
    pub compatible: bool,
    pub port: String,
    pub firmware: String,
    pub protocol: u16,
    pub max_mics: u8,
    pub problem: String,
    pub usb_id: String,
}

impl ReceiverView {
    fn new(r: &Receiver) -> ReceiverView {
        let info = r.info.as_ref();
        ReceiverView {
            connected: r.port.is_some(),
            answered: info.is_some(),
            compatible: info.is_some_and(|i| i.protocol == PROTOCOL_VERSION),
            port: r.port.clone().unwrap_or_default(),
            firmware: info.map(|i| i.firmware.clone()).unwrap_or_default(),
            protocol: info.map_or(0, |i| i.protocol),
            max_mics: info.map_or(0, |i| i.max_mics),
            problem: r.problem.clone().unwrap_or_default(),
            usb_id: format!("{USB_VID:04x}:{USB_PID:04x}"),
        }
    }
}

pub struct MicView {
    pub key: String,
    pub name: String,
    pub addr: String,
    pub state: &'static str,
    /// green, blue, amber, red or gray.
    pub tone: &'static str,
    pub detail: String,
    pub rssi: String,
    pub quality: &'static str,
    pub quality_label: String,
    pub rate: u32,
    pub wants_stream: bool,
    pub record: bool,
    pub recording: bool,
    pub listeners: usize,
    pub level_pct: u32,
    pub level_text: String,
}

fn quality_value(q: Quality) -> &'static str {
    match q {
        Quality::Low => "Low",
        Quality::Medium => "Medium",
        Quality::High => "High",
    }
}

fn failure_text(f: Failure) -> String {
    match f {
        Failure::ConnectTimeout => "Not in range or asleep (connection timed out).".into(),
        Failure::NotAnLeAudioSource => "This device is not a usable LE Audio source.".into(),
        Failure::Rejected => "The mic rejected the stream setup.".into(),
        Failure::Iso => "The audio link (CIS) was lost.".into(),
        Failure::NoSlot => "The receiver has no free connection slot.".into(),
        Failure::Disconnected(r) => format!("Link dropped (HCI reason 0x{r:02X})."),
    }
}

/// Maps -60..0 dBFS to a 0..100 % bar.
fn level_pct(dbfs: f32) -> u32 {
    (((dbfs + 60.0) / 60.0).clamp(0.0, 1.0) * 100.0).round() as u32
}

impl MicView {
    fn new(m: &Mic, receiver_up: bool) -> MicView {
        let (state, tone) = match m.link {
            None if !receiver_up => ("Offline", "gray"),
            None if m.connecting() => ("Connecting", "amber"),
            None => (
                "Not connected",
                if m.last_failure.is_some() {
                    "red"
                } else {
                    "gray"
                },
            ),
            Some(MicLink::Connecting) => ("Connecting", "amber"),
            Some(MicLink::Discovering) => ("Discovering", "amber"),
            Some(MicLink::Ready) => ("Ready", "blue"),
            Some(MicLink::Starting) => ("Starting stream", "amber"),
            Some(MicLink::Streaming) => ("Streaming", "green"),
            Some(MicLink::Stopping) => ("Stopping stream", "amber"),
            Some(MicLink::Gone(_)) => ("Not connected", "gray"),
        };
        let detail = match (m.link, m.last_failure) {
            (None, _) if !receiver_up => "Waiting for the receiver.".to_string(),
            (None, Some(f)) => {
                let wait = m
                    .next_connect()
                    .map(|t| t.saturating_duration_since(Instant::now()).as_secs())
                    .filter(|s| *s > 0)
                    .map(|s| format!(" Retrying in {s} s."))
                    .unwrap_or_default();
                format!("{}{wait}", failure_text(f))
            }
            _ => String::new(),
        };
        let q = m.cfg.quality;
        let rate = m.stream_rate().unwrap_or(q.sampling_hz());
        let (level_pct, level_text) = match m.level {
            Some(l) => (
                level_pct(l.rms_dbfs),
                format!("{:.0} dBFS rms, {:.0} peak", l.rms_dbfs, l.peak_dbfs),
            ),
            None => (0, "No signal".to_string()),
        };
        MicView {
            key: m.key(),
            name: m.cfg.name.clone(),
            addr: m.cfg.addr.to_string(),
            state,
            tone,
            detail,
            rssi: m.rssi.map(|r| r.to_string()).unwrap_or_else(|| "–".into()),
            quality: quality_value(q),
            quality_label: format!("{} ({} kHz)", quality_value(q), q.sampling_hz() / 1000),
            rate,
            wants_stream: m.wants_stream(),
            record: m.cfg.record,
            recording: m.recording_path().is_some(),
            listeners: m.listeners,
            level_pct,
            level_text,
        }
    }
}

pub struct AdvertView {
    pub name: String,
    pub addr: String,
    pub addr_text: String,
    pub rssi: i8,
    pub known: bool,
}

pub struct AdvertsView {
    pub usable: bool,
    pub scanning: bool,
    pub items: Vec<AdvertView>,
}

impl AdvertsView {
    fn new(e: &Engine) -> AdvertsView {
        AdvertsView {
            usable: e.receiver.usable(),
            scanning: e.scanning(),
            items: e
                .adverts()
                .into_iter()
                .map(|(addr, a)| AdvertView {
                    name: if a.name.is_empty() {
                        "Unnamed mic".into()
                    } else {
                        a.name.clone()
                    },
                    addr: addr.to_string(),
                    addr_text: config::addr_to_text(&addr),
                    rssi: a.rssi,
                    known: e.mic(&addr).is_some(),
                })
                .collect(),
        }
    }
}

// ----- templates -----

#[derive(Template)]
#[template(path = "dashboard.html")]
struct DashboardPage {
    page: &'static str,
    receiver: ReceiverView,
    mics: Vec<MicView>,
}

#[derive(Template)]
#[template(path = "add.html")]
struct AddPage {
    page: &'static str,
    receiver: ReceiverView,
    adverts: AdvertsView,
}

#[derive(Template)]
#[template(path = "recordings.html")]
struct RecordingsPage {
    page: &'static str,
    folders: Vec<RecFolder>,
    root: String,
}

#[derive(Template)]
#[template(path = "not_found.html")]
struct NotFoundPage {
    page: &'static str,
}

#[derive(Template)]
#[template(path = "partials/receiver.html")]
struct ReceiverFragment {
    receiver: ReceiverView,
}

#[derive(Template)]
#[template(path = "partials/mic_card.html")]
struct MicCardFragment {
    m: MicView,
}

#[derive(Template)]
#[template(path = "partials/adverts.html")]
struct AdvertsFragment {
    adverts: AdvertsView,
}

fn html<T: Template>(t: &T) -> Response {
    match t.render() {
        Ok(s) => Html(s).into_response(),
        Err(e) => {
            tracing::error!("rendering: {e}");
            (StatusCode::INTERNAL_SERVER_ERROR, "template error").into_response()
        }
    }
}

// ----- pages -----

async fn dashboard(State(app): State<AppState>) -> Response {
    let page = {
        let e = lock(&app.engine);
        let up = e.receiver.usable();
        DashboardPage {
            page: "dashboard",
            receiver: ReceiverView::new(&e.receiver),
            mics: e.mics().iter().map(|m| MicView::new(m, up)).collect(),
        }
    };
    html(&page)
}

async fn add_page(State(app): State<AppState>) -> Response {
    let page = {
        let mut e = lock(&app.engine);
        e.scan_hold(Instant::now());
        AddPage {
            page: "add",
            receiver: ReceiverView::new(&e.receiver),
            adverts: AdvertsView::new(&e),
        }
    };
    html(&page)
}

async fn not_found() -> Response {
    let mut r = html(&NotFoundPage { page: "" });
    *r.status_mut() = StatusCode::NOT_FOUND;
    r
}

async fn app_js() -> Response {
    (
        [
            (header::CONTENT_TYPE, "text/javascript; charset=utf-8"),
            (header::CACHE_CONTROL, "no-cache"),
        ],
        APP_JS,
    )
        .into_response()
}

// ----- recordings -----

pub struct RecFile {
    pub name: String,
    pub url: String,
    pub duration: String,
    pub size: String,
    pub live: bool,
}

pub struct RecDay {
    pub date: String,
    pub files: Vec<RecFile>,
}

pub struct RecFolder {
    pub name: String,
    pub days: Vec<RecDay>,
}

/// Percent-encodes one URL path segment.
fn url_segment(s: &str) -> String {
    let mut out = String::with_capacity(s.len());
    for b in s.bytes() {
        if b.is_ascii_alphanumeric() || matches!(b, b'-' | b'_' | b'.' | b'~') {
            out.push(b as char);
        } else {
            out.push_str(&format!("%{b:02X}"));
        }
    }
    out
}

fn sorted_dirs(dir: &Path) -> Vec<(String, PathBuf)> {
    let mut v: Vec<(String, PathBuf)> = std::fs::read_dir(dir)
        .into_iter()
        .flatten()
        .flatten()
        .filter_map(|e| Some((e.file_name().into_string().ok()?, e.path())))
        .collect();
    v.sort();
    v
}

fn human_size(n: u64) -> String {
    match n {
        n if n >= 1 << 30 => format!("{:.1} GB", n as f64 / f64::from(1u32 << 30)),
        n if n >= 1 << 20 => format!("{:.1} MB", n as f64 / f64::from(1u32 << 20)),
        n => format!("{} kB", n.div_ceil(1024)),
    }
}

fn duration_text(secs: u64) -> String {
    format!("{}:{:02}:{:02}", secs / 3600, secs / 60 % 60, secs % 60)
}

/// `<root>/<mic>/<date>/<time>.wav`, newest day first.
pub fn list_recordings(root: &Path, live: &HashSet<PathBuf>) -> Vec<RecFolder> {
    let mut folders = Vec::new();
    for (mic, mic_path) in sorted_dirs(root) {
        if !mic_path.is_dir() {
            continue;
        }
        let mut days = Vec::new();
        for (date, day_path) in sorted_dirs(&mic_path).into_iter().rev() {
            if !day_path.is_dir() {
                continue;
            }
            let mut files = Vec::new();
            for (file, path) in sorted_dirs(&day_path).into_iter().rev() {
                if !file.ends_with(".wav") {
                    continue;
                }
                let size = std::fs::metadata(&path).map(|m| m.len()).unwrap_or(0);
                let duration = hound::WavReader::open(&path)
                    .map(|r| u64::from(r.duration()) / u64::from(r.spec().sample_rate.max(1)))
                    .map(duration_text)
                    .unwrap_or_else(|_| "–".into());
                files.push(RecFile {
                    name: file.trim_end_matches(".wav").replace('-', ":"),
                    url: format!(
                        "/recordings/files/{}/{}/{}",
                        url_segment(&mic),
                        url_segment(&date),
                        url_segment(&file)
                    ),
                    duration,
                    size: human_size(size),
                    live: live.contains(&path),
                });
            }
            if !files.is_empty() {
                days.push(RecDay { date, files });
            }
        }
        if !days.is_empty() {
            folders.push(RecFolder { name: mic, days });
        }
    }
    folders
}

async fn recordings_page(State(app): State<AppState>) -> Response {
    let (root, live) = {
        let e = lock(&app.engine);
        let live: HashSet<PathBuf> = e
            .mics()
            .iter()
            .filter_map(|m| m.recording_path().map(Path::to_path_buf))
            .collect();
        (e.paths().recordings.clone(), live)
    };
    let root2 = root.clone();
    let folders = tokio::task::spawn_blocking(move || list_recordings(&root2, &live))
        .await
        .unwrap_or_default();
    html(&RecordingsPage {
        page: "recordings",
        folders,
        root: root.display().to_string(),
    })
}

// ----- actions -----

/// Applies a user action to the mic named by its URL key, then back to the
/// dashboard.
fn on_mic(
    app: &AppState,
    key: &str,
    act: impl FnOnce(&mut Engine, &Addr, Instant) -> bool,
) -> Response {
    let found =
        config::addr_from_key(key).is_some_and(|a| act(&mut lock(&app.engine), &a, Instant::now()));
    if found {
        Redirect::to("/").into_response()
    } else {
        (StatusCode::NOT_FOUND, "no such mic").into_response()
    }
}

#[derive(Deserialize)]
struct AddForm {
    addr: String,
    name: String,
}

async fn add_mic(State(app): State<AppState>, Form(f): Form<AddForm>) -> Response {
    let Some(addr) = config::addr_from_text(&f.addr) else {
        return (StatusCode::BAD_REQUEST, "bad address").into_response();
    };
    lock(&app.engine).add_mic(addr, &f.name, Instant::now());
    Redirect::to("/").into_response()
}

#[derive(Deserialize)]
struct OnForm {
    on: bool,
}

async fn set_stream(
    State(app): State<AppState>,
    UrlPath(key): UrlPath<String>,
    Form(f): Form<OnForm>,
) -> Response {
    on_mic(&app, &key, |e, a, now| e.set_streaming(a, f.on, now))
}

async fn set_record(
    State(app): State<AppState>,
    UrlPath(key): UrlPath<String>,
    Form(f): Form<OnForm>,
) -> Response {
    on_mic(&app, &key, |e, a, now| e.set_recording(a, f.on, now))
}

#[derive(Deserialize)]
struct SettingsForm {
    name: String,
    quality: Quality,
}

async fn configure(
    State(app): State<AppState>,
    UrlPath(key): UrlPath<String>,
    Form(f): Form<SettingsForm>,
) -> Response {
    on_mic(&app, &key, |e, a, now| {
        e.configure_mic(a, &f.name, f.quality, now)
    })
}

async fn forget(State(app): State<AppState>, UrlPath(key): UrlPath<String>) -> Response {
    on_mic(&app, &key, |e, a, _| e.forget_mic(a))
}

// ----- live updates -----

/// Keeps the dongle scanning while an Add page's event stream is open.
struct ScanGuard(Arc<Mutex<Engine>>);

impl Drop for ScanGuard {
    fn drop(&mut self) {
        lock(&self.0).scan_watch(false, Instant::now());
    }
}

#[derive(Deserialize)]
struct EventsQuery {
    scan: Option<String>,
}

fn html_event<T: Template>(t: &T) -> Option<Event> {
    match t.render() {
        Ok(s) => {
            // Template indentation leaves many blank lines; SSE sends each
            // line as its own `data:` field.
            let lines: Vec<&str> = s
                .lines()
                .map(str::trim_end)
                .filter(|l| !l.trim_start().is_empty())
                .collect();
            Some(Event::default().event("html").data(lines.join("\n")))
        }
        Err(e) => {
            tracing::error!("rendering fragment: {e}");
            None
        }
    }
}

fn change_events(engine: &Mutex<Engine>, change: Change, scan: bool) -> Vec<Event> {
    let e = lock(engine);
    let mut out = Vec::new();
    match change {
        Change::Receiver => {
            out.extend(html_event(&ReceiverFragment {
                receiver: ReceiverView::new(&e.receiver),
            }));
            if scan {
                out.extend(html_event(&AdvertsFragment {
                    adverts: AdvertsView::new(&e),
                }));
            }
            // Card states depend on whether the receiver is up.
            let up = e.receiver.usable();
            for m in e.mics() {
                out.extend(html_event(&MicCardFragment {
                    m: MicView::new(m, up),
                }));
            }
        }
        Change::Mic(addr) => {
            if let Some(m) = e.mic(&addr) {
                out.extend(html_event(&MicCardFragment {
                    m: MicView::new(m, e.receiver.usable()),
                }));
            }
        }
        Change::MicSet => {
            out.push(Event::default().event("micset").data("1"));
            if scan {
                out.extend(html_event(&AdvertsFragment {
                    adverts: AdvertsView::new(&e),
                }));
            }
        }
        Change::Adverts => {
            if scan {
                out.extend(html_event(&AdvertsFragment {
                    adverts: AdvertsView::new(&e),
                }));
            }
        }
        Change::Meter(addr, l) => {
            let json = serde_json::json!({
                "key": config::addr_key(&addr),
                "pct": level_pct(l.rms_dbfs),
                "text": format!("{:.0} dBFS rms, {:.0} peak", l.rms_dbfs, l.peak_dbfs),
            });
            out.push(Event::default().event("meter").data(json.to_string()));
        }
        Change::Rssi(addr, rssi) => {
            let json = serde_json::json!({ "key": config::addr_key(&addr), "rssi": rssi });
            out.push(Event::default().event("rssi").data(json.to_string()));
        }
    }
    out
}

async fn events(
    State(app): State<AppState>,
    Query(q): Query<EventsQuery>,
) -> Sse<impl Stream<Item = Result<Event, Infallible>>> {
    let scan = q.scan.is_some();
    let (rx, guard) = {
        let mut e = lock(&app.engine);
        let rx = e.subscribe();
        if scan {
            e.scan_watch(true, Instant::now());
        }
        (rx, scan.then(|| ScanGuard(app.engine.clone())))
    };
    // Start with the current state: whatever changed between rendering the
    // page and subscribing here would otherwise be missed.
    let initial = VecDeque::from(change_events(&app.engine, Change::Receiver, scan));
    let state = (rx, app.engine, guard, initial);
    let stream =
        futures_util::stream::unfold(state, |(mut rx, engine, guard, mut queue)| async move {
            loop {
                if let Some(ev) = queue.pop_front() {
                    return Some((Ok(ev), (rx, engine, guard, queue)));
                }
                match rx.recv().await {
                    Ok(change) => queue.extend(change_events(&engine, change, guard.is_some())),
                    // Missed updates: the page re-renders itself.
                    Err(RecvError::Lagged(_)) => {
                        queue.push_back(Event::default().event("reload").data("1"))
                    }
                    Err(RecvError::Closed) => return None,
                }
            }
        });
    Sse::new(stream).keep_alive(KeepAlive::default())
}

// ----- live listening -----

/// Counts a listener for as long as its socket lives (the mic streams while
/// anyone listens).
struct ListenGuard(Arc<Mutex<Engine>>, Addr);

impl Drop for ListenGuard {
    fn drop(&mut self) {
        lock(&self.0).remove_listener(&self.1, Instant::now());
    }
}

async fn listen(
    State(app): State<AppState>,
    UrlPath(key): UrlPath<String>,
    ws: WebSocketUpgrade,
) -> Response {
    let Some(addr) = config::addr_from_key(&key) else {
        return (StatusCode::NOT_FOUND, "no such mic").into_response();
    };
    let Some((rx, rate)) = lock(&app.engine).add_listener(&addr, Instant::now()) else {
        return (StatusCode::NOT_FOUND, "no such mic").into_response();
    };
    let guard = ListenGuard(app.engine.clone(), addr);
    ws.on_upgrade(move |socket| listen_socket(socket, rx, rate, guard))
}

fn rate_message(rate: u32) -> Message {
    Message::Text(format!("{{\"rate\":{rate}}}").into())
}

/// Sends `{"rate":N}` whenever the stream's rate is (re)announced, then
/// binary messages of little-endian i16 mono samples.
async fn listen_socket(
    mut socket: WebSocket,
    mut rx: tokio::sync::broadcast::Receiver<crate::engine::Pcm>,
    rate: Option<u32>,
    _guard: ListenGuard,
) {
    let mut sent_rate = 0;
    if let Some(r) = rate {
        sent_rate = r;
        if socket.send(rate_message(r)).await.is_err() {
            return;
        }
    }
    loop {
        tokio::select! {
            pcm = rx.recv() => match pcm {
                Ok(p) => {
                    if p.rate != sent_rate {
                        sent_rate = p.rate;
                        if socket.send(rate_message(p.rate)).await.is_err() {
                            return;
                        }
                    }
                    let mut bytes = Vec::with_capacity(p.samples.len() * 2);
                    for s in p.samples.iter() {
                        bytes.extend_from_slice(&s.to_le_bytes());
                    }
                    if socket.send(Message::Binary(bytes.into())).await.is_err() {
                        return;
                    }
                }
                Err(RecvError::Lagged(_)) => {}
                Err(RecvError::Closed) => return,
            },
            msg = socket.recv() => match msg {
                Some(Ok(Message::Close(_))) => {
                    // Flushes the close reply tungstenite queued.
                    let _ = socket.send(Message::Close(None)).await;
                    return;
                }
                Some(Err(_)) | None => return,
                Some(Ok(_)) => {}
            },
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn url_segments_are_encoded() {
        assert_eq!(url_segment("Apple tree"), "Apple%20tree");
        assert_eq!(url_segment("06-00-00.wav"), "06-00-00.wav");
    }

    #[test]
    fn level_maps_to_percent() {
        assert_eq!(level_pct(-96.0), 0);
        assert_eq!(level_pct(-30.0), 50);
        assert_eq!(level_pct(3.0), 100);
    }
}
