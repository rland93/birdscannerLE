//! The application state and its reaction to the dongle and the user.
//!
//! [`Engine`] is plain data driven from outside: the serial thread calls
//! [`Engine::link_up`], [`Engine::handle`], [`Engine::tick`] and
//! [`Engine::link_down`] and writes whatever [`Engine::take_outbox`] returns;
//! the web server calls the user actions. Every observable change is announced
//! on a broadcast channel ([`Engine::subscribe`]) for the live UI.

use std::collections::{BTreeMap, HashMap};
use std::sync::Arc;
use std::time::{Duration, Instant};

use chrono::NaiveDateTime;
use garden_link::{
    Addr, DongleToHost, Failure, HostToDongle, MicId, MicLink, PROTOCOL_VERSION, Quality,
};
use tokio::sync::broadcast;

use crate::audio::{Level, Meter, StreamDecoder};
use crate::config::{self, Config, MicConfig, Paths};
use crate::recorder::Recorder;

/// After the dongle's Hello, wait this long for its slot reports before
/// asking it to connect mics it may already hold.
pub const HELLO_GRACE: Duration = Duration::from_secs(1);
/// Re-send Hello while the dongle has not answered.
pub const HELLO_RETRY: Duration = Duration::from_secs(2);
/// Give up on a Connect the dongle never answered.
pub const CONNECT_TIMEOUT: Duration = Duration::from_secs(20);
/// Reconnect backoff after a mic is Gone: doubles up to the maximum.
pub const BACKOFF_FIRST: Duration = Duration::from_secs(2);
pub const BACKOFF_MAX: Duration = Duration::from_secs(60);
/// Re-send StartStream/StopStream if the mic's state did not move.
pub const REQUEST_RETRY: Duration = Duration::from_secs(10);
/// Forget adverts not heard for this long.
pub const ADVERT_TTL: Duration = Duration::from_secs(30);
/// Keep scanning this long after the Add page was loaded.
pub const SCAN_HOLD: Duration = Duration::from_secs(60);

/// A chunk of decoded audio for live listeners.
#[derive(Clone, Debug)]
pub struct Pcm {
    pub rate: u32,
    pub samples: Arc<[i16]>,
}

/// Something the UI should redraw.
#[derive(Clone, Debug, PartialEq)]
pub enum Change {
    Receiver,
    /// A mic's card (state, settings, recording).
    Mic(Addr),
    /// Mics were added or removed.
    MicSet,
    Adverts,
    Meter(Addr, Level),
    Rssi(Addr, i8),
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct DongleInfo {
    pub protocol: u16,
    pub firmware: String,
    pub max_mics: u8,
}

#[derive(Debug, Default)]
pub struct Receiver {
    /// The serial port, while open.
    pub port: Option<String>,
    /// The dongle's last Hello on this connection.
    pub info: Option<DongleInfo>,
    /// Why we can't use the dongle (protocol mismatch, open errors).
    pub problem: Option<String>,
    /// Frames that did not decode (resync after connect, corruption).
    pub bad_frames: u64,
    hello_sent: Option<Instant>,
    hello_at: Option<Instant>,
}

impl Receiver {
    /// Talking to a dongle that speaks our protocol.
    pub fn usable(&self) -> bool {
        self.port.is_some()
            && self
                .info
                .as_ref()
                .is_some_and(|i| i.protocol == PROTOCOL_VERSION)
    }
}

#[derive(Clone, Debug)]
pub struct Advert {
    pub name: String,
    pub rssi: i8,
    seen: Instant,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Stats {
    pub frames_decoded: u64,
    pub frames_concealed: u64,
    pub frames_dropped: u64,
    pub samples: u64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Request {
    Start,
    Stop,
}

/// A known mic: its settings and what we know about it right now.
#[derive(Debug)]
pub struct Mic {
    pub cfg: MicConfig,
    /// The dongle's slot, while it holds the mic.
    pub slot: Option<MicId>,
    /// The link state; `None` while no slot holds the mic.
    pub link: Option<MicLink>,
    pub rssi: Option<i8>,
    pub last_failure: Option<Failure>,
    pub listeners: usize,
    pub level: Option<Level>,
    pub stats: Stats,
    stream: Option<StreamDecoder>,
    meter: Option<Meter>,
    recorder: Option<Recorder>,
    /// When we sent Connect, while waiting for the dongle to report the mic.
    connect_sent: Option<Instant>,
    next_connect: Option<Instant>,
    backoff: Duration,
    request: Option<(Request, Instant)>,
    audio_tx: broadcast::Sender<Pcm>,
}

impl Mic {
    fn new(cfg: MicConfig) -> Mic {
        Mic {
            cfg,
            slot: None,
            link: None,
            rssi: None,
            last_failure: None,
            listeners: 0,
            level: None,
            stats: Stats::default(),
            stream: None,
            meter: None,
            recorder: None,
            connect_sent: None,
            next_connect: None,
            backoff: BACKOFF_FIRST,
            request: None,
            audio_tx: broadcast::channel(64).0,
        }
    }

    pub fn key(&self) -> String {
        config::addr_key(&self.cfg.addr)
    }

    /// Something wants audio from this mic.
    pub fn wants_stream(&self) -> bool {
        self.cfg.stream || self.cfg.record || self.listeners > 0
    }

    /// The running stream's sample rate.
    pub fn stream_rate(&self) -> Option<u32> {
        self.stream.as_ref().map(|s| s.sampling_hz)
    }

    /// The WAV file being written.
    pub fn recording_path(&self) -> Option<&std::path::Path> {
        self.recorder.as_ref().map(|r| r.path())
    }

    /// A Connect is out and unanswered.
    pub fn connecting(&self) -> bool {
        self.connect_sent.is_some()
    }

    /// When the next Connect goes out (while not held by the dongle).
    pub fn next_connect(&self) -> Option<Instant> {
        self.next_connect
    }
}

pub type Clock = Box<dyn Fn() -> NaiveDateTime + Send>;

pub struct Engine {
    paths: Paths,
    clock: Clock,
    pub receiver: Receiver,
    mics: Vec<Mic>,
    /// Every occupied dongle slot, known mic or not.
    slots: HashMap<MicId, Addr>,
    adverts: BTreeMap<Addr, Advert>,
    adverts_dirty: bool,
    scanners: usize,
    scan_until: Option<Instant>,
    scanning: bool,
    outbox: Vec<HostToDongle>,
    changes: broadcast::Sender<Change>,
    closed: bool,
}

impl Engine {
    pub fn new(cfg: Config, paths: Paths, clock: Clock) -> Engine {
        Engine {
            paths,
            clock,
            receiver: Receiver::default(),
            mics: cfg.mics.into_iter().map(Mic::new).collect(),
            slots: HashMap::new(),
            adverts: BTreeMap::new(),
            adverts_dirty: false,
            scanners: 0,
            scan_until: None,
            scanning: false,
            outbox: Vec::new(),
            changes: broadcast::channel(1024).0,
            closed: false,
        }
    }

    /// Wall-clock local time, as used for recordings.
    pub fn local_now() -> NaiveDateTime {
        chrono::Local::now().naive_local()
    }

    pub fn subscribe(&self) -> broadcast::Receiver<Change> {
        self.changes.subscribe()
    }

    pub fn paths(&self) -> &Paths {
        &self.paths
    }

    pub fn mics(&self) -> &[Mic] {
        &self.mics
    }

    pub fn mic(&self, addr: &Addr) -> Option<&Mic> {
        self.mics.iter().find(|m| m.cfg.addr == *addr)
    }

    /// Adverts heard while scanning, strongest first.
    pub fn adverts(&self) -> Vec<(Addr, &Advert)> {
        let mut v: Vec<_> = self.adverts.iter().map(|(a, ad)| (*a, ad)).collect();
        v.sort_by_key(|(_, ad)| std::cmp::Reverse(ad.rssi));
        v
    }

    pub fn scanning(&self) -> bool {
        self.scanning
    }

    /// Messages for the dongle, oldest first.
    pub fn take_outbox(&mut self) -> Vec<HostToDongle> {
        std::mem::take(&mut self.outbox)
    }

    fn emit(&self, c: Change) {
        // No subscriber is fine.
        let _ = self.changes.send(c);
    }

    fn send(&mut self, msg: HostToDongle) {
        if self.receiver.port.is_some() && !self.closed {
            tracing::debug!(?msg, "to dongle");
            self.outbox.push(msg);
        }
    }

    fn save(&self) {
        let cfg = Config {
            mics: self.mics.iter().map(|m| m.cfg.clone()).collect(),
        };
        if let Err(e) = cfg.save(&self.paths.config_file) {
            tracing::error!("saving {}: {e}", self.paths.config_file.display());
        }
    }

    // ----- the serial link -----

    /// A serial port to the dongle was opened.
    pub fn link_up(&mut self, port: &str, now: Instant) {
        tracing::info!("receiver on {port}");
        self.receiver.port = Some(port.to_string());
        self.receiver.info = None;
        self.receiver.problem = None;
        self.receiver.hello_at = None;
        self.receiver.hello_sent = Some(now);
        self.outbox.clear();
        self.send(HostToDongle::Hello);
        self.emit(Change::Receiver);
    }

    /// The port closed or failed; every mic is offline as far as we know.
    pub fn link_down(&mut self, problem: Option<String>) {
        if self.receiver.port.is_some() {
            tracing::warn!(
                "receiver disconnected{}",
                problem
                    .as_deref()
                    .map(|p| format!(": {p}"))
                    .unwrap_or_default()
            );
        }
        self.receiver.port = None;
        self.receiver.info = None;
        self.receiver.hello_at = None;
        self.receiver.hello_sent = None;
        self.receiver.problem = problem;
        self.outbox.clear();
        self.slots.clear();
        self.adverts.clear();
        self.scanning = false;
        for i in 0..self.mics.len() {
            let m = &mut self.mics[i];
            m.slot = None;
            m.link = None;
            m.rssi = None;
            m.connect_sent = None;
            m.next_connect = None;
            m.backoff = BACKOFF_FIRST;
            m.request = None;
            self.end_stream(i);
            self.emit(Change::Mic(self.mics[i].cfg.addr));
        }
        self.emit(Change::Receiver);
        self.emit(Change::Adverts);
    }

    /// Records why no port could be opened (shown while disconnected).
    pub fn set_problem(&mut self, problem: Option<String>) {
        if self.receiver.problem != problem {
            self.receiver.problem = problem;
            self.emit(Change::Receiver);
        }
    }

    /// Timers: Hello retries, reconnect backoff, advert expiry.
    pub fn tick(&mut self, now: Instant) {
        if self.receiver.port.is_some()
            && self.receiver.info.is_none()
            && self
                .receiver
                .hello_sent
                .is_none_or(|t| now.duration_since(t) >= HELLO_RETRY)
        {
            self.receiver.hello_sent = Some(now);
            self.send(HostToDongle::Hello);
        }
        let before = self.adverts.len();
        self.adverts
            .retain(|_, a| now.duration_since(a.seen) < ADVERT_TTL);
        if self.adverts.len() != before {
            self.adverts_dirty = true;
        }
        if std::mem::take(&mut self.adverts_dirty) {
            self.emit(Change::Adverts);
        }
        self.reconcile(now);
    }

    /// One message from the dongle.
    pub fn handle(&mut self, msg: &DongleToHost<'_>, now: Instant) {
        if self.closed {
            return;
        }
        if let DongleToHost::Hello {
            protocol,
            firmware,
            max_mics,
        } = *msg
        {
            tracing::info!("dongle firmware {firmware}, protocol {protocol}, {max_mics} mic slots");
            self.receiver.info = Some(DongleInfo {
                protocol,
                firmware: firmware.to_string(),
                max_mics,
            });
            if protocol == PROTOCOL_VERSION {
                self.receiver.problem = None;
                self.receiver.hello_at.get_or_insert(now);
                // A Hello also follows a dongle reset: re-apply the scan state.
                self.scanning = false;
            } else {
                self.receiver.problem = Some(format!(
                    "The receiver speaks protocol {protocol}, this app speaks {PROTOCOL_VERSION}: update the receiver firmware or this app."
                ));
            }
            self.emit(Change::Receiver);
            self.reconcile(now);
            return;
        }
        if let DongleToHost::Log { text } = *msg {
            tracing::info!(target: "dongle", "{text}");
            return;
        }
        if !self.receiver.usable() {
            return;
        }
        match *msg {
            DongleToHost::Hello { .. } | DongleToHost::Log { .. } => {}
            DongleToHost::Advert { addr, rssi, name } => {
                if self.scanning {
                    // Names come in scan responses; the adverts in between
                    // carry none, so keep the last one heard.
                    let name = match (name, self.adverts.get(&addr)) {
                        ("", Some(old)) => old.name.clone(),
                        (name, _) => name.to_string(),
                    };
                    let changed = self
                        .adverts
                        .get(&addr)
                        .is_none_or(|a| a.name != name || a.rssi != rssi);
                    self.adverts.insert(
                        addr,
                        Advert {
                            name,
                            rssi,
                            seen: now,
                        },
                    );
                    self.adverts_dirty |= changed;
                }
            }
            DongleToHost::MicStatus {
                mic,
                addr,
                link,
            } => self.on_status(mic, addr, link, now),
            DongleToHost::Rssi { mic, rssi } => {
                if let Some(i) = self.by_slot(mic) {
                    self.mics[i].rssi = Some(rssi);
                    self.emit(Change::Rssi(self.mics[i].cfg.addr, rssi));
                }
            }
            DongleToHost::StreamConfig {
                mic,
                sampling_hz,
                frame_duration_us,
                octets_per_frame,
            } => {
                if let Some(i) = self.by_slot(mic) {
                    self.start_stream(i, sampling_hz, frame_duration_us, octets_per_frame);
                }
            }
            DongleToHost::Audio { mic, seq, frame } => {
                if let Some(i) = self.by_slot(mic) {
                    self.on_audio(i, seq, frame);
                }
            }
        }
    }

    fn by_slot(&self, id: MicId) -> Option<usize> {
        self.mics.iter().position(|m| m.slot == Some(id))
    }

    fn by_addr(&self, addr: &Addr) -> Option<usize> {
        self.mics.iter().position(|m| m.cfg.addr == *addr)
    }

    fn on_status(&mut self, id: MicId, addr: Addr, link: MicLink, now: Instant) {
        tracing::info!("mic {addr} (slot {}): {link:?}", id.0);
        let gone = matches!(link, MicLink::Gone(_));
        let Some(i) = self.by_addr(&addr) else {
            if gone {
                self.slots.retain(|_, a| *a != addr);
            } else {
                // A mic we don't know (forgotten while we were not running):
                // free the slot once the dongle has settled it.
                let first = self.slots.insert(id, addr).is_none();
                if first || matches!(link, MicLink::Ready | MicLink::Streaming) {
                    self.send(HostToDongle::Disconnect { mic: id });
                }
            }
            return;
        };
        // The slot may have belonged to another mic before (a missed Gone).
        if !gone
            && let Some(old) = self.slots.insert(id, addr).filter(|a| *a != addr)
            && let Some(j) = self.by_addr(&old)
        {
            self.mics[j].slot = None;
            self.mics[j].link = None;
            self.end_stream(j);
            self.emit(Change::Mic(old));
        }
        // From here the mic holds `id`, or no slot at all if it is Gone (whose
        // id may never have been the mic's, e.g. for NoSlot).
        self.slots.retain(|k, a| *a != addr || (!gone && *k == id));
        let m = &mut self.mics[i];
        m.request = None;
        m.connect_sent = None;
        match link {
            MicLink::Gone(f) => {
                m.slot = None;
                m.link = None;
                m.rssi = None;
                m.last_failure = Some(f);
                m.next_connect = Some(now + m.backoff);
                m.backoff = (m.backoff * 2).min(BACKOFF_MAX);
                self.end_stream(i);
            }
            _ => {
                m.slot = Some(id);
                m.link = Some(link);
                if matches!(link, MicLink::Ready | MicLink::Streaming) {
                    m.backoff = BACKOFF_FIRST;
                    m.last_failure = None;
                }
                // The stream belongs to Starting/Streaming (its StreamConfig
                // comes in between); every other state ends it.
                if !matches!(link, MicLink::Starting | MicLink::Streaming) {
                    self.end_stream(i);
                }
            }
        }
        self.emit(Change::Mic(addr));
        self.reconcile(now);
    }

    fn start_stream(&mut self, i: usize, hz: u32, frame_us: u32, octets: u16) {
        self.end_stream(i);
        let m = &mut self.mics[i];
        tracing::info!(
            "mic {} streams LC3 {hz} Hz, {frame_us} us, {octets} bytes",
            m.cfg.addr
        );
        match StreamDecoder::new(hz, frame_us, octets) {
            Ok(d) => {
                m.stream = Some(d);
                m.meter = Some(Meter::new(hz));
                if m.cfg.record {
                    self.start_recording(i);
                }
            }
            Err(e) => tracing::error!(
                "mic {}: unsupported stream {hz} Hz / {frame_us} us: {e:?}",
                m.cfg.addr
            ),
        }
        self.emit(Change::Mic(self.mics[i].cfg.addr));
    }

    fn end_stream(&mut self, i: usize) {
        let m = &mut self.mics[i];
        let had = m.stream.take().is_some();
        m.meter = None;
        m.level = None;
        if let Some(r) = m.recorder.take() {
            match r.finish() {
                Ok(p) => tracing::info!("recorded {}", p.display()),
                Err(e) => tracing::error!("finishing recording: {e}"),
            }
        }
        if had {
            self.emit(Change::Mic(self.mics[i].cfg.addr));
        }
    }

    fn start_recording(&mut self, i: usize) {
        let m = &mut self.mics[i];
        let Some(rate) = m.stream.as_ref().map(|s| s.sampling_hz) else {
            return;
        };
        if m.recorder.is_some() {
            return;
        }
        let dir = self
            .paths
            .recordings
            .join(config::folder_name(&m.cfg.name, &m.cfg.addr));
        match Recorder::start(&dir, rate, (self.clock)()) {
            Ok(r) => m.recorder = Some(r),
            Err(e) => tracing::error!("cannot record to {}: {e}", dir.display()),
        }
    }

    fn on_audio(&mut self, i: usize, seq: u16, frame: &[u8]) {
        let now = (self.clock)();
        let m = &mut self.mics[i];
        let Some(dec) = m.stream.as_mut() else {
            m.stats.frames_dropped += 1;
            return;
        };
        let rate = dec.sampling_hz;
        let (recorder, meter, tx) = (&mut m.recorder, &mut m.meter, &m.audio_tx);
        let mut level = None;
        let mut samples = 0u64;
        let mut failed = None;
        let pushed = dec.push(seq, frame, |pcm| {
            samples += pcm.len() as u64;
            if tx.receiver_count() > 0 {
                let _ = tx.send(Pcm {
                    rate,
                    samples: pcm.into(),
                });
            }
            if let Some(r) = recorder.as_mut()
                && let Err(e) = r.write(pcm, now)
            {
                failed = Some(e);
            }
            if let Some(l) = meter.as_mut().and_then(|mt| mt.feed(pcm)) {
                level = Some(l);
            }
        });
        m.stats.frames_decoded += u64::from(pushed.decoded);
        m.stats.frames_concealed += u64::from(pushed.concealed);
        m.stats.frames_dropped += u64::from(pushed.dropped);
        m.stats.samples += samples;
        let addr = m.cfg.addr;
        if let Some(l) = level {
            m.level = Some(l);
        }
        if let Some(e) = failed {
            tracing::error!("recording {addr} stopped: {e}");
            m.recorder = None;
            self.emit(Change::Mic(addr));
        }
        if let Some(l) = level {
            self.emit(Change::Meter(addr, l));
        }
    }

    /// Sends whatever moves each mic towards what the user wants.
    fn reconcile(&mut self, now: Instant) {
        if !self.receiver.usable() || self.closed {
            return;
        }
        let want_scan = self.scanners > 0 || self.scan_until.is_some_and(|t| now < t);
        if want_scan != self.scanning {
            self.scanning = want_scan;
            self.send(HostToDongle::Scan { enable: want_scan });
            if !want_scan {
                self.adverts.clear();
                self.emit(Change::Adverts);
            }
        }
        let settled = self
            .receiver
            .hello_at
            .is_some_and(|t| now.duration_since(t) >= HELLO_GRACE);
        for i in 0..self.mics.len() {
            let m = &mut self.mics[i];
            let mut out = None;
            match (m.link, m.slot) {
                (None, _) => {
                    if let Some(sent) = m.connect_sent {
                        if now.duration_since(sent) < CONNECT_TIMEOUT {
                            continue;
                        }
                        tracing::warn!("mic {}: no answer to Connect", m.cfg.addr);
                        m.connect_sent = None;
                        m.next_connect = Some(now + m.backoff);
                        m.backoff = (m.backoff * 2).min(BACKOFF_MAX);
                    }
                    if settled && m.next_connect.is_none_or(|t| now >= t) {
                        m.connect_sent = Some(now);
                        m.next_connect = None;
                        out = Some(HostToDongle::Connect { addr: m.cfg.addr });
                    }
                }
                (Some(link), Some(mic)) => {
                    let idle = m
                        .request
                        .is_none_or(|(_, t)| now.duration_since(t) >= REQUEST_RETRY);
                    let rate_wrong = m
                        .stream_rate()
                        .is_some_and(|r| r != m.cfg.quality.sampling_hz());
                    if idle && link == MicLink::Ready && m.wants_stream() {
                        m.request = Some((Request::Start, now));
                        out = Some(HostToDongle::StartStream {
                            mic,
                            quality: m.cfg.quality,
                        });
                    } else if idle
                        && link == MicLink::Streaming
                        && (!m.wants_stream() || rate_wrong)
                    {
                        // A quality change restarts the stream: Ready again,
                        // then StartStream with the new setting.
                        m.request = Some((Request::Stop, now));
                        out = Some(HostToDongle::StopStream { mic });
                    }
                }
                (Some(_), None) => {}
            }
            if let Some(msg) = out {
                self.send(msg);
            }
        }
    }

    // ----- user actions -----

    /// Adds a mic and asks the dongle to connect it; `false` if it was
    /// known already.
    pub fn add_mic(&mut self, addr: Addr, name: &str, now: Instant) -> bool {
        if self.by_addr(&addr).is_some() {
            return false;
        }
        let name = self.unique_name(name.trim(), None, &addr);
        tracing::info!("adding mic {addr} as {name:?}");
        self.mics.push(Mic::new(MicConfig {
            addr,
            name,
            quality: Quality::Medium,
            stream: false,
            record: false,
        }));
        self.save();
        self.emit(Change::MicSet);
        self.reconcile(now);
        true
    }

    /// Removes a mic from the config and disconnects it if connected.
    pub fn forget_mic(&mut self, addr: &Addr) -> bool {
        let Some(i) = self.by_addr(addr) else {
            return false;
        };
        tracing::info!("forgetting mic {addr}");
        self.end_stream(i);
        let m = self.mics.remove(i);
        if let Some(id) = m.slot {
            self.slots.remove(&id);
            self.send(HostToDongle::Disconnect { mic: id });
        }
        self.save();
        self.emit(Change::MicSet);
        true
    }

    /// Renames a mic and sets its quality (a running stream restarts at the
    /// new quality).
    pub fn configure_mic(
        &mut self,
        addr: &Addr,
        name: &str,
        quality: Quality,
        now: Instant,
    ) -> bool {
        let Some(i) = self.by_addr(addr) else {
            return false;
        };
        let name = self.unique_name(name.trim(), Some(i), addr);
        let m = &mut self.mics[i];
        m.cfg.name = name;
        m.cfg.quality = quality;
        m.request = None;
        self.save();
        self.emit(Change::Mic(*addr));
        self.reconcile(now);
        true
    }

    /// Start stream: keep it running. Stop stream: also stops recording.
    pub fn set_streaming(&mut self, addr: &Addr, on: bool, now: Instant) -> bool {
        let Some(i) = self.by_addr(addr) else {
            return false;
        };
        let m = &mut self.mics[i];
        m.cfg.stream = on;
        if !on {
            m.cfg.record = false;
            if let Some(r) = m.recorder.take() {
                let _ = r.finish();
            }
        }
        m.request = None;
        self.save();
        self.emit(Change::Mic(*addr));
        self.reconcile(now);
        true
    }

    pub fn set_recording(&mut self, addr: &Addr, on: bool, now: Instant) -> bool {
        let Some(i) = self.by_addr(addr) else {
            return false;
        };
        let m = &mut self.mics[i];
        m.cfg.record = on;
        m.request = None;
        if on {
            self.start_recording(i);
        } else if let Some(r) = self.mics[i].recorder.take()
            && let Err(e) = r.finish()
        {
            tracing::error!("finishing recording: {e}");
        }
        self.save();
        self.emit(Change::Mic(*addr));
        self.reconcile(now);
        true
    }

    /// A live listener joins: returns the audio feed and the current rate.
    pub fn add_listener(
        &mut self,
        addr: &Addr,
        now: Instant,
    ) -> Option<(broadcast::Receiver<Pcm>, Option<u32>)> {
        let i = self.by_addr(addr)?;
        let m = &mut self.mics[i];
        m.listeners += 1;
        let rx = m.audio_tx.subscribe();
        let rate = m.stream_rate();
        self.emit(Change::Mic(*addr));
        self.reconcile(now);
        Some((rx, rate))
    }

    pub fn remove_listener(&mut self, addr: &Addr, now: Instant) {
        if let Some(i) = self.by_addr(addr) {
            let m = &mut self.mics[i];
            m.listeners = m.listeners.saturating_sub(1);
            self.emit(Change::Mic(*addr));
            self.reconcile(now);
        }
    }

    /// A page that shows adverts opened (`true`) or closed its live feed.
    pub fn scan_watch(&mut self, on: bool, now: Instant) {
        if on {
            self.scanners += 1;
        } else {
            self.scanners = self.scanners.saturating_sub(1);
        }
        self.reconcile(now);
    }

    /// Scan for a while even without a live feed (the Add page was loaded).
    pub fn scan_hold(&mut self, now: Instant) {
        self.scan_until = Some(now + SCAN_HOLD);
        self.reconcile(now);
    }

    /// Finishes recordings and stops reacting (the app is exiting).
    pub fn shutdown(&mut self) {
        for i in 0..self.mics.len() {
            self.end_stream(i);
        }
        self.closed = true;
    }

    fn unique_name(&self, wanted: &str, skip: Option<usize>, addr: &Addr) -> String {
        let base = if wanted.is_empty() {
            format!("Mic {}", &config::addr_key(addr)[8..12])
        } else {
            wanted.to_string()
        };
        let taken = |n: &str| {
            self.mics
                .iter()
                .enumerate()
                .any(|(j, m)| Some(j) != skip && m.cfg.name.eq_ignore_ascii_case(n))
        };
        if !taken(&base) {
            return base;
        }
        (2..)
            .map(|k| format!("{base} {k}"))
            .find(|n| !taken(n))
            .unwrap_or(base)
    }
}
