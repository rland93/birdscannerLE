//! The USB serial link to the dongle: finding the port, framing, and a
//! blocking I/O thread that reconnects after unplugging.

use std::io::{self, Read, Write};
use std::sync::{Arc, Mutex, MutexGuard};
use std::time::{Duration, Instant};

use garden_link::postcard::accumulator::{CobsAccumulator, FeedResult};
use garden_link::{DongleToHost, HostToDongle, MAX_FRAME, USB_PID, USB_PRODUCT, USB_VID};

use crate::engine::Engine;

/// How often to look for the dongle while it is not connected.
pub const RETRY: Duration = Duration::from_secs(2);
/// Serial read timeout: bounds how long a command from the UI waits.
const READ_TIMEOUT: Duration = Duration::from_millis(20);
/// How often [`Engine::tick`] runs while connected.
const TICK: Duration = Duration::from_millis(250);

pub fn lock(engine: &Mutex<Engine>) -> MutexGuard<'_, Engine> {
    engine.lock().unwrap_or_else(|e| e.into_inner())
}

/// Splits the byte stream from the dongle into messages.
pub struct FrameReader {
    acc: CobsAccumulator<MAX_FRAME>,
    /// Frames that were too long or did not decode.
    pub bad_frames: u64,
}

impl Default for FrameReader {
    fn default() -> Self {
        FrameReader {
            acc: CobsAccumulator::new(),
            bad_frames: 0,
        }
    }
}

impl FrameReader {
    /// Feeds received bytes; `f` gets each complete message.
    pub fn feed(&mut self, mut bytes: &[u8], mut f: impl FnMut(DongleToHost<'_>)) {
        while !bytes.is_empty() {
            bytes = match self.acc.feed_ref::<DongleToHost>(bytes) {
                FeedResult::Consumed => break,
                FeedResult::OverFull(rest) | FeedResult::DeserError(rest) => {
                    self.bad_frames += 1;
                    rest
                }
                FeedResult::Success { data, remaining } => {
                    f(data);
                    remaining
                }
            };
        }
    }
}

/// Serialises `msgs` as COBS frames.
pub fn encode_all(msgs: &[HostToDongle]) -> Vec<u8> {
    let mut out = Vec::new();
    let mut buf = [0u8; MAX_FRAME];
    for m in msgs {
        match garden_link::encode(m, &mut buf) {
            Ok(frame) => out.extend_from_slice(frame),
            Err(e) => tracing::error!("cannot encode {m:?}: {e:?}"),
        }
    }
    out
}

/// Runs the protocol on an open port until it fails: sends Hello, feeds
/// everything received to the engine, writes what the engine wants to send.
/// Returns the error that ended it. `Ok(0)` from `read` counts as a closed
/// port; a timeout is just a quiet moment.
pub fn serve<P: Read + Write + ?Sized>(
    engine: &Mutex<Engine>,
    port: &mut P,
    name: &str,
) -> io::Error {
    let mut reader = FrameReader::default();
    let mut buf = [0u8; 4096];
    lock(engine).link_up(name, Instant::now());
    let mut last_tick = Instant::now();
    loop {
        let out = {
            let mut e = lock(engine);
            let now = Instant::now();
            if now.duration_since(last_tick) >= TICK {
                last_tick = now;
                e.tick(now);
            }
            e.take_outbox()
        };
        if !out.is_empty()
            && let Err(e) = port
                .write_all(&encode_all(&out))
                .and_then(|()| port.flush())
        {
            return e;
        }
        match port.read(&mut buf) {
            Ok(0) => return io::Error::new(io::ErrorKind::UnexpectedEof, "port closed"),
            Ok(n) => {
                let mut e = lock(engine);
                let now = Instant::now();
                let before = reader.bad_frames;
                reader.feed(&buf[..n], |msg| e.handle(&msg, now));
                if reader.bad_frames != before {
                    e.receiver.bad_frames = reader.bad_frames;
                    tracing::debug!("{} undecodable frames so far", reader.bad_frames);
                }
            }
            Err(e)
                if matches!(
                    e.kind(),
                    io::ErrorKind::TimedOut
                        | io::ErrorKind::WouldBlock
                        | io::ErrorKind::Interrupted
                ) => {}
            Err(e) => return e,
        }
    }
}

/// The dongle's serial port, found by USB VID/PID. On macOS the call-out
/// device (`/dev/cu.*`) is preferred over the dial-in one.
pub fn find_dongle() -> Option<String> {
    let ports = serialport::available_ports().ok()?;
    let mut found: Vec<(bool, bool, String)> = ports
        .into_iter()
        .filter_map(|p| match p.port_type {
            serialport::SerialPortType::UsbPort(u) if u.vid == USB_VID && u.pid == USB_PID => {
                let named = u
                    .product
                    .as_deref()
                    .is_some_and(|s| s.contains(USB_PRODUCT));
                Some((!named, p.port_name.starts_with("/dev/tty."), p.port_name))
            }
            _ => None,
        })
        .collect();
    found.sort();
    found.into_iter().next().map(|(_, _, name)| name)
}

/// Starts the serial thread: forever find the port (or use `fixed`), open
/// it, [`serve`] it, and retry every [`RETRY`] after a failure.
pub fn spawn(engine: Arc<Mutex<Engine>>, fixed: Option<String>) -> std::thread::JoinHandle<()> {
    std::thread::Builder::new()
        .name("serial".into())
        .spawn(move || {
            loop {
                let name = match fixed.clone().or_else(find_dongle) {
                    Some(n) => n,
                    None => {
                        lock(&engine).set_problem(None);
                        std::thread::sleep(RETRY);
                        continue;
                    }
                };
                match serialport::new(&name, 115_200).timeout(READ_TIMEOUT).open() {
                    Ok(mut port) => {
                        // CDC-ACM devices commonly treat DTR as "host is listening".
                        if let Err(e) = port.write_data_terminal_ready(true) {
                            tracing::debug!("{name}: setting DTR: {e}");
                        }
                        let err = serve(&engine, &mut *port, &name);
                        lock(&engine).link_down(Some(format!("{name}: {err}")));
                    }
                    Err(e) => {
                        let msg = format!("cannot open {name}: {e}");
                        let mut eng = lock(&engine);
                        if eng.receiver.problem.as_deref() != Some(msg.as_str()) {
                            tracing::warn!("{msg}");
                        }
                        eng.set_problem(Some(msg));
                    }
                }
                std::thread::sleep(RETRY);
            }
        })
        .expect("spawning the serial thread")
}
