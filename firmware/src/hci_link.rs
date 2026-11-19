//! App-core side of the HCI link to the network core.
//!
//! Boots the network core (which runs the SoftDevice Controller, see
//! `netcore/`) and implements `bt_hci::transport::Transport` over the
//! `hci-shm` rings so trouble-host can drive it as an external controller.
//!
//! Disconnection Complete events are also copied to [`DISCONNECTIONS`]:
//! trouble-host only tracks ACL links, but the audio code needs to know when a
//! CIS goes away.

use core::future::poll_fn;
use core::task::Poll;

use bt_hci::transport::{PacketKind, PacketToController, PacketToHost, Transport};
use embassy_nrf::interrupt::typelevel::{self, Interrupt as _};
use embassy_nrf::pac;
use embassy_nrf::pac::reset::vals::Forceoff;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::waitqueue::AtomicWaker;
use embassy_time::Timer;
use hci_shm::{H4_EVT, MAX_RECORD, Shared};

/// IPC channel we signal the net core on, and the one it signals back on.
/// Must match `netcore/src/hci.rs`.
const IPC_CH_TO_NET: usize = 0;
const IPC_CH_FROM_NET: usize = 1;

const HCI_EVT_DISCONNECTION_COMPLETE: u8 = 0x05;

/// Handles of links (ACL or CIS) the controller reported as disconnected.
pub static DISCONNECTIONS: Channel<CriticalSectionRawMutex, u16, 4> = Channel::new();

static RX_WAKER: AtomicWaker = AtomicWaker::new();

pub struct IpcInterruptHandler;

impl typelevel::Handler<typelevel::IPC> for IpcInterruptHandler {
    unsafe fn on_interrupt() {
        let r = pac::IPC;
        if r.events_receive(IPC_CH_FROM_NET).read() != 0 {
            r.events_receive(IPC_CH_FROM_NET).write_value(0);
            RX_WAKER.wake();
        }
    }
}

/// Prepare shared memory and IPC, then release the network core.
pub fn start_netcore(_irq: impl embassy_nrf::interrupt::typelevel::Binding<typelevel::IPC, IpcInterruptHandler>) -> ShmTransport {
    // Network-core bus accesses are non-secure by default; the shared rings
    // live in (secure) app RAM, so make the network core a secure master.
    pac::SPU.extdomain(0).perm().modify(|w| w.set_secattr(true));

    // SAFETY: the network core is held in force-off until below.
    let shm = unsafe { Shared::init() };

    let r = pac::IPC;
    r.send_cnf(IPC_CH_TO_NET).write(|w| w.0 = 1 << IPC_CH_TO_NET);
    r.receive_cnf(IPC_CH_FROM_NET).write(|w| w.0 = 1 << IPC_CH_FROM_NET);
    r.events_receive(IPC_CH_FROM_NET).write_value(0);
    r.intenset().write(|w| w.0 = 1 << IPC_CH_FROM_NET);
    typelevel::IPC::unpend();
    unsafe { typelevel::IPC::enable() };

    pac::RESET.network().forceoff().write(|w| w.set_forceoff(Forceoff::Release));
    ShmTransport { shm }
}

fn notify_net() {
    pac::IPC.tasks_send(IPC_CH_TO_NET).write_value(1);
}

#[derive(Debug, Clone, Copy, defmt::Format)]
pub enum Error {
    /// The net core sent a record we could not parse.
    Malformed,
}

impl embedded_io::Error for Error {
    fn kind(&self) -> embedded_io::ErrorKind {
        embedded_io::ErrorKind::InvalidData
    }
}

impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_str("malformed HCI record from the network core")
    }
}

impl core::error::Error for Error {}

/// Required by `ExternalController`; parsing from a byte slice can only fail
/// on malformed input.
impl From<bt_hci::ReadHciError<core::convert::Infallible>> for Error {
    fn from(_: bt_hci::ReadHciError<core::convert::Infallible>) -> Self {
        Error::Malformed
    }
}

/// Byte sink that writes straight into a fixed buffer.
struct Cursor<'a> {
    buf: &'a mut [u8],
    pos: usize,
}

impl embedded_io::ErrorType for Cursor<'_> {
    type Error = embedded_io::ErrorKind;
}

impl embedded_io::Write for Cursor<'_> {
    fn write(&mut self, data: &[u8]) -> Result<usize, Self::Error> {
        let n = data.len().min(self.buf.len() - self.pos);
        if n == 0 && !data.is_empty() {
            return Err(embedded_io::ErrorKind::OutOfMemory);
        }
        self.buf[self.pos..self.pos + n].copy_from_slice(&data[..n]);
        self.pos += n;
        Ok(n)
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}

/// HCI transport over the shared-RAM rings.
///
/// The host-to-controller ring is single-producer: all writers run on the
/// same thread-mode executor and `try_push` never yields, so pushes cannot
/// interleave.
pub struct ShmTransport {
    shm: &'static Shared,
}

impl embedded_io::ErrorType for ShmTransport {
    type Error = Error;
}

impl Transport for ShmTransport {
    async fn read<'a, P: PacketToHost<'a>>(&self, rx: &'a mut [u8]) -> Result<P, Self::Error> {
        let mut rec = [0u8; MAX_RECORD];
        let n = poll_fn(|cx| {
            RX_WAKER.register(cx.waker());
            match self.shm.to_app.try_pop(&mut rec) {
                Some(n) => Poll::Ready(n),
                None => Poll::Pending,
            }
        })
        .await;
        // The net core may be waiting for space in this ring.
        notify_net();

        let (ind, pkt) = (rec[0], &rec[1..n]);
        if ind == H4_EVT && pkt.first() == Some(&HCI_EVT_DISCONNECTION_COMPLETE) && pkt.len() >= 6 && pkt[2] == 0 {
            let handle = u16::from_le_bytes([pkt[3], pkt[4]]) & 0x0fff;
            // Only the audio task consumes this; dropping when full is fine.
            let _ = DISCONNECTIONS.try_send(handle);
        }
        let mut reader = &rec[..n];
        let kind = PacketKind::read(&mut reader).map_err(|_| Error::Malformed)?;
        P::read_hci(kind, &mut reader, rx).map_err(|_| Error::Malformed)
    }

    async fn write<P: PacketToController>(&self, tx: &P) -> Result<(), Self::Error> {
        // ExternalController hands over the bare packet; the H4 indicator
        // comes from its type.
        let mut pkt = [0u8; MAX_RECORD - 1];
        let mut cur = Cursor { buf: &mut pkt, pos: 0 };
        tx.write_hci(&mut cur).map_err(|_| Error::Malformed)?;
        let len = cur.pos;
        while !self.shm.to_net.try_push(P::KIND as u8, &[&pkt[..len]]) {
            // Only when the net core is far behind; it drains within microseconds.
            Timer::after_micros(200).await;
        }
        notify_net();
        Ok(())
    }
}
