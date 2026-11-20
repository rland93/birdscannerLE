//! HCI transport to the SoftDevice Controller running on this same chip.
//!
//! Uses the same raw-HCI layer (`sdc-hci`) as the nRF5340 network core, so
//! trouble-host talks to the controller exactly as it does on the mic, and ISO
//! data (which nrf-sdc's own `Controller` impl cannot return) gets through.
//!
//! [`raw_command`] runs a command without going through trouble-host, for the
//! few commands bt-hci 0.10 encodes incorrectly (LE Set CIG Parameters and LE
//! Create CIS); their events are consumed here.
//!
//! Disconnection Complete events are also reported to
//! [`crate::mics::on_disconnection`] because trouble-host only tracks ACL
//! links and the stream code must learn about lost CISes.

use core::cell::RefCell;
use core::future::poll_fn;
use core::task::Poll;

use bt_hci::transport::{PacketKind, PacketToController, PacketToHost, Transport};
use embassy_sync::waitqueue::AtomicWaker;
use hci_shm::{H4_EVT, MAX_RECORD};
use heapless::Deque;
use nrf_sdc::SoftdeviceController;

const HCI_EVT_DISCONNECTION_COMPLETE: u8 = 0x05;
const HCI_EVT_CMD_COMPLETE: u8 = 0x0e;
const HCI_EVT_CMD_STATUS: u8 = 0x0f;

#[derive(Debug, Clone, Copy, defmt::Format)]
pub enum Error {
    Malformed,
    Controller,
}

impl embedded_io::Error for Error {
    fn kind(&self) -> embedded_io::ErrorKind {
        embedded_io::ErrorKind::InvalidData
    }
}

impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_str(match self {
            Error::Malformed => "malformed HCI packet",
            Error::Controller => "controller rejected packet",
        })
    }
}

impl core::error::Error for Error {}

impl From<bt_hci::ReadHciError<core::convert::Infallible>> for Error {
    fn from(_: bt_hci::ReadHciError<core::convert::Infallible>) -> Self {
        Error::Malformed
    }
}

type Event = ([u8; sdc_hci::EVENT_BUF], usize);

pub struct LocalTransport {
    sdc: &'static SoftdeviceController<'static>,
    /// Command Complete/Status events produced synchronously by `write`.
    pending: RefCell<Deque<Event, 4>>,
    waker: AtomicWaker,
}

impl LocalTransport {
    pub fn new(sdc: &'static SoftdeviceController<'static>) -> Self {
        Self { sdc, pending: RefCell::new(Deque::new()), waker: AtomicWaker::new() }
    }
}

impl embedded_io::ErrorType for LocalTransport {
    type Error = Error;
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

impl Transport for LocalTransport {
    async fn read<'a, P: PacketToHost<'a>>(&self, rx: &'a mut [u8]) -> Result<P, Self::Error> {
        let mut out = [0u8; sdc_hci::GET_BUF];
        let (ind, len) = poll_fn(|cx| {
            self.waker.register(cx.waker());
            self.sdc.register_waker(cx.waker());
            if let Some((evt, len)) = self.pending.borrow_mut().pop_front() {
                out[..len].copy_from_slice(&evt[..len]);
                return Poll::Ready((H4_EVT, len));
            }
            match sdc_hci::try_get(&mut out) {
                Some(got) => Poll::Ready(got),
                None => Poll::Pending,
            }
        })
        .await;

        let pkt = &out[..len];
        if ind == H4_EVT && pkt.first() == Some(&HCI_EVT_DISCONNECTION_COMPLETE) && pkt.len() >= 6 && pkt[2] == 0 {
            let handle = u16::from_le_bytes([pkt[3], pkt[4]]) & 0x0fff;
            crate::mics::on_disconnection(handle, pkt[5]);
        }
        let kind = match ind {
            hci_shm::H4_EVT => PacketKind::Event,
            hci_shm::H4_ACL => PacketKind::AclData,
            hci_shm::H4_ISO => PacketKind::IsoData,
            _ => return Err(Error::Malformed),
        };
        P::read_hci(kind, &mut &pkt[..], rx).map_err(|_| Error::Malformed)
    }

    async fn write<P: PacketToController>(&self, tx: &P) -> Result<(), Self::Error> {
        let mut pkt = [0u8; MAX_RECORD - 1];
        let mut cur = Cursor { buf: &mut pkt, pos: 0 };
        tx.write_hci(&mut cur).map_err(|_| Error::Malformed)?;
        let len = cur.pos;
        let pkt = &pkt[..len];
        match P::KIND {
            PacketKind::Cmd => {
                let mut evt = [0u8; sdc_hci::EVENT_BUF];
                if let Some(len) = sdc_hci::run_command(pkt, &mut evt) {
                    // trouble-host issues one command at a time per slot; a few slots suffice.
                    if self.pending.borrow_mut().push_back((evt, len)).is_err() {
                        defmt::panic!("HCI event queue overflow");
                    }
                    self.waker.wake();
                }
                Ok(())
            }
            PacketKind::AclData => self.sdc.hci_data_put(pkt).map_err(|_| Error::Controller),
            PacketKind::IsoData => self.sdc.hci_iso_data_put(pkt).map_err(|_| Error::Controller),
            _ => Err(Error::Malformed),
        }
    }
}

/// Result of a command run through [`raw_command`].
pub struct RawResult {
    pub status: u8,
    /// Return parameters (Command Complete only).
    ret: [u8; sdc_hci::EVENT_BUF],
    ret_len: usize,
}

impl RawResult {
    pub fn ret(&self) -> &[u8] {
        &self.ret[..self.ret_len]
    }
}

/// Run one HCI command (`opcode`, `params`) directly on the controller,
/// bypassing trouble-host. Only for commands whose completion trouble-host
/// does not need to see.
pub fn raw_command(opcode: u16, params: &[u8]) -> RawResult {
    let mut pkt = [0u8; 3 + 255];
    pkt[..2].copy_from_slice(&opcode.to_le_bytes());
    pkt[2] = params.len() as u8;
    pkt[3..3 + params.len()].copy_from_slice(params);
    let mut evt = [0u8; sdc_hci::EVENT_BUF];
    let mut r = RawResult { status: 0xff, ret: [0; sdc_hci::EVENT_BUF], ret_len: 0 };
    match sdc_hci::run_command(&pkt[..3 + params.len()], &mut evt) {
        Some(len) if evt[0] == HCI_EVT_CMD_COMPLETE && len >= 6 => {
            r.status = evt[5];
            r.ret_len = len - 6;
            r.ret[..r.ret_len].copy_from_slice(&evt[6..len]);
        }
        Some(_) if evt[0] == HCI_EVT_CMD_STATUS => r.status = evt[2],
        _ => {}
    }
    r
}
