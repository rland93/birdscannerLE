//! Raw HCI bridge: H4 records from the app core go into the SoftDevice
//! Controller, everything the controller produces goes back.
//!
//! The SDC has no "put raw command" entry point (sdk-nrf's `hci_internal.c`
//! fills that role in C); [`crate::dispatch`] maps opcodes to the SDC's
//! per-command functions, whose parameter/return structs are packed and
//! identical to the HCI wire format.

use core::future::poll_fn;
use core::mem::size_of;
use core::task::Poll;

use defmt::{debug, warn};
use embassy_nrf::interrupt::typelevel::{self, Interrupt as _};
use embassy_nrf::pac;
use embassy_sync::waitqueue::AtomicWaker;
use hci_shm::{H4_ACL, H4_CMD, H4_EVT, H4_ISO, MAX_RECORD, Shared};
use nrf_sdc::{SoftdeviceController, raw};

use crate::dispatch::dispatch;

/// IPC channel the app core signals on, and the one we signal back on.
const IPC_CH_FROM_APP: usize = 0;
const IPC_CH_TO_APP: usize = 1;

const HCI_EVT_CMD_COMPLETE: u8 = 0x0e;
const HCI_EVT_CMD_STATUS: u8 = 0x0f;
const HCI_ERR_UNKNOWN_CMD: u8 = 0x01;
const OP_HOST_NUM_COMPLETED_PACKETS: u16 = 0x0c35;
const OP_LE_SET_CIG_PARAMS: u16 = 0x2062;
const OP_LE_SET_CIG_PARAMS_TEST: u16 = 0x2063;

pub(crate) struct Done {
    status: u8,
    ret_len: usize,
    command_status: bool,
}

impl Done {
    pub(crate) fn status_event(self) -> Self {
        Self { command_status: true, ..self }
    }
}

pub(crate) unsafe fn call_n(f: unsafe extern "C" fn() -> u8) -> Done {
    Done { status: unsafe { f() }, ret_len: 0, command_status: false }
}

pub(crate) unsafe fn call_p<P>(f: unsafe extern "C" fn(*const P) -> u8, params: *const u8) -> Done {
    Done { status: unsafe { f(params.cast()) }, ret_len: 0, command_status: false }
}

pub(crate) unsafe fn call_r<R>(f: unsafe extern "C" fn(*mut R) -> u8, ret: &mut [u8]) -> Done {
    assert!(size_of::<R>() <= ret.len());
    Done { status: unsafe { f(ret.as_mut_ptr().cast()) }, ret_len: size_of::<R>(), command_status: false }
}

pub(crate) unsafe fn call_pr<P, R>(
    f: unsafe extern "C" fn(*const P, *mut R) -> u8,
    params: *const u8,
    ret: &mut [u8],
) -> Done {
    assert!(size_of::<R>() <= ret.len());
    Done { status: unsafe { f(params.cast(), ret.as_mut_ptr().cast()) }, ret_len: size_of::<R>(), command_status: false }
}

/// Execute one HCI command packet (`opcode(2) len(1) params`); writes the
/// resulting Command Complete/Status event into `evt` and returns its length,
/// or `None` if the command produces no event.
fn run_command(pkt: &[u8], evt: &mut [u8; 260]) -> Option<usize> {
    let opcode = u16::from_le_bytes([pkt[0], pkt[1]]);
    // Zero-padded, max-size parameter block: SDC functions read fixed-size
    // structs (and trailing flexible arrays) straight from it.
    let mut params = [0u8; 256];
    let plen = (pkt[2] as usize).min(pkt.len() - 3);
    params[..plen].copy_from_slice(&pkt[3..3 + plen]);
    let mut ret = [0u8; 255];

    // SAFETY: `params` is 256 readable bytes; `ret` is large enough for every return struct.
    let done = unsafe { dispatch(opcode, params.as_ptr(), &mut ret) };
    let [op_lo, op_hi] = opcode.to_le_bytes();
    let Some(done) = done else {
        warn!("HCI: unsupported opcode {=u16:#06x}", opcode);
        evt[..6].copy_from_slice(&[HCI_EVT_CMD_STATUS, 4, HCI_ERR_UNKNOWN_CMD, 1, op_lo, op_hi]);
        return Some(6);
    };
    if opcode == OP_HOST_NUM_COMPLETED_PACKETS && done.status == 0 {
        // Spec: no event on success.
        return None;
    }
    if done.command_status {
        evt[..6].copy_from_slice(&[HCI_EVT_CMD_STATUS, 4, done.status, 1, op_lo, op_hi]);
        return Some(6);
    }
    let ret_len = match opcode {
        // Return struct ends in a flexible array of CIS handles.
        OP_LE_SET_CIG_PARAMS | OP_LE_SET_CIG_PARAMS_TEST => 2 + 2 * ret[1] as usize,
        _ => done.ret_len,
    };
    evt[..6].copy_from_slice(&[HCI_EVT_CMD_COMPLETE, (4 + ret_len) as u8, 1, op_lo, op_hi, done.status]);
    evt[6..6 + ret_len].copy_from_slice(&ret[..ret_len]);
    Some(6 + ret_len)
}

static IPC_WAKER: AtomicWaker = AtomicWaker::new();

pub struct IpcInterruptHandler;

impl typelevel::Handler<typelevel::IPC> for IpcInterruptHandler {
    unsafe fn on_interrupt() {
        let r = pac::IPC;
        if r.events_receive(IPC_CH_FROM_APP).read() != 0 {
            r.events_receive(IPC_CH_FROM_APP).write_value(0);
            IPC_WAKER.wake();
        }
    }
}

fn setup_ipc() {
    let r = pac::IPC;
    // Event n <-> channel n in both directions keeps the mapping obvious.
    r.receive_cnf(IPC_CH_FROM_APP).write(|w| w.0 = 1 << IPC_CH_FROM_APP);
    r.send_cnf(IPC_CH_TO_APP).write(|w| w.0 = 1 << IPC_CH_TO_APP);
    r.events_receive(IPC_CH_FROM_APP).write_value(0);
    r.intenset().write(|w| w.0 = 1 << IPC_CH_FROM_APP);
    typelevel::IPC::unpend();
    unsafe { typelevel::IPC::enable() };
}

fn notify_app() {
    pac::IPC.tasks_send(IPC_CH_TO_APP).write_value(1);
}

/// Forward HCI traffic forever.
pub async fn run(shm: &'static Shared, sdc: &SoftdeviceController<'_>) -> ! {
    setup_ipc();
    let mut rec = [0u8; MAX_RECORD];
    let mut evt = [0u8; 260];
    // Controller output; large enough for ISO SDUs too.
    let mut out = [0u8; raw::HCI_MSG_BUFFER_MAX_SIZE as usize + 256];
    // A controller packet that did not fit into the ring yet.
    let mut stalled: Option<(u8, usize)> = None;

    poll_fn(|cx| {
        // Register before draining so nothing that arrives during the drain is missed.
        IPC_WAKER.register(cx.waker());
        sdc.register_waker(cx.waker());
        let mut signalled = false;

        loop {
            let mut progress = false;

            // Controller -> host. Never consume a host record whose reply
            // might not fit while controller output is stalled.
            if let Some((ind, len)) = stalled {
                if !shm.to_app.try_push(ind, &[&out[..len]]) {
                    break;
                }
                stalled = None;
                signalled = true;
                progress = true;
            }
            let mut ty = 0u8;
            // SAFETY: `out` is at least HCI_MSG_BUFFER_MAX_SIZE bytes.
            if unsafe { raw::sdc_hci_get(out.as_mut_ptr(), &mut ty) } == 0 {
                let ind = match ty as u32 {
                    raw::SDC_HCI_MSG_TYPE_EVT => H4_EVT,
                    raw::SDC_HCI_MSG_TYPE_DATA => H4_ACL,
                    raw::SDC_HCI_MSG_TYPE_ISO => H4_ISO,
                    other => panic!("unknown SDC message type {}", other),
                };
                let len = hci_shm::packet_len(ind, &out).unwrap();
                if shm.to_app.try_push(ind, &[&out[..len]]) {
                    signalled = true;
                } else {
                    stalled = Some((ind, len));
                }
                progress = true;
            }

            // Host -> controller. A command reply needs ring space; check first.
            if stalled.is_none() && shm.to_app.fits(1 + evt.len()) {
                if let Some(n) = shm.to_net.try_pop(&mut rec) {
                    signalled = true; // freed space in to_net
                    progress = true;
                    let (ind, pkt) = (rec[0], &rec[1..n]);
                    match ind {
                        H4_CMD => {
                            debug!("HCI cmd {=u16:#06x}", u16::from_le_bytes([pkt[0], pkt[1]]));
                            if let Some(len) = run_command(pkt, &mut evt) {
                                assert!(shm.to_app.try_push(H4_EVT, &[&evt[..len]]));
                            }
                        }
                        H4_ACL => {
                            if let Err(e) = sdc.hci_data_put(pkt) {
                                warn!("HCI: ACL rejected: {}", e);
                            }
                        }
                        H4_ISO => {
                            if let Err(e) = sdc.hci_iso_data_put(pkt) {
                                debug!("HCI: ISO SDU rejected: {}", e);
                            }
                        }
                        other => warn!("HCI: dropping record with indicator {=u8}", other),
                    }
                }
            }

            if !progress {
                break;
            }
        }
        if signalled {
            notify_app();
        }
        Poll::<()>::Pending
    })
    .await;
    unreachable!()
}
