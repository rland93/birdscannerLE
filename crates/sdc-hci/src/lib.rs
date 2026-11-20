//! Raw HCI access to Nordic's SoftDevice Controller (SDC).
//!
//! The SDC exposes one C function per HCI command instead of a "put raw
//! command" entry point (sdk-nrf's `hci_internal.c` fills that role in C).
//! [`run_command`] maps opcodes to those functions, whose parameter and return
//! structs are packed and identical to the HCI wire format, and builds the
//! Command Complete/Status event. [`try_get`] fetches controller output
//! including ISO data, which nrf-sdc's own `Controller` impl does not handle.
//!
//! Used by the nRF5340 network core (`netcore/`, HCI over shared RAM) and the
//! nRF52840 receiver dongle (`dongle/`, host on the same chip).

#![no_std]

mod dispatch;

use core::mem::size_of;

use hci_shm::{H4_ACL, H4_EVT, H4_ISO};
use nrf_sdc::raw;

pub use hci_shm::packet_len;

/// Size of the buffer [`run_command`] writes its event into.
pub const EVENT_BUF: usize = 260;
/// Size of the buffer [`try_get`] needs (largest HCI packet incl. ISO SDUs).
pub const GET_BUF: usize = raw::HCI_MSG_BUFFER_MAX_SIZE as usize + 256;

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
///
/// The SDC must be enabled (nrf-sdc `Builder::build`) before calling this.
pub fn run_command(pkt: &[u8], evt: &mut [u8; EVENT_BUF]) -> Option<usize> {
    let opcode = u16::from_le_bytes([pkt[0], pkt[1]]);
    // Zero-padded, max-size parameter block: SDC functions read fixed-size
    // structs (and trailing flexible arrays) straight from it.
    let mut params = [0u8; 256];
    let plen = (pkt[2] as usize).min(pkt.len() - 3);
    params[..plen].copy_from_slice(&pkt[3..3 + plen]);
    let mut ret = [0u8; 255];

    // SAFETY: `params` is 256 readable bytes; `ret` is large enough for every return struct.
    let done = unsafe { dispatch::dispatch(opcode, params.as_ptr(), &mut ret) };
    let [op_lo, op_hi] = opcode.to_le_bytes();
    let Some(done) = done else {
        defmt::warn!("HCI: unsupported opcode {=u16:#06x}", opcode);
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

/// Fetch one packet from the controller into `out`. Returns its H4 indicator
/// and length (without indicator), or `None` if nothing is pending. Register a
/// waker with `SoftdeviceController::register_waker` before calling to be
/// woken when more arrives.
pub fn try_get(out: &mut [u8; GET_BUF]) -> Option<(u8, usize)> {
    let mut ty = 0u8;
    // SAFETY: `out` is at least HCI_MSG_BUFFER_MAX_SIZE bytes.
    if unsafe { raw::sdc_hci_get(out.as_mut_ptr(), &mut ty) } != 0 {
        return None;
    }
    let ind = match ty as u32 {
        raw::SDC_HCI_MSG_TYPE_EVT => H4_EVT,
        raw::SDC_HCI_MSG_TYPE_DATA => H4_ACL,
        raw::SDC_HCI_MSG_TYPE_ISO => H4_ISO,
        other => panic!("unknown SDC message type {}", other),
    };
    let len = packet_len(ind, &out[..]).unwrap();
    Some((ind, len))
}
