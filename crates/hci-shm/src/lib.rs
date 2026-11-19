//! HCI transport between the nRF5340 application core (host) and network
//! core (controller) over shared RAM.
//!
//! Two single-producer/single-consumer byte rings live at [`SHARED_ADDR`] in
//! application-core RAM, which both cores can address. Each record is
//! `[len_lo, len_hi, h4_indicator, packet...]` where `len` counts the
//! indicator plus packet. Records are only published once complete, so a
//! reader never sees a partial packet.
//!
//! Start-up: the application core zeroes the region, writes [`MAGIC`] and only
//! then releases the network core. The network core waits for the magic.
//!
//! Signalling is not part of this crate: after pushing or popping, each side
//! triggers an IPC event so the peer re-polls (see the firmware crates).

#![no_std]

use core::cell::UnsafeCell;
use core::sync::atomic::{AtomicU32, Ordering, fence};

/// Start of the shared region: the last 16 KiB of application-core RAM.
/// The app core's linker script must stop RAM below this address.
pub const SHARED_ADDR: usize = 0x2007_C000;
pub const SHARED_SIZE: usize = 16 * 1024;
pub const MAGIC: u32 = 0x4843_4931; // "HCI1"

/// H4 packet indicators.
pub const H4_CMD: u8 = 0x01;
pub const H4_ACL: u8 = 0x02;
pub const H4_EVT: u8 = 0x04;
pub const H4_ISO: u8 = 0x05;

const RING_BYTES: usize = 8 * 1024 - 64;
const HEADER: usize = 2;

/// Upper bound of `indicator + packet` the rings carry.
pub const MAX_RECORD: usize = 1 + 4 + 1024;

#[repr(C)]
pub struct Ring {
    /// Free-running byte count written by the producer.
    head: AtomicU32,
    /// Free-running byte count consumed by the consumer.
    tail: AtomicU32,
    buf: UnsafeCell<[u8; RING_BYTES]>,
}

// SAFETY: SPSC protocol; producer only writes free space before publishing
// `head`, consumer only reads published bytes before publishing `tail`.
unsafe impl Sync for Ring {}

impl Ring {
    fn used(&self) -> usize {
        self.head.load(Ordering::Acquire).wrapping_sub(self.tail.load(Ordering::Acquire)) as usize
    }

    fn copy_in(&self, at: u32, data: &[u8]) {
        let buf = self.buf.get() as *mut u8;
        let start = at as usize % RING_BYTES;
        let first = data.len().min(RING_BYTES - start);
        // SAFETY: indices are within the ring; the region is free space owned by the producer.
        unsafe {
            core::ptr::copy_nonoverlapping(data.as_ptr(), buf.add(start), first);
            core::ptr::copy_nonoverlapping(data.as_ptr().add(first), buf, data.len() - first);
        }
    }

    fn copy_out(&self, at: u32, out: &mut [u8]) {
        let buf = self.buf.get() as *const u8;
        let start = at as usize % RING_BYTES;
        let first = out.len().min(RING_BYTES - start);
        // SAFETY: indices are within the ring; the region is published data owned by the consumer.
        unsafe {
            core::ptr::copy_nonoverlapping(buf.add(start), out.as_mut_ptr(), first);
            core::ptr::copy_nonoverlapping(buf, out.as_mut_ptr().add(first), out.len() - first);
        }
    }

    /// Producer: append one record built from `indicator` and `parts`
    /// (concatenated). Returns false, writing nothing, if it does not fit.
    pub fn try_push(&self, indicator: u8, parts: &[&[u8]]) -> bool {
        let len = 1 + parts.iter().map(|p| p.len()).sum::<usize>();
        assert!(len <= MAX_RECORD, "HCI record too large");
        if RING_BYTES - self.used() < HEADER + len {
            return false;
        }
        let mut at = self.head.load(Ordering::Relaxed);
        self.copy_in(at, &(len as u16).to_le_bytes());
        at = at.wrapping_add(HEADER as u32);
        self.copy_in(at, &[indicator]);
        at = at.wrapping_add(1);
        for p in parts {
            self.copy_in(at, p);
            at = at.wrapping_add(p.len() as u32);
        }
        fence(Ordering::Release);
        self.head.store(at, Ordering::Release);
        true
    }

    /// Consumer: pop one record into `out` as `[indicator, packet...]`.
    /// Returns the record length, or `None` if the ring is empty.
    pub fn try_pop(&self, out: &mut [u8]) -> Option<usize> {
        if self.used() == 0 {
            return None;
        }
        fence(Ordering::Acquire);
        let at = self.tail.load(Ordering::Relaxed);
        let mut hdr = [0u8; HEADER];
        self.copy_out(at, &mut hdr);
        let len = u16::from_le_bytes(hdr) as usize;
        assert!(len <= out.len(), "HCI record larger than pop buffer");
        self.copy_out(at.wrapping_add(HEADER as u32), &mut out[..len]);
        self.tail.store(at.wrapping_add((HEADER + len) as u32), Ordering::Release);
        Some(len)
    }

    /// Bytes a producer could still push, including record headers.
    pub fn free(&self) -> usize {
        RING_BYTES - self.used()
    }

    /// Whether a record of `len` bytes (indicator + packet) fits right now.
    pub fn fits(&self, len: usize) -> bool {
        self.free() >= HEADER + len
    }

    pub fn is_empty(&self) -> bool {
        self.used() == 0
    }
}

#[repr(C)]
pub struct Shared {
    pub magic: AtomicU32,
    /// Host -> controller.
    pub to_net: Ring,
    /// Controller -> host.
    pub to_app: Ring,
}

const _: () = assert!(size_of::<Shared>() <= SHARED_SIZE);

impl Shared {
    /// # Safety
    /// Only valid on the nRF5340 where [`SHARED_ADDR`] is reserved for this.
    pub unsafe fn get() -> &'static Shared {
        unsafe { &*(SHARED_ADDR as *const Shared) }
    }

    /// App core, before releasing the network core: clear rings, publish magic.
    ///
    /// # Safety
    /// The network core must not be running.
    pub unsafe fn init() -> &'static Shared {
        unsafe { core::ptr::write_bytes(SHARED_ADDR as *mut u8, 0, size_of::<Shared>()) };
        fence(Ordering::SeqCst);
        let s = unsafe { Self::get() };
        s.magic.store(MAGIC, Ordering::Release);
        s
    }

    pub fn ready(&self) -> bool {
        self.magic.load(Ordering::Acquire) == MAGIC
    }
}

/// Total packet length (excluding indicator) from the start of an H4 packet,
/// or `None` if the indicator is unknown or the header is incomplete.
pub fn packet_len(indicator: u8, pkt: &[u8]) -> Option<usize> {
    Some(match indicator {
        H4_CMD => 3 + *pkt.get(2)? as usize,
        H4_ACL => 4 + u16::from_le_bytes([*pkt.get(2)?, *pkt.get(3)?]) as usize,
        H4_EVT => 2 + *pkt.get(1)? as usize,
        H4_ISO => 4 + (u16::from_le_bytes([*pkt.get(2)?, *pkt.get(3)?]) & 0x3fff) as usize,
        _ => return None,
    })
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use std::boxed::Box;

    fn ring() -> Box<Ring> {
        Box::new(Ring { head: AtomicU32::new(0), tail: AtomicU32::new(0), buf: UnsafeCell::new([0; RING_BYTES]) })
    }

    #[test]
    fn records_survive_wraparound_and_full_ring() {
        let r = ring();
        let mut out = [0u8; MAX_RECORD];
        let payload: std::vec::Vec<u8> = (0..300u32).map(|i| i as u8).collect();
        let mut pushed = 0u32;
        let mut popped = 0u32;
        // Interleave so head/tail wrap the ring many times.
        for round in 0..200 {
            while r.try_push(H4_ACL, &[&[round as u8], &payload[..(round * 7) % 300]]) {
                pushed += 1;
                if pushed - popped > 5 {
                    break;
                }
            }
            while let Some(n) = r.try_pop(&mut out) {
                assert_eq!(out[0], H4_ACL);
                let round_tag = out[1];
                let body = &out[2..n];
                let expect_len = (round_tag as usize * 7) % 300;
                assert_eq!(body, &payload[..expect_len]);
                popped += 1;
            }
        }
        assert_eq!(pushed, popped);
        assert!(r.is_empty());
    }

    #[test]
    fn push_fails_without_partial_write_when_full() {
        let r = ring();
        let big = [0xAAu8; 1000];
        let mut n = 0;
        while r.try_push(H4_EVT, &[&big]) {
            n += 1;
        }
        assert_eq!(n, RING_BYTES / (HEADER + 1001));
        let mut out = [0u8; MAX_RECORD];
        for _ in 0..n {
            assert_eq!(r.try_pop(&mut out), Some(1001));
        }
        assert_eq!(r.try_pop(&mut out), None);
    }

    #[test]
    fn packet_lengths() {
        assert_eq!(packet_len(H4_EVT, &[0x0e, 4]), Some(6));
        assert_eq!(packet_len(H4_ISO, &[0, 0, 0x28, 0xC0]), Some(4 + 0x28));
        assert_eq!(packet_len(H4_CMD, &[0x03, 0x0c, 0]), Some(3));
    }
}
