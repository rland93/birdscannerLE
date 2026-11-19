//! Heap use of `Encoder::new`/`Decoder::new` (printed with `--nocapture`), and none per frame.

use std::alloc::{GlobalAlloc, Layout, System};
use std::sync::atomic::{AtomicUsize, Ordering::Relaxed};

/// Counts bytes requested through `alloc` (frees are not subtracted: this is the total asked).
struct Counting;

static BYTES: AtomicUsize = AtomicUsize::new(0);
static CALLS: AtomicUsize = AtomicUsize::new(0);

// SAFETY: forwards every call to `System` unchanged.
#[allow(unsafe_code)]
unsafe impl GlobalAlloc for Counting {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        BYTES.fetch_add(layout.size(), Relaxed);
        CALLS.fetch_add(1, Relaxed);
        // SAFETY: the caller's contract, passed on.
        unsafe { System.alloc(layout) }
    }
    unsafe fn alloc_zeroed(&self, layout: Layout) -> *mut u8 {
        BYTES.fetch_add(layout.size(), Relaxed);
        CALLS.fetch_add(1, Relaxed);
        // SAFETY: the caller's contract, passed on.
        unsafe { System.alloc_zeroed(layout) }
    }
    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        // SAFETY: the caller's contract, passed on.
        unsafe { System.dealloc(ptr, layout) }
    }
}

#[global_allocator]
static GLOBAL: Counting = Counting;

/// (bytes, allocations) requested while running `f`.
fn measure<T>(f: impl FnOnce() -> T) -> (T, usize, usize) {
    let (b0, c0) = (BYTES.load(Relaxed), CALLS.load(Relaxed));
    let v = f();
    (v, BYTES.load(Relaxed) - b0, CALLS.load(Relaxed) - c0)
}

/// One test function only: the counters are process-wide and the harness runs tests on threads.
#[test]
fn heap_use() {
    for (frame_us, sr, sr_pcm, bytes) in [
        (10_000, 32_000, 32_000, 80),
        (10_000, 16_000, 32_000, 40),
        (10_000, 16_000, 16_000, 40),
        (10_000, 24_000, 32_000, 60),
        (7_500, 16_000, 16_000, 30),
    ] {
        let (enc, eb, ec) = measure(|| lc3::Encoder::new(frame_us, sr, sr_pcm).unwrap());
        let (dec, db, dc) = measure(|| lc3::Decoder::new(frame_us, sr, sr_pcm).unwrap());
        let (mut enc, mut dec) = (enc, dec);
        let pcm = vec![1234i16; enc.frame_samples()];
        let mut frame = vec![0u8; bytes];
        let mut out = vec![0i16; dec.frame_samples()];
        let ((), fb, _) = measure(|| {
            for _ in 0..10 {
                enc.encode(&pcm, &mut frame).unwrap();
                dec.decode(Some(&frame), &mut out).unwrap();
                dec.decode(None, &mut out).unwrap();
            }
        });
        println!(
            "{frame_us} us {sr}/{sr_pcm} Hz: Encoder::new {eb} B in {ec} allocs, \
             Decoder::new {db} B in {dc} allocs; size_of Encoder {} B, Decoder {} B",
            size_of::<lc3::Encoder>(),
            size_of::<lc3::Decoder>()
        );
        assert_eq!(fb, 0, "encode/decode must not allocate");
    }
}
