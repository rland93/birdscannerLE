# lc3 (no_std port)

**This is a modified no_std port of [lc3 0.1.0](https://crates.io/crates/lc3)** for the
nrf53bird firmware: `#![no_std]` with `alloc`, `sqrtf`/`truncf`/`floorf` from `libm` (exact
operations, so results are unchanged), the x86-64 SSE2 kernels (`src/simd.rs`) removed in
favour of their portable bodies, and the on-stack MDCT buffers sized for 480 samples (the
largest frame accepted, 10 ms at 48 kHz) instead of 960. `tests/upstream.rs` checks the encoder
and decoder output byte-for-byte against the unmodified crates.io release. The upstream README
follows.

Pure-Rust LC3 (Low Complexity Communication Codec, the Bluetooth LE Audio codec): encoder and
decoder with packet loss concealment, ported from Google's liblc3 1.1.3 (Apache-2.0). Its
output is bit-exact with liblc3's, on every CPU. Only dependency: `libm`; no `unsafe`.

```rust
use lc3::{Decoder, Encoder, Status};

// 10 ms frames at 48 kHz, 96 kbit/s: 120-byte frames of 480 samples
let mut enc = Encoder::new(10_000, 48_000, 48_000)?;
let mut dec = Decoder::new(10_000, 48_000, 48_000)?;
let pcm = vec![0i16; enc.frame_samples()];
let mut frame = [0u8; 120];
enc.encode(&pcm, &mut frame)?;

let mut out = vec![0i16; dec.frame_samples()];
assert_eq!(dec.decode(Some(&frame), &mut out)?, Status::Decoded);
dec.decode(None, &mut out)?; // a lost frame: concealed
# Ok::<(), lc3::Error>(())
```

Frame durations 7.5 and 10 ms (LC3) and liblc3's 2.5 and 5 ms; sample rates 8, 16, 24, 32
and 48 kHz; PCM at the coded rate or above (e.g. 48 kHz PCM coded at 16 kHz); 16-bit or float
samples; frames of 20 to 400 bytes (any bitrate the frame size gives).

## Results
Measured, with the recipe that reproduces each number, in
[`docs/RESULTS.md`](https://github.com/jj2218/lc3-rs/blob/main/docs/RESULTS.md).

- **Bit-exact with liblc3** (built without `-ffast-math`, the order its source states): the
  decoder on every Bluetooth SIG conformance operating point, every frame duration, 48 kHz
  output and burst losses; the encoder byte-identical on the same grid. On x86-64 and aarch64.
- **Faster than liblc3 as distributions build it** (`-O3 -ffast-math`, LTO) at 48 kHz, on par
  at 16 kHz, on x86-64 without AVX2; faster than liblc3's exact build everywhere.
- **Fuzzed against liblc3**: the decoder on random and corrupted frames, the encoder on random
  PCM, both formats.

## Scope
Single channel (a stream per channel, as liblc3). No LC3plus high-resolution mode. Not
qualified by the Bluetooth SIG (the SIG's test software is members-only); its conformance
operating points are checked through liblc3, which passes them. Needs Rust 1.88 or later.

## Patents
LC3 is covered by patents of its developers and others, several in force into the late 2030s.
This crate's Apache-2.0 licence, like liblc3's, grants no rights under any third party's
patents. Bluetooth SIG members' qualified Bluetooth products are licensed royalty-free through
the SIG's Patent/Copyright License Agreement; any other use (outside Bluetooth, or by
non-members) may need licences from the patent holders (Fraunhofer IIS offers licences for its
LC3/LC3plus patents; Ericsson has declared LC3plus patents under FRAND terms at ETSI). This is
not legal advice.

## License
Apache-2.0 ([LICENSE](https://github.com/jj2218/lc3-rs/blob/main/crates/lc3/LICENSE)).
Derived from liblc3 1.1.3 (Apache-2.0); see
[NOTICE](https://github.com/jj2218/lc3-rs/blob/main/crates/lc3/NOTICE).
