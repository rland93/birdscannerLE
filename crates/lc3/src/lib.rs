//! Pure-Rust LC3 (Low Complexity Communication Codec, the Bluetooth LE Audio codec): encoder
//! and decoder with packet loss concealment, ported from liblc3 1.1.3 (Apache-2.0; see NOTICE).
//! Bit-exact with liblc3 built without `-ffast-math` (the operation order its source states),
//! so the output is the same on every CPU.
//!
//! One [`Encoder`] or [`Decoder`] per channel, set up for a frame duration (7.5 or 10 ms; also
//! liblc3's 2.5 and 5 ms), a coded sample rate (8–48 kHz) and a PCM rate at or above it. A
//! frame is 20 to [`MAX_FRAME_BYTES`] bytes: its size sets the bitrate. A lost frame is decoded
//! as `None` and concealed.
//!
//! ```
//! use lc3::{Decoder, Encoder, Status};
//!
//! // 10 ms frames at 48 kHz, 96 kbit/s: 120-byte frames of 480 samples
//! let mut enc = Encoder::new(10_000, 48_000, 48_000)?;
//! let mut dec = Decoder::new(10_000, 48_000, 48_000)?;
//! let pcm = vec![0i16; enc.frame_samples()];
//! let mut frame = [0u8; 120];
//! enc.encode(&pcm, &mut frame)?;
//!
//! let mut out = vec![0i16; dec.frame_samples()];
//! assert_eq!(dec.decode(Some(&frame), &mut out)?, Status::Decoded);
//! dec.decode(None, &mut out)?; // a lost frame: concealed
//! # Ok::<(), lc3::Error>(())
//! ```
//!
//! no_std port: this copy of lc3 0.1.0 is `#![no_std]` (it needs `alloc` for the per-channel
//! history buffers), takes `sqrtf`/`truncf`/`floorf` from `libm` (exact, so results are unchanged)
//! and drops the x86-64 SSE2 kernels for their portable bodies (equal by contract). Its
//! on-stack MDCT buffers are sized for 480 samples (10 ms at 48 kHz, the largest frame this
//! crate accepts) instead of liblc3's 960, halving `encode`'s stack use.

#![cfg_attr(not(test), no_std)]

extern crate alloc;

mod attdet;
mod bits;
mod bwdet;
mod common;
mod decoder;
mod encoder;
mod energy;
mod fastmath;
mod ltpf;
mod mdct;
mod plc;
mod sns;
mod spec;
mod tables;
#[cfg(test)]
mod testutil;
mod tns;

pub use decoder::{Decoder, Error, MAX_FRAME_BYTES, MIN_FRAME_BYTES, Status};
pub use encoder::Encoder;
