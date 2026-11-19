//! Frame geometry (liblc3 `tables.h`, `common.h`): durations and rates as the C enums' indices.

use crate::tables::{LC3_NE_2M5, LC3_NS_2M5, LC3_NS_4M};

/// `enum lc3_dt`: 2.5, 5, 7.5 and 10 ms.
pub(crate) const DT_2M5: usize = 0;
pub(crate) const DT_5M: usize = 1;
pub(crate) const DT_7M5: usize = 2;
pub(crate) const DT_10M: usize = 3;
/// `enum lc3_srate` (and `enum lc3_bandwidth`): 8, 16, 24, 32, 48 kHz, then high resolution.
pub(crate) const SRATE_48K: usize = 4;
pub(crate) const SRATE_48K_HR: usize = 5;

/// The most samples per frame this crate handles: 10 ms at 48 kHz. (liblc3's `LC3_MAX_NS` is
/// 960, 10 ms at 96 kHz, but the high-resolution rates are rejected by `resolve_srate`; the
/// no_std port sizes its on-stack MDCT buffers by this to halve their stack use.)
pub(crate) const MAX_NS: usize = 480;

/// `lc3_hr`: a high-resolution rate.
pub(crate) fn hr(sr: usize) -> bool {
    sr >= SRATE_48K_HR
}

/// `lc3_ns`: samples (and coefficients) per frame.
pub(crate) fn ns(dt: usize, sr: usize) -> usize {
    LC3_NS_2M5[sr] * (1 + dt)
}

/// `lc3_ne`: encoded coefficients per frame.
pub(crate) fn ne(dt: usize, sr: usize) -> usize {
    LC3_NE_2M5[sr] * (1 + dt)
}

/// `lc3_nd`: MDCT delayed samples (half a frame plus the 1.25 ms, or 2 ms at 7.5 ms, overlap).
pub(crate) fn nd(dt: usize, sr: usize) -> usize {
    (ns(dt, sr)
        + if dt == DT_7M5 {
            LC3_NS_4M[sr]
        } else {
            LC3_NS_2M5[sr]
        })
        >> 1
}

/// `lc3_nh`: 18 ms of history, aligned on a frame (none at high resolution).
pub(crate) fn nh(dt: usize, sr: usize) -> usize {
    if sr > SRATE_48K_HR {
        0
    } else {
        (8 + usize::from(dt == DT_7M5)) * LC3_NS_2M5[sr]
    }
}
