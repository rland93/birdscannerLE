//! Band energies of the spectrum (liblc3 `energy.c`).

use crate::common::{DT_2M5, DT_5M, DT_7M5};
use crate::tables::{LC3_BAND_LIM, LC3_NUM_BANDS};

/// `lc3_energy_compute`: the mean square per band into `e`; returns the near-Nyquist flag.
pub(crate) fn compute(dt: usize, sr: usize, x: &[f32], e: &mut [f32]) -> bool {
    let nb = LC3_NUM_BANDS[dt][sr];
    let lim = LC3_BAND_LIM[dt][sr].expect("bands for (dt, sr)");
    let mut e_sum = [0f32; 2];
    let iband_h = nb
        - match dt {
            DT_2M5 => 2,
            DT_5M => 3,
            DT_7M5 => 4,
            _ => 2,
        };
    for iband in 0..nb {
        let (i0, ie) = (lim[iband], lim[iband + 1]);
        let mut sx2 = x[i0] * x[i0];
        for &v in &x[i0 + 1..ie] {
            sx2 += v * v;
        }
        e[iband] = sx2 / (ie - i0) as f32;
        e_sum[usize::from(iband >= iband_h)] += e[iband];
    }
    e_sum[1] > 30.0 * e_sum[0]
}
