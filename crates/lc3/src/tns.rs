//! Temporal noise shaping (liblc3 `tns.c`). Analysis (encoder): LPC from the normalised
//! autocorrelation of spectrum subdivisions (Levinson-Durbin), weighting, reflection
//! coefficients quantised, lattice forward filtering. Synthesis (decoder): reflection
//! coefficients from the bitstream, lattice inverse filtering. One or two frequency ranges.

use crate::bits::{Bits, Writer};
use crate::common::{DT_2M5, DT_5M, DT_7M5, SRATE_48K, ne};
use crate::tables::{
    LC3_TNS_COEFFS_BITS, LC3_TNS_COEFFS_MODELS, LC3_TNS_ORDER_BITS, LC3_TNS_ORDER_MODELS,
    TNS_LAG_WINDOW, TNS_Q_INV,
};

/// `LC3_BANDWIDTH_SWB`.
const BANDWIDTH_SWB: usize = 3;

/// `lc3_tns_data_t`.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub(crate) struct TnsData {
    pub nfilters: usize,
    pub lpc_weighting: bool,
    pub rc_order: [usize; 2],
    pub rc: [[i32; 8]; 2],
}

/// `resolve_lpc_weighting`.
fn resolve_lpc_weighting(dt: usize, nbytes: i32) -> bool {
    nbytes * 8 < 120 * (1 + dt as i32)
}

/// `unquantize_rc`.
fn unquantize_rc(rc_q: &[i32; 8], order: usize, rc: &mut [f32; 8]) {
    for i in 0..order {
        let rc_m = TNS_Q_INV[rc_q[i].unsigned_abs() as usize];
        rc[i] = if rc_q[i] < 0 { -rc_m } else { rc_m };
    }
}

/// `inverse_filtering`.
fn inverse_filtering(
    dt: usize,
    bw: usize,
    rc_order: &[usize; 2],
    rc: &[[f32; 8]; 2],
    x: &mut [f32],
) {
    let nfilters = 1 + usize::from(dt >= DT_5M && bw >= BANDWIDTH_SWB);
    let nf = ne(dt, bw.min(SRATE_48K)) >> (nfilters - 1);
    let mut ie = 3 * (1 + dt);
    let mut s = [0f32; 8];
    for f in 0..nfilters {
        let i0 = ie;
        ie = nf * (1 + f);
        if rc_order[f] == 0 {
            continue;
        }
        let rc = &rc[f];
        for xi_ref in &mut x[i0..ie] {
            let mut xi = *xi_ref;
            xi -= s[7] * rc[7];
            for k in (0..7).rev() {
                xi -= s[k] * rc[k];
                s[k + 1] = s[k] + rc[k] * xi;
            }
            s[0] = xi;
            *xi_ref = xi;
        }
        s[rc_order[f]..].fill(0.0);
    }
}

/// `lc3_tns_synthesize`.
pub(crate) fn synthesize(dt: usize, bw: usize, data: &TnsData, x: &mut [f32]) {
    let mut rc = [[0f32; 8]; 2];
    for f in 0..data.nfilters {
        if data.rc_order[f] != 0 {
            unquantize_rc(&data.rc[f], data.rc_order[f], &mut rc[f]);
        }
    }
    inverse_filtering(dt, bw, &data.rc_order, &rc, x);
}

/// `lc3_tns_get_data`; `Err` on an order above 4 at 2.5 or 5 ms.
pub(crate) fn get_data(bits: &mut Bits, dt: usize, bw: usize, nbytes: i32) -> Result<TnsData, ()> {
    let mut data = TnsData {
        nfilters: 1 + usize::from(dt >= DT_5M && bw >= BANDWIDTH_SWB),
        lpc_weighting: resolve_lpc_weighting(dt, nbytes),
        ..TnsData::default()
    };
    for f in 0..data.nfilters {
        data.rc_order[f] = bits.get_bit() as usize;
        if data.rc_order[f] == 0 {
            continue;
        }
        data.rc_order[f] +=
            bits.get_symbol(&LC3_TNS_ORDER_MODELS[usize::from(data.lpc_weighting)]) as usize;
        if dt <= DT_5M && data.rc_order[f] > 4 {
            return Err(());
        }
        for i in 0..data.rc_order[f] {
            data.rc[f][i] = bits.get_symbol(&LC3_TNS_COEFFS_MODELS[i]) as i32 - 8;
        }
    }
    Ok(data)
}

/// `dot(x + xs, x + xs + k, (xe - xs) - k)` for every lag k = 0..=8: one chain per lane,
/// each in the C's order (i ascending), so each equals the C's `dot`; while every lag is
/// active the 8 first share a vector (latency-bound chains, 9 at the price of one).
fn lags(x: &[f32], xs: usize, xe: usize) -> [f32; 9] {
    let (mut v, mut v8) = ([0f32; 8], 0f32);
    let mid = xe.saturating_sub(8).max(xs);
    for i in xs..mid {
        let w: &[f32; 8] = x[i..i + 8].try_into().unwrap();
        for k in 0..8 {
            v[k] += x[i] * w[k];
        }
        v8 += x[i] * x[i + 8];
    }
    for i in mid..xe {
        for k in 0..(xe - i).min(8) {
            v[k] += x[i] * x[i + k];
        }
    }
    let mut c = [0f32; 9];
    c[..8].copy_from_slice(&v);
    c[8] = v8;
    c
}

/// `compute_lpc_coeffs`: per filter, the prediction gain and LPC coefficients `a[f][..=maxorder]`.
fn compute_lpc_coeffs(
    dt: usize,
    bw: usize,
    maxorder: usize,
    x: &[f32],
    gain: &mut [f32; 2],
    a: &mut [[f32; 9]; 2],
) {
    // [dt][bw]: subdivision limits (2 or 3 per filter)
    const SUB_2M5: [&[usize]; 5] = [
        &[3, 10, 20],
        &[3, 20, 40],
        &[3, 30, 60],
        &[3, 40, 80],
        &[3, 51, 100],
    ];
    const SUB_5M: [&[usize]; 5] = [
        &[6, 23, 40],
        &[6, 43, 80],
        &[6, 63, 120],
        &[6, 43, 80, 120, 160],
        &[6, 53, 100, 150, 200],
    ];
    const SUB_7M5: [&[usize]; 5] = [
        &[9, 26, 43, 60],
        &[9, 46, 83, 120],
        &[9, 66, 123, 180],
        &[9, 46, 82, 120, 159, 200, 240],
        &[9, 56, 103, 150, 200, 250, 300],
    ];
    const SUB_10M: [&[usize]; 5] = [
        &[12, 34, 57, 80],
        &[12, 61, 110, 160],
        &[12, 88, 164, 240],
        &[12, 61, 110, 160, 213, 266, 320],
        &[12, 74, 137, 200, 266, 333, 400],
    ];
    let sub = match dt {
        DT_2M5 => SUB_2M5,
        DT_5M => SUB_5M,
        DT_7M5 => SUB_7M5,
        _ => SUB_10M,
    }[bw.min(4)];

    // Normalised autocorrelation
    let nfilters = 1 + usize::from(dt >= DT_5M && bw >= BANDWIDTH_SWB);
    let nsubdivisions = 2 + usize::from(dt >= DT_7M5);
    let mut r = [[0f32; 9]; 2];
    let mut isub = 0;
    for rf in r.iter_mut().take(nfilters) {
        let mut c = [[0f32; 3]; 9];
        for s in 0..nsubdivisions {
            let (xs, xe) = (sub[isub], sub[isub + 1]);
            isub += 1;
            let l = lags(x, xs, xe);
            for (k, ck) in c.iter_mut().enumerate().take(maxorder + 1) {
                ck[s] = l[k];
            }
        }
        rf[0] = nsubdivisions as f32;
        if nsubdivisions == 2 {
            let (e0, e1) = (c[0][0], c[0][1]);
            for k in 1..=maxorder {
                rf[k] = if e0 == 0.0 || e1 == 0.0 {
                    0.0
                } else {
                    (c[k][0] / e0 + c[k][1] / e1) * TNS_LAG_WINDOW[k]
                };
            }
        } else {
            let (e0, e1, e2) = (c[0][0], c[0][1], c[0][2]);
            for k in 1..=maxorder {
                rf[k] = if e0 == 0.0 || e1 == 0.0 || e2 == 0.0 {
                    0.0
                } else {
                    (c[k][0] / e0 + c[k][1] / e1 + c[k][2] / e2) * TNS_LAG_WINDOW[k]
                };
            }
        }
    }

    // Levinson-Durbin recursion, two orders per step
    for f in 0..nfilters {
        let r = &r[f];
        let a0 = &mut a[f];
        let mut a1 = [0f32; 9];
        let mut err = r[0];
        gain[f] = err;
        a0[0] = 1.0;
        let mut k = 1;
        while k <= maxorder {
            let mut rc = -r[k];
            for i in 1..k {
                rc -= a0[i] * r[k - i];
            }
            rc /= err;
            err *= 1.0 - rc * rc;
            for i in 1..k {
                a1[i] = a0[i] + rc * a0[k - i];
            }
            a1[k] = rc;
            k += 1;

            let mut rc = -r[k];
            for i in 1..k {
                rc -= a1[i] * r[k - i];
            }
            rc /= err;
            err *= 1.0 - rc * rc;
            for i in 1..k {
                a0[i] = a1[i] + rc * a1[k - i];
            }
            a0[k] = rc;
            k += 1;
        }
        gain[f] /= err;
    }
}

/// `lpc_weighting`: bandwidth expansion by the prediction gain.
fn lpc_weighting(pred_gain: f32, a: &mut [f32; 9]) {
    let gamma = 1.0 - (1.0f32 - 0.85) * (2.0 - pred_gain) / (2.0 - 1.5);
    let mut g = 1f32;
    for v in &mut a[1..] {
        g *= gamma;
        *v *= g;
    }
}

/// `lpc_reflection`: the reflection coefficients of `a`.
fn lpc_reflection(a: &[f32; 9], maxorder: usize, rc: &mut [f32; 8]) {
    let mut b = [[0f32; 7]; 2];
    rc[maxorder - 1] = a[maxorder];
    let mut e = 1.0 - rc[maxorder - 1] * rc[maxorder - 1];
    for i in 0..maxorder - 1 {
        b[1][i] = (a[1 + i] - rc[maxorder - 1] * a[(maxorder - 1) - i]) / e;
    }
    let mut i1 = 1;
    for k in (1..=maxorder - 2).rev() {
        let i0 = i1;
        i1 = k & 1;
        rc[k] = b[i0][k];
        e = 1.0 - rc[k] * rc[k];
        for i in 0..k {
            b[i1][i] = (b[i0][i] - rc[k] * b[i0][k - 1 - i]) / e;
        }
    }
    rc[0] = b[i1][0];
}

/// `quantize_rc`: indices of `rc` on the arcsine grid; returns the order (trailing zeros
/// dropped).
fn quantize_rc(rc: &[f32; 8], maxorder: usize, rc_q: &mut [i32; 8]) -> usize {
    // sin(pi/17 · (i + 0.5)) in Q15
    const Q_THR: [u16; 8] = [
        0x0bcf, 0x2307, 0x390e, 0x4d23, 0x5e98, 0x6cd4, 0x775b, 0x7dd2,
    ];
    let thr = |i: i32| f32::from(Q_THR[i as usize]) / 32768.0;
    let mut order = maxorder;
    for i in 0..maxorder {
        let rc_m = rc[i].abs();
        let mut q = 4 * i32::from(rc_m >= thr(4));
        let mut j = 0;
        while j < 4 && rc_m >= thr(q) {
            j += 1;
            q += 1;
        }
        rc_q[i] = if rc[i] < 0.0 { -q } else { q };
        order = if rc_q[i] != 0 { maxorder } else { order - 1 };
    }
    order
}

/// `forward_filtering`: the lattice analysis filter over the one or two ranges.
fn forward_filtering(
    dt: usize,
    bw: usize,
    rc_order: &[usize; 2],
    rc: &[[f32; 8]; 2],
    x: &mut [f32],
) {
    let nfilters = 1 + usize::from(dt >= DT_5M && bw >= BANDWIDTH_SWB);
    let nf = ne(dt, bw.min(SRATE_48K)) >> (nfilters - 1);
    let mut ie = 3 * (1 + dt);
    let mut s = [0f32; 8];
    for f in 0..nfilters {
        let i0 = ie;
        ie = nf * (1 + f);
        if rc_order[f] == 0 {
            continue;
        }
        let rc = &rc[f];
        for xi_ref in &mut x[i0..ie] {
            let mut xi = *xi_ref;
            let mut s1 = xi;
            for k in 0..rc_order[f] {
                let s0 = s[k];
                s[k] = s1;
                s1 = rc[k] * xi + s0;
                xi += rc[k] * s0;
            }
            *xi_ref = xi;
        }
    }
}

/// `lc3_tns_analyze`: the filters of the spectrum `x` (filtered in place).
pub(crate) fn analyze(dt: usize, bw: usize, nn_flag: bool, nbytes: i32, x: &mut [f32]) -> TnsData {
    let mut data = TnsData {
        nfilters: 1 + usize::from(dt >= DT_5M && bw >= BANDWIDTH_SWB),
        lpc_weighting: resolve_lpc_weighting(dt, nbytes),
        ..TnsData::default()
    };
    let maxorder = if dt <= DT_5M { 4 } else { 8 };
    let (mut pred_gain, mut a, mut rc) = ([0f32; 2], [[0f32; 9]; 2], [[0f32; 8]; 2]);
    compute_lpc_coeffs(dt, bw, maxorder, x, &mut pred_gain, &mut a);
    for f in 0..data.nfilters {
        data.rc_order[f] = 0;
        if nn_flag || pred_gain[f] <= 1.5 {
            continue;
        }
        if data.lpc_weighting && pred_gain[f] < 2.0 {
            lpc_weighting(pred_gain[f], &mut a[f]);
        }
        lpc_reflection(&a[f], maxorder, &mut rc[f]);
        data.rc_order[f] = quantize_rc(&rc[f], maxorder, &mut data.rc[f]);
        unquantize_rc(&data.rc[f], data.rc_order[f], &mut rc[f]);
    }
    forward_filtering(dt, bw, &data.rc_order, &rc, x);
    data
}

/// `lc3_tns_get_nbits`.
pub(crate) fn get_nbits(data: &TnsData) -> i32 {
    let mut nbits = 0;
    for f in 0..data.nfilters {
        let rc_order = data.rc_order[f];
        let mut nbits_2048 = 2048;
        if rc_order > 0 {
            nbits_2048 +=
                i32::from(LC3_TNS_ORDER_BITS[usize::from(data.lpc_weighting)][rc_order - 1]);
        }
        for i in 0..rc_order {
            nbits_2048 += i32::from(LC3_TNS_COEFFS_BITS[i][(8 + data.rc[f][i]) as usize]);
        }
        nbits += (nbits_2048 + (1 << 11) - 1) >> 11;
    }
    nbits
}

/// `lc3_tns_put_data`.
pub(crate) fn put_data(bits: &mut Writer, data: &TnsData) {
    for f in 0..data.nfilters {
        let rc_order = data.rc_order[f];
        bits.put_bit(rc_order > 0);
        if rc_order == 0 {
            continue;
        }
        bits.put_symbol(
            &LC3_TNS_ORDER_MODELS[usize::from(data.lpc_weighting)],
            rc_order - 1,
        );
        for i in 0..rc_order {
            bits.put_symbol(&LC3_TNS_COEFFS_MODELS[i], (8 + data.rc[f][i]) as usize);
        }
    }
}
