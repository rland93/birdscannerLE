//! Long-term post-filter (liblc3 `ltpf.c`). Analysis (encoder): the signal resampled to 12.8
//! and 6.4 kHz, an open-loop pitch search, its fractional refinement and the activation
//! decision. Synthesis (decoder): a pitch-lag IIR over the decoder's history ring buffer, faded
//! in and out across parameter changes.

use crate::bits::{Bits, Writer};
use crate::common::{DT_2M5, DT_5M, DT_7M5, DT_10M, nh, ns};
use crate::tables::{
    LC3_NS_4M, LTPF_H_8K_12K8, LTPF_H_16K_12K8, LTPF_H_24K_12K8, LTPF_H_32K_12K8, LTPF_H_48K_12K8,
    LTPF_H4_CORR, lc3_ltpf_cden, lc3_ltpf_cnum,
};

/// `MAX_FILTER_WIDTH`.
const MAX_FILTER_WIDTH: usize = 12;

/// `lc3_ltpf_data_t`.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub(crate) struct LtpfData {
    pub active: bool,
    pub pitch_index: i32,
}

/// `lc3_ltpf_synthesis_t`.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub(crate) struct LtpfSynthesis {
    pub active: bool,
    pub pitch: i32,
    pub c: [f32; 2 * MAX_FILTER_WIDTH],
    pub x: [f32; MAX_FILTER_WIDTH],
}

/// `lc3_ltpf_analysis_t`.
#[derive(Clone, Debug)]
pub(crate) struct LtpfAnalysis {
    active: bool,
    pitch: i32,
    nc: [f32; 2],
    /// `hp50`: the 50 Hz high-pass biquad state.
    s1: i64,
    s2: i64,
    x_12k8: [i16; 384],
    x_6k4: [i16; 178],
    tc: i32,
}

impl Default for LtpfAnalysis {
    fn default() -> Self {
        LtpfAnalysis {
            active: false,
            pitch: 0,
            nc: [0.0; 2],
            s1: 0,
            s2: 0,
            x_12k8: [0; 384],
            x_6k4: [0; 178],
            tc: 0,
        }
    }
}

impl LtpfAnalysis {
    /// `filter_hp50`: one Q30 sample through the 50 Hz high-pass at 12.8 kHz.
    fn filter_hp50(&mut self, xn: i32) -> i32 {
        const A1: i64 = -2110217691;
        const A2: i64 = 1037111617;
        const B1: i64 = -2110535566;
        const B2: i64 = 1055267782;
        let xn = i64::from(xn);
        let yn = (self.s1.wrapping_add(xn * B2) >> 30) as i32;
        self.s1 = self
            .s2
            .wrapping_add(xn * B1)
            .wrapping_sub(i64::from(yn) * A1);
        self.s2 = (xn * B2).wrapping_sub(i64::from(yn) * A2);
        yn
    }

    /// `resample_x64k_12k8` (`f` = 5, `p` 8, 4, 2) and `resample_x192k_12k8` (`f` = 15,
    /// `p` 8, 4): `n` samples at 12.8 kHz into `y` from `x[xt..]` (Q15, `w - 1` previous).
    #[allow(clippy::too_many_arguments)]
    fn resample(&mut self, f: usize, p: usize, h: &[i16], x: &[i16], xt: usize, y: &mut [i16]) {
        let w = 2 * (8 * f / p);
        let base = xt + 1 - w;
        for (j, yj) in y.iter_mut().enumerate() {
            let i = f * j;
            let hn = &h[(i % p) * w..][..w];
            let xn = &x[base + i / p..][..w];
            let un = dot_wrap(xn, hn);
            let yn = self.filter_hp50(un);
            *yj = (yn.wrapping_add(1 << 15) >> 16) as i16;
        }
    }
}

/// Σ x[k]·h[k], wrapping i32 (the C's `int32_t` sum; wrapping sums are exact in any order).
/// no_std port: upstream's x86-64 SSE2 kernel is dropped; this is its portable contract.
fn dot_wrap(x: &[i16], h: &[i16]) -> i32 {
    let mut un = 0i32;
    for (&a, &b) in x.iter().zip(h) {
        un = un.wrapping_add(i32::from(a) * i32::from(b));
    }
    un
}

/// `resample_6k4`: `y.len()` samples at 6.4 kHz from `x[b..]` at 12.8 kHz (3 previous).
fn resample_6k4(x: &[i16], b: usize, y: &mut [i16]) {
    const H: [i32; 3] = [18477, 15424, 8105];
    for (j, yj) in y.iter_mut().enumerate() {
        let c = b - 1 + 2 * j;
        let at = |k: usize| i32::from(x[k]);
        *yj = ((at(c) * H[0] + (at(c - 1) + at(c + 1)) * H[1] + (at(c - 2) + at(c + 2)) * H[2])
            >> 16) as i16;
    }
}

/// `dot`: the rounded `>> 6` integer dot product, as a float. Upstream sums the
/// products' high and low 16-bit halves in separate 32-bit lanes (an SSE2
/// pmulhw/pmullw kernel); that equals the exact integer sum, which is what this
/// computes directly with a 64-bit accumulator (one SMLAL per term on Cortex-M).
fn dot(a: &[i16], b: &[i16], n: usize) -> f32 {
    let v: i64 = a[..n].iter().zip(&b[..n]).map(|(&x, &y)| i64::from(i32::from(x) * i32::from(y))).sum();
    ((v + (1 << 5)) >> 6) as i32 as f32
}

/// `correlate`: `y[k] = dot(x[a..], x[b - k..], n)`.
fn correlate(x: &[i16], a: usize, b: usize, n: usize, y: &mut [f32]) {
    for (k, yk) in y.iter_mut().enumerate() {
        *yk = dot(&x[a..], &x[b - k..], n);
    }
}

/// `argmax`: (argument, value) of the first maximum.
fn argmax(x: &[f32]) -> (usize, f32) {
    let (mut arg, mut x_max) = (0, x[0]);
    for (i, &v) in x.iter().enumerate().skip(1) {
        if x_max < v {
            (arg, x_max) = (i, v);
        }
    }
    (arg, x_max)
}

/// `argmax_weighted`: the argument of the maximum of `x[i] · (1 + i·w_incr)`, and its
/// unweighted value.
fn argmax_weighted(x: &[f32], w_incr: f32) -> (usize, f32) {
    let (mut arg, mut x_max) = (0, x[0]);
    let mut xw_max = x[0];
    let mut w = 1.0 + w_incr;
    for (i, &v) in x.iter().enumerate().skip(1) {
        if xw_max < v * w {
            (arg, x_max) = (i, v);
            xw_max = v * w;
        }
        w += w_incr;
    }
    (arg, x_max)
}

/// `interpolate`: `y.len()` samples of `x[b..]` (2 previous, 1 after) at phase `d` (0 to 3).
fn interpolate(x: &[i16], b: usize, d: usize, y: &mut [i16]) {
    const H4_Q15: [[i32; 4]; 4] = [
        [6877, 19121, 6877, 0],
        [3506, 18025, 11000, 220],
        [1300, 15048, 15048, 1300],
        [220, 11000, 18025, 3506],
    ];
    let h = &H4_Q15[d];
    for (j, yj) in y.iter_mut().enumerate() {
        let at = |k: usize| i32::from(x[b + j + 2 - k]);
        *yj = ((at(1) * h[0] + at(2) * h[1] + at(3) * h[2] + at(4) * h[3]) >> 15) as i16;
    }
}

/// `interpolate_corr`: the correlation `r` interpolated at `r[c]` + `d`/4 (`d` -3 to 3).
fn interpolate_corr(r: &[f32], c: usize, d: i32) -> f32 {
    let h = &LTPF_H4_CORR[((4 + d) % 4) as usize];
    let (mut y, h) = if d < 0 {
        (r[c - 4] * h[0], &h[1..])
    } else if d > 0 {
        (r[c + 4] * h[7], &h[..])
    } else {
        (0.0, &h[..])
    };
    y += r[c - 3] * h[0]
        + r[c - 2] * h[1]
        + r[c - 1] * h[2]
        + r[c] * h[3]
        + r[c + 1] * h[4]
        + r[c + 2] * h[5]
        + r[c + 3] * h[6];
    y
}

impl LtpfAnalysis {
    /// `detect_pitch`: the open-loop pitch lag on `n` 6.4 kHz samples at `x_6k4[b..]`
    /// (114 previous); returns (lag, pitch present).
    fn detect_pitch(&mut self, b: usize, n: usize) -> (i32, bool) {
        const R0: usize = 17;
        const NR: usize = 98;
        let x = &self.x_6k4;
        let k0 = (self.tc - 4).max(0) as usize;
        let nk = (NR as i32 - 1).min(self.tc + 4) as usize - k0 + 1;
        let mut r = [0f32; NR];
        correlate(x, b, b - R0, n, &mut r);
        let (t1, rm1) = argmax_weighted(&r, -0.5 / (NR - 1) as f32);
        let (t2, rm2) = argmax(&r[k0..k0 + nk]);
        let t2 = k0 + t2;
        let nc = |rm: f32, t: usize| {
            if rm <= 0.0 {
                0.0
            } else {
                let x1 = b - (R0 + t);
                rm / libm::sqrtf(dot(&x[b..], &x[b..], n) * dot(&x[x1..], &x[x1..], n))
            }
        };
        let (nc1, nc2) = (nc(rm1, t1), nc(rm2, t2));
        let t1sel = nc2 <= 0.85 * nc1;
        self.tc = if t1sel { t1 } else { t2 } as i32;
        (R0 as i32 + self.tc, if t1sel { nc1 } else { nc2 } > 0.6)
    }
}

/// `refine_pitch`: the fractional pitch around lag `tc` on `n` 12.8 kHz samples at `x[b..]`
/// (232 previous); returns (pitch index, pitch in Q2).
fn refine_pitch(x: &[i16], b: usize, n: usize, tc: i32) -> (i32, i32) {
    let r0 = 32.max(2 * tc - 4);
    let nr = (228.min(2 * tc + 4) - r0 + 1) as usize;
    let mut r = [0f32; 17];
    correlate(x, b, b - (r0 as usize - 4), n, &mut r[..nr + 8]);
    let (a, _) = argmax(&r[4..4 + nr]);
    let mut e = r0 + a as i32;
    let c = (e - (r0 - 4)) as usize;

    let mut f = 0;
    let mut dm = interpolate_corr(&r, c, 0);
    for i in 1..=3 {
        if e >= 127 && ((i & 1) != 0 || e >= 157) {
            continue;
        }
        let d = interpolate_corr(&r, c, i);
        if d > dm {
            (dm, f) = (d, i);
        }
        if e > 32 {
            let d = interpolate_corr(&r, c, -i);
            if d > dm {
                (dm, f) = (d, -i);
            }
        }
    }
    e -= i32::from(f < 0);
    f += 4 * i32::from(f < 0);
    let index = if e < 127 {
        4 * e + f - 128
    } else if e < 157 {
        2 * e + (f >> 1) + 126
    } else {
        e + 283
    };
    (index, 4 * e + f)
}

/// `lc3_ltpf_analyse`: from the PCM ring `x` (current frame at `xt`, at rate `sr`), the pitch
/// and activation into `data`; returns whether a pitch is present.
pub(crate) fn analyse(
    dt: usize,
    sr: usize,
    ltpf: &mut LtpfAnalysis,
    x: &[i16],
    xt: usize,
    data: &mut LtpfData,
) -> bool {
    // Resampling to 12.8 kHz
    let z_12k8 = ltpf.x_12k8.len();
    let mut n_12k8 = (1 + dt) * 32;
    ltpf.x_12k8.copy_within(n_12k8.., 0);
    let mut y = [0i16; 128];
    let y = &mut y[..n_12k8];
    match sr {
        0 => ltpf.resample(5, 8, &LTPF_H_8K_12K8, x, xt, y),
        1 => ltpf.resample(5, 4, &LTPF_H_16K_12K8, x, xt, y),
        2 => ltpf.resample(15, 8, &LTPF_H_24K_12K8, x, xt, y),
        3 => ltpf.resample(5, 2, &LTPF_H_32K_12K8, x, xt, y),
        _ => ltpf.resample(15, 4, &LTPF_H_48K_12K8, x, xt, y),
    }
    ltpf.x_12k8[z_12k8 - n_12k8..].copy_from_slice(y);
    let mut b_12k8 = z_12k8 - n_12k8 - if dt == DT_7M5 { 44 } else { 24 };

    // Resampling to 6.4 kHz
    let z_6k4 = ltpf.x_6k4.len();
    let mut n_6k4 = n_12k8 >> 1;
    ltpf.x_6k4.copy_within(n_6k4.., 0);
    resample_6k4(&ltpf.x_12k8, b_12k8, &mut ltpf.x_6k4[z_6k4 - n_6k4..]);
    let mut b_6k4 = z_6k4 - n_6k4;

    // Enlarged for the small frame size
    if dt == DT_2M5 {
        b_12k8 -= n_12k8;
        b_6k4 -= n_6k4;
        n_12k8 += n_12k8;
        n_6k4 += n_6k4;
    }

    // Pitch detection
    let (tc, pitch_present) = ltpf.detect_pitch(b_6k4, n_6k4);
    let (mut pitch, mut nc) = (0, 0f32);
    if pitch_present {
        let x12 = &ltpf.x_12k8;
        let (index, p) = refine_pitch(x12, b_12k8, n_12k8, tc);
        (data.pitch_index, pitch) = (index, p);
        let (mut u, mut v) = ([0i16; 128], [0i16; 128]);
        interpolate(x12, b_12k8, 0, &mut u[..n_12k8]);
        interpolate(
            x12,
            b_12k8 - (pitch >> 2) as usize,
            (pitch & 3) as usize,
            &mut v[..n_12k8],
        );
        nc = dot(&u, &v, n_12k8) / libm::sqrtf(dot(&u, &u, n_12k8) * dot(&v, &v, n_12k8));
    }

    // Activation
    data.active = if ltpf.active {
        let pitch_diff = pitch.max(ltpf.pitch) - pitch.min(ltpf.pitch);
        let nc_diff = nc - ltpf.nc[0];
        pitch_present && (nc > 0.9 || (nc > 0.84 && pitch_diff < 8 && nc_diff > -0.1))
    } else {
        pitch_present && (dt == DT_10M || ltpf.nc[1] > 0.94) && (ltpf.nc[0] > 0.94 && nc > 0.94)
    };
    ltpf.active = data.active;
    ltpf.pitch = pitch;
    ltpf.nc[1] = ltpf.nc[0];
    ltpf.nc[0] = nc;
    pitch_present
}

/// `lc3_ltpf_put_data`.
pub(crate) fn put_data(bits: &mut Writer, data: &LtpfData) {
    bits.put_bit(data.active);
    bits.put_bits(data.pitch_index as u32, 9);
}

/// `lc3_ltpf_get_nbits`.
pub(crate) fn get_nbits(pitch: bool) -> i32 {
    1 + 10 * i32::from(pitch)
}

/// `lc3_ltpf_get_data`.
pub(crate) fn get_data(bits: &mut Bits) -> LtpfData {
    LtpfData {
        active: bits.get_bit() != 0,
        pitch_index: bits.get_bits(9) as i32,
    }
}

/// `synthesize_template`: filters `n` samples at `xh[x..]` in place (`xh`: the ring of `nh`
/// samples), `x0` the `w - 1` previous inputs, `c` the `w` den then `w` num coefficients,
/// `fade` -1 out, 1 in, 0 none.
#[allow(clippy::too_many_arguments)]
fn synthesize_template(
    xh: &mut [f32],
    nh: usize,
    lag: usize,
    x0: &[f32],
    x: usize,
    n: usize,
    c: &[f32],
    w: usize,
    fade: i32,
) {
    // per width, as the C's `synthesize_4/6/8/12`: the ring indices `% w` become constants
    match w {
        4 => synthesize_w::<4>(xh, nh, lag, x0, x, n, c, fade),
        6 => synthesize_w::<6>(xh, nh, lag, x0, x, n, c, fade),
        8 => synthesize_w::<8>(xh, nh, lag, x0, x, n, c, fade),
        12 => synthesize_w::<12>(xh, nh, lag, x0, x, n, c, fade),
        _ => unreachable!("LTPF filter width {w}"),
    }
}

/// `synthesize_template` for width `W`.
#[allow(clippy::too_many_arguments)]
fn synthesize_w<const W: usize>(
    xh: &mut [f32],
    nh: usize,
    lag: usize,
    x0: &[f32],
    mut x: usize,
    n: usize,
    c: &[f32],
    fade: i32,
) {
    let w = W;
    let c = &c[..2 * W];
    let mut g = f32::from(u8::from(fade <= 0));
    let g_incr = (i32::from(fade > 0) - i32::from(fade < 0)) as f32 / n as f32;
    let mut u = [0f32; MAX_FILTER_WIDTH];

    let lag = lag + (w >> 1);
    let mut y = if x < lag { x + (nh - lag) } else { x - lag };
    let y_end = nh - 1;
    let next = |y: usize| if y < y_end { y + 1 } else { 0 };

    for j in 0..w - 1 {
        u[j] = 0.0;
        let (yi, xi) = (xh[y], x0[j]);
        y = next(y);
        for k in 0..=j {
            u[j - k] -= yi * c[k];
        }
        for k in 0..=j {
            u[j - k] += xi * c[w + k];
        }
    }
    u[w - 1] = 0.0;

    let mut i = 0;
    while i < n {
        for j in 0..w {
            let (yi, xi) = (xh[y], xh[x]);
            y = next(y);
            for k in 0..w {
                u[(j + (w - 1) - k) % w] -= yi * c[k];
            }
            for k in 0..w {
                u[(j + (w - 1) - k) % w] += xi * c[w + k];
            }
            xh[x] = xi - g * u[j];
            x += 1;
            u[j] = 0.0;
            g += g_incr;
        }
        i += w;
    }
}

/// `lc3_ltpf_synthesize`: post-filters the `ns` samples at `xh[x..]` (`xh`: the ring buffer
/// of `ns + nh` samples); `data` `None` on a lost frame.
pub(crate) fn synthesize(
    dt: usize,
    sr: usize,
    nbytes: i32,
    ltpf: &mut LtpfSynthesis,
    data: Option<&LtpfData>,
    xh: &mut [f32],
    x: usize,
) {
    let nh = ns(dt, sr) + nh(dt, sr);

    let p_idx = data.map_or(0, |d| d.pitch_index);
    let mut pitch = if p_idx >= 440 {
        (p_idx - 283) << 2
    } else if p_idx >= 380 {
        (((p_idx >> 1) - 63) << 2) + ((p_idx & 1) << 1)
    } else {
        (((p_idx >> 2) + 32) << 2) + (p_idx & 3)
    };
    pitch = (pitch * ns(DT_10M, sr) as i32 + 64) / 128;

    let mut nbits = (nbytes * 8 * (1 + DT_10M as i32)) / (1 + dt as i32);
    if dt == DT_2M5 {
        nbits = (6 * nbits + 5) / 10;
    }
    if dt == DT_5M {
        nbits -= 160;
    }
    let g_idx = (nbits / 80).max(3 + sr as i32) - (3 + sr as i32);
    let active = data.is_some_and(|d| d.active) && g_idx < 4;

    let w = 4.max(LC3_NS_4M[sr] >> 4);
    let mut c = [0f32; 2 * MAX_FILTER_WIDTH];
    for i in 0..w {
        let g = if active {
            0.4 - 0.05 * g_idx as f32
        } else {
            0.0
        };
        c[i] = g * lc3_ltpf_cden(sr, (pitch & 3) as usize)[(w - 1) - i];
        c[w + i] = 0.85 * g * lc3_ltpf_cnum(sr, g_idx.min(3) as usize)[(w - 1) - i];
    }

    let ns = ns(dt, sr);
    let nt = ns / (1 + dt);
    let mut x0 = [[0f32; MAX_FILTER_WIDTH]; 2];
    x0[0][..w - 1].copy_from_slice(&ltpf.x[..w - 1]);
    ltpf.x[..w - 1].copy_from_slice(&xh[x + ns - (w - 1)..x + ns]);
    if active {
        x0[1][..w - 1].copy_from_slice(&xh[x + nt - (w - 1)..x + nt]);
    }

    let lag = (pitch / 4) as usize;
    let old_lag = (ltpf.pitch / 4) as usize;
    if !ltpf.active && active {
        synthesize_template(xh, nh, lag, &x0[0], x, nt, &c, w, 1);
    } else if ltpf.active && !active {
        synthesize_template(xh, nh, old_lag, &x0[0], x, nt, &ltpf.c, w, -1);
    } else if ltpf.active && active && ltpf.pitch == pitch {
        synthesize_template(xh, nh, lag, &x0[0], x, nt, &c, w, 0);
    } else if ltpf.active && active {
        synthesize_template(xh, nh, old_lag, &x0[0], x, nt, &ltpf.c, w, -1);
        // the w - 1 samples before x in the ring, as just filtered
        let s = if x == 0 { nh } else { x } - (w - 1);
        let mut x1 = [0f32; MAX_FILTER_WIDTH];
        x1[..w - 1].copy_from_slice(&xh[s..s + w - 1]);
        synthesize_template(xh, nh, lag, &x1, x, nt, &c, w, 1);
    }

    if active && ns > nt {
        synthesize_template(xh, nh, lag, &x0[1], x + nt, ns - nt, &c, w, 0);
    }

    ltpf.active = active;
    ltpf.pitch = pitch;
    ltpf.c[..2 * w].copy_from_slice(&c[..2 * w]);
}

#[cfg(test)]
mod tests {
    use super::dot;
    use crate::testutil::Rng;

    /// The lane form against the plain i64 sum, extremes included (-32768² pairs wrap i32).
    #[test]
    fn dot_exact() {
        let mut r = Rng::new(7);
        for t in 0..2000 {
            let n = [16, 32, 64, 128, 7, 50, 10, 20, 30, 60][t % 10];
            let pick = |r: &mut Rng| match r.u64() % 4 {
                0 => i16::MIN,
                1 => i16::MAX,
                _ => r.u64() as i16,
            };
            let a: Vec<i16> = (0..n).map(|_| pick(&mut r)).collect();
            let b: Vec<i16> = (0..n)
                .map(|_| if t % 5 == 0 { i16::MIN } else { pick(&mut r) })
                .collect();
            let v: i64 = a
                .iter()
                .zip(&b)
                .map(|(&x, &y)| i64::from(x) * i64::from(y))
                .sum();
            let want = ((v + 32) >> 6) as i32 as f32;
            assert_eq!(dot(&a, &b, n), want, "n {n}");
            let wrap = a.iter().zip(&b).fold(0i32, |s, (&x, &y)| {
                s.wrapping_add(i32::from(x) * i32::from(y))
            });
            assert_eq!(super::dot_wrap(&a, &b), wrap, "n {n}");
        }
    }
}
