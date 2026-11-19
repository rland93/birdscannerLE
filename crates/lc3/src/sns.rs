//! Spectral noise shaping (liblc3 `sns.c`). Analysis (encoder): 16 scale factors from the
//! smoothed, pre-emphasised band energies (log2), two-stage VQ (codebooks, then a PVQ of the
//! DCT residual over 4 shapes), its enumeration. Synthesis (decoder): PVQ de-enumeration, the
//! 16 scale factors through the inverse DCT and codebooks. Both interpolate them per band and
//! apply them as gains.

use crate::bits::{Bits, Writer};
use crate::common::{DT_7M5, SRATE_48K};
use crate::fastmath::{exp2f, log2f};
use crate::tables::{
    DCT16_M, LC3_BAND_LIM, LC3_NUM_BANDS, LC3_SNS_HFCB, LC3_SNS_LFCB, LC3_SNS_MPVQ_OFFSETS,
    LC3_SNS_VQ_GAINS, SNS_GE,
};

/// `LC3_MAX_BANDS`.
const MAX_BANDS: usize = 64;

/// `lc3_sns_data_t`.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub(crate) struct SnsData {
    pub lfcb: usize,
    pub hfcb: usize,
    pub shape: usize,
    pub gain: usize,
    pub idx_a: i32,
    pub idx_b: i32,
    pub ls_a: bool,
    pub ls_b: bool,
}

/// `dct16_inverse`.
fn dct16_inverse(x: &[f32; 16], y: &mut [f32; 16]) {
    for i in 0..16 {
        y[i] = 0.0;
        for j in 0..16 {
            y[i] += x[j] * DCT16_M[i][j];
        }
    }
}

/// `normalize`.
fn normalize(c: &[i32; 16], cn: &mut [f32; 16]) {
    let c2_sum: i32 = c.iter().map(|&v| v * v).sum();
    let c_norm = 1.0 / libm::sqrtf(c2_sum as f32);
    for i in 0..16 {
        cn[i] = c[i] as f32 * c_norm;
    }
}

/// `unquantize`.
fn unquantize(
    lfcb: usize,
    hfcb: usize,
    c: &[f32; 16],
    shape: usize,
    gain: usize,
    scf: &mut [f32; 16],
) {
    let g = LC3_SNS_VQ_GAINS[shape][gain];
    dct16_inverse(c, scf);
    for i in 0..8 {
        scf[i] = LC3_SNS_LFCB[lfcb][i] + g * scf[i];
    }
    for i in 8..16 {
        scf[i] = LC3_SNS_HFCB[hfcb][i - 8] + g * scf[i];
    }
}

/// `deenum_mvpq`: `n` pulse positions of `c` from the enumeration index.
fn deenum_mvpq(mut idx: i32, mut ls: bool, mut npulses: usize, c: &mut [i32], n: usize) {
    let mut o = 0;
    let mut i = n as isize - 1;
    while i >= 0 && idx != 0 {
        let row = &LC3_SNS_MPVQ_OFFSETS[i as usize];
        let mut ci = 0;
        while idx < row[npulses - ci] {
            ci += 1;
        }
        idx -= row[npulses - ci];
        c[o] = if ls { -(ci as i32) } else { ci as i32 };
        o += 1;
        npulses -= ci;
        if ci > 0 {
            ls = idx & 1 != 0;
            idx >>= 1;
        }
        i -= 1;
    }
    let ci = npulses as i32;
    if i >= 0 {
        c[o] = if ls { -ci } else { ci };
        o += 1;
    }
    i -= 1;
    while i >= 0 {
        c[o] = 0;
        o += 1;
        i -= 1;
    }
}

/// `deenumerate`.
fn deenumerate(shape: usize, idx_a: i32, ls_a: bool, idx_b: i32, ls_b: bool, c: &mut [i32; 16]) {
    let npulses_a = [10, 10, 8, 6][shape];
    deenum_mvpq(idx_a, ls_a, npulses_a, c, if shape < 2 { 10 } else { 16 });
    if shape == 0 {
        deenum_mvpq(idx_b, ls_b, 1, &mut c[10..], 6);
    } else if shape == 1 {
        c[10..].fill(0);
    }
}

/// `spectral_shaping`: the 16 factors (negated if `inv`) interpolated to band gains 2^-scf,
/// applied to `x[i]` and written by `put(i, x[i] · gain)`.
fn spectral_shaping(
    dt: usize,
    sr: usize,
    scf_q: &[f32; 16],
    inv: bool,
    mut apply: impl FnMut(usize, f32),
) {
    let sg = |v: f32| if inv { -v } else { v };
    let mut scf = [0f32; MAX_BANDS];
    let mut s0;
    let mut s1 = sg(scf_q[0]);
    scf[0] = s1;
    scf[1] = s1;
    for i in 0..15 {
        s0 = s1;
        s1 = sg(scf_q[i + 1]);
        scf[4 * i + 2] = s0 + 0.125 * (s1 - s0);
        scf[4 * i + 3] = s0 + 0.375 * (s1 - s0);
        scf[4 * i + 4] = s0 + 0.625 * (s1 - s0);
        scf[4 * i + 5] = s0 + 0.875 * (s1 - s0);
    }
    let s0 = sg(scf_q[14]);
    scf[62] = s1 + 0.125 * (s1 - s0);
    scf[63] = s1 + 0.375 * (s1 - s0);

    let nb = LC3_NUM_BANDS[dt][sr];
    let n4 = if nb < 32 { 32 % nb } else { 0 };
    let n2 = if nb < 32 { nb - n4 } else { MAX_BANDS - nb };
    for i4 in 0..n4 {
        scf[i4] = 0.25 * (scf[4 * i4] + scf[4 * i4 + 1] + scf[4 * i4 + 2] + scf[4 * i4 + 3]);
    }
    for i2 in n4..n4 + n2 {
        scf[i2] = 0.5 * (scf[2 * (n4 + i2)] + scf[2 * (n4 + i2) + 1]);
    }
    scf.copy_within(4 * n4 + 2 * n2..4 * n4 + 2 * n2 + (nb - n4 - n2), n4 + n2);

    let lim = LC3_BAND_LIM[dt][sr].expect("bands for (dt, sr)");
    let mut i = 0;
    for ib in 0..nb {
        let g_sns = exp2f(-scf[ib]);
        while i < lim[ib + 1] {
            apply(i, g_sns);
            i += 1;
        }
    }
}

/// `lc3_sns_synthesize`: the spectrum `x` shaped into `y`.
pub(crate) fn synthesize(dt: usize, sr: usize, data: &SnsData, x: &[f32], y: &mut [f32]) {
    let (mut scf, mut cn, mut c) = ([0f32; 16], [0f32; 16], [0i32; 16]);
    deenumerate(
        data.shape, data.idx_a, data.ls_a, data.idx_b, data.ls_b, &mut c,
    );
    normalize(&c, &mut cn);
    unquantize(data.lfcb, data.hfcb, &cn, data.shape, data.gain, &mut scf);
    spectral_shaping(dt, sr, &scf, true, |i, g| y[i] = x[i] * g);
}

/// `dct16_forward`.
fn dct16_forward(x: &[f32; 16], y: &mut [f32; 16]) {
    for i in 0..16 {
        y[i] = 0.0;
        for j in 0..16 {
            y[i] += x[j] * DCT16_M[j][i];
        }
    }
}

/// `compute_scale_factors`: 16 scale factors from the band energies `eb`.
fn compute_scale_factors(dt: usize, sr: usize, eb: &[f32], att: bool, scf: &mut [f32; 16]) {
    let mut e = [0f32; MAX_BANDS];

    // Copy and padding
    let nb = LC3_NUM_BANDS[dt][sr];
    let n4 = if nb < 32 { 32 % nb } else { 0 };
    let n2 = if nb < 32 { nb - n4 } else { MAX_BANDS - nb };
    for i4 in 0..n4 {
        e[4 * i4..4 * i4 + 4].fill(eb[i4]);
    }
    for i2 in n4..n4 + n2 {
        e[2 * (n4 + i2)] = eb[i2];
        e[2 * (n4 + i2) + 1] = eb[i2];
    }
    let m = nb - n4 - n2;
    e[4 * n4 + 2 * n2..4 * n4 + 2 * n2 + m].copy_from_slice(&eb[n4 + n2..n4 + n2 + m]);

    // Smoothing, pre-emphasis and logarithm
    let ge = &SNS_GE[sr.min(SRATE_48K)];
    let (mut e0, mut e1) = (e[0], e[0]);
    let mut e2;
    let mut e_sum = 0f32;
    let mut i = 0;
    while i < MAX_BANDS - 1 {
        e2 = e[i + 1];
        e[i] = (e0 * 0.25 + e1 * 0.5 + e2 * 0.25) * ge[i];
        e_sum += e[i];
        i += 1;
        e0 = e[i + 1];
        e[i] = (e1 * 0.25 + e2 * 0.5 + e0 * 0.25) * ge[i];
        e_sum += e[i];
        i += 1;
        e1 = e[i + 1];
        e[i] = (e2 * 0.25 + e0 * 0.5 + e1 * 0.25) * ge[i];
        e_sum += e[i];
        i += 1;
    }
    e[MAX_BANDS - 1] = (e0 * 0.25 + e1 * 0.75) * ge[MAX_BANDS - 1];
    e_sum += e[MAX_BANDS - 1];

    let noise_floor = (e_sum * (1e-4 / 64.0)).max(f32::from_bits(0x2f80_0000)); // 2^-32
    for v in &mut e {
        *v = log2f(v.max(noise_floor)) * 0.5;
    }

    // Grouping and scaling. The C's `a * 1.f/12` is `(a * 1.f) / 12`: divisions, kept.
    scf[0] = (e[0] + e[4]) * 1.0 / 12.0 + (e[0] + e[3]) * 2.0 / 12.0 + (e[1] + e[2]) * 3.0 / 12.0;
    let mut scf_sum = scf[0];
    for i in 1..15 {
        scf[i] = (e[4 * i - 1] + e[4 * i + 4]) * 1.0 / 12.0
            + (e[4 * i] + e[4 * i + 3]) * 2.0 / 12.0
            + (e[4 * i + 1] + e[4 * i + 2]) * 3.0 / 12.0;
        scf_sum += scf[i];
    }
    scf[15] =
        (e[59] + e[63]) * 1.0 / 12.0 + (e[60] + e[63]) * 2.0 / 12.0 + (e[61] + e[62]) * 3.0 / 12.0;
    scf_sum += scf[15];

    let cf = 0.85f32;
    for v in scf.iter_mut() {
        *v = cf * (*v - scf_sum * 1.0 / 16.0);
    }

    // Attack handling
    if !att {
        return;
    }
    let (mut s1, mut s2, mut s3, mut s4) = (scf[0], scf[1], scf[2], scf[3]);
    let mut sn = s1 + s2;
    sn += s3;
    scf[0] = sn * 1.0 / 3.0;
    sn += s4;
    scf[1] = sn * 1.0 / 4.0;
    let mut scf_sum = scf[0] + scf[1];
    for i in 2..14 {
        let s0 = s1;
        (s1, s2, s3, s4) = (s2, s3, s4, scf[i + 2]);
        sn += s4;
        scf[i] = sn * 1.0 / 5.0;
        scf_sum += scf[i];
        sn -= s0;
    }
    scf[14] = sn * 1.0 / 4.0;
    sn -= s1;
    scf[15] = sn * 1.0 / 3.0;
    scf_sum += scf[14] + scf[15];
    let k = if dt == DT_7M5 { 0.3f32 } else { 0.5 };
    for v in scf.iter_mut() {
        *v = k * (*v - scf_sum * 1.0 / 16.0);
    }
}

/// `resolve_codebooks`: the nearest low and high frequency codebook entries.
fn resolve_codebooks(scf: &[f32; 16]) -> (usize, usize) {
    let (mut dlfcb_max, mut dhfcb_max) = (0f32, 0f32);
    let (mut lfcb_idx, mut hfcb_idx) = (0, 0);
    for icb in 0..32 {
        let (lfcb, hfcb) = (&LC3_SNS_LFCB[icb], &LC3_SNS_HFCB[icb]);
        let (mut dlfcb, mut dhfcb) = (0f32, 0f32);
        for i in 0..8 {
            dlfcb += (scf[i] - lfcb[i]) * (scf[i] - lfcb[i]);
            dhfcb += (scf[8 + i] - hfcb[i]) * (scf[8 + i] - hfcb[i]);
        }
        if icb == 0 || dlfcb < dlfcb_max {
            (lfcb_idx, dlfcb_max) = (icb, dlfcb);
        }
        if icb == 0 || dhfcb < dhfcb_max {
            (hfcb_idx, dhfcb_max) = (icb, dhfcb);
        }
    }
    (lfcb_idx, hfcb_idx)
}

/// `add_pulse`: unit pulses on `y[..n]` (against `x`) from `start` up to `end` pulses.
fn add_pulse(
    x: &[f32],
    y: &mut [i32],
    n: usize,
    start: i32,
    end: i32,
    corr: &mut f32,
    energy: &mut f32,
) {
    for _ in start..end {
        let mut best_c2 = (*corr + x[0]) * (*corr + x[0]);
        let mut best_e = *energy + (2 * y[0]) as f32 + 1.0;
        let mut nbest = 0;
        for i in 1..n {
            let c2 = (*corr + x[i]) * (*corr + x[i]);
            let e = *energy + (2 * y[i]) as f32 + 1.0;
            if c2 * best_e > e * best_c2 {
                (best_c2, best_e, nbest) = (c2, e, i);
            }
        }
        *corr += x[nbest];
        *energy += (2 * y[nbest]) as f32 + 1.0;
        y[nbest] += 1;
    }
}

/// `quantize`: the 4 pulse configurations `c` (normalised `cn`) of the residual of `scf`
/// against the codebooks; returns the (shape, gain) of least squared error.
fn quantize(
    scf: &[f32; 16],
    lfcb_idx: usize,
    hfcb_idx: usize,
    c: &mut [[i32; 16]; 4],
    cn: &mut [[f32; 16]; 4],
) -> (usize, usize) {
    // Residual
    let (lfcb, hfcb) = (&LC3_SNS_LFCB[lfcb_idx], &LC3_SNS_HFCB[hfcb_idx]);
    let (mut r, mut x) = ([0f32; 16], [0f32; 16]);
    for i in 0..8 {
        r[i] = scf[i] - lfcb[i];
        r[8 + i] = scf[8 + i] - hfcb[i];
    }
    dct16_forward(&r, &mut x);

    // Shape 3: projected to or below the pyramid N = 16, K = 6, then unit pulses up to K = 6
    let mut xm = [0f32; 16];
    let mut xm_sum = 0f32;
    for i in 0..16 {
        xm[i] = x[i].abs();
        xm_sum += xm[i];
    }
    let proj_factor = (6 - 1) as f32 / xm_sum.max(1e-31);
    let (mut corr, mut energy) = (0f32, 0f32);
    let mut npulses = 0;
    for i in 0..16 {
        c[3][i] = libm::floorf(xm[i] * proj_factor) as i32;
        npulses += c[3][i];
        corr += c[3][i] as f32 * xm[i];
        energy += (c[3][i] * c[3][i]) as f32;
    }
    add_pulse(&xm, &mut c[3], 16, npulses, 6, &mut corr, &mut energy);
    npulses = 6;

    // Shape 2: up to K = 8 on shape 3
    c[2] = c[3];
    add_pulse(&xm, &mut c[2], 16, npulses, 8, &mut corr, &mut energy);
    npulses = 8;

    // Shape 1: the pulses of shape 2 on 0..10, then up to K = 10 over N = 10
    c[1] = c[2];
    for i in 10..16 {
        c[1][i] = 0;
        npulses -= c[2][i];
        corr -= c[2][i] as f32 * xm[i];
        energy -= (c[2][i] * c[2][i]) as f32;
    }
    add_pulse(&xm, &mut c[1], 10, npulses, 10, &mut corr, &mut energy);

    // Shape 0: one more pulse on 10..16, on shape 1
    c[0] = c[1];
    add_pulse(&xm[10..], &mut c[0][10..], 6, 0, 1, &mut corr, &mut energy);

    // Signs and unit energy
    for j in 0..16 {
        for ci in c.iter_mut() {
            if x[j] < 0.0 {
                ci[j] = -ci[j];
            }
        }
    }
    for i in 0..4 {
        normalize(&c[i], &mut cn[i]);
    }

    // Shape and gain of least squared error
    let mut mse_min = f32::MAX;
    let (mut shape_idx, mut gain_idx) = (0, 0);
    for ic in 0..4 {
        let mut cmse_min = f32::MAX;
        let mut cgain_idx = 0;
        for (ig, &g) in LC3_SNS_VQ_GAINS[ic].iter().enumerate() {
            let mut mse = 0f32;
            for i in 0..16 {
                mse += (x[i] - g * cn[ic][i]) * (x[i] - g * cn[ic][i]);
            }
            if mse < cmse_min {
                (cgain_idx, cmse_min) = (ig, mse);
            }
        }
        if cmse_min < mse_min {
            (shape_idx, gain_idx, mse_min) = (ic, cgain_idx, cmse_min);
        }
    }
    (shape_idx, gain_idx)
}

/// `enum_mvpq`: the enumeration (index, leading sign) of the `n` pulses of `c`.
fn enum_mvpq(c: &[i32], n: usize) -> (i32, bool) {
    // first significant coefficient, from the end
    let mut p = n;
    let mut i = 0;
    let mut ci;
    loop {
        p -= 1;
        ci = c[p];
        if ci != 0 || i >= 15 {
            break;
        }
        i += 1;
    }
    let mut idx = 0i32;
    let mut ls = ci < 0;
    let mut j = ci.unsigned_abs();
    i += 1;
    while i < n {
        p -= 1;
        ci = c[p];
        if ci != 0 {
            idx = (idx << 1) | i32::from(ls);
            ls = ci < 0;
        }
        idx += LC3_SNS_MPVQ_OFFSETS[i][j.min(10) as usize];
        i += 1;
        j += ci.unsigned_abs();
    }
    (idx, ls)
}

/// `lc3_sns_analyze`: the scale factors of the band energies `eb`, quantised into the
/// returned data; the spectrum `x` shaped in place.
pub(crate) fn analyze(dt: usize, sr: usize, eb: &[f32], att: bool, x: &mut [f32]) -> SnsData {
    let (mut scf, mut cn, mut c) = ([0f32; 16], [[0f32; 16]; 4], [[0i32; 16]; 4]);
    compute_scale_factors(dt, sr, eb, att, &mut scf);
    let (lfcb, hfcb) = resolve_codebooks(&scf);
    let (shape, gain) = quantize(&scf, lfcb, hfcb, &mut c, &mut cn);
    unquantize(lfcb, hfcb, &cn[shape], shape, gain, &mut scf);
    let (idx_a, ls_a) = enum_mvpq(&c[shape], if shape < 2 { 10 } else { 16 });
    let (idx_b, ls_b) = if shape == 0 {
        enum_mvpq(&c[shape][10..], 6)
    } else {
        (0, false)
    };
    spectral_shaping(dt, sr, &scf, false, |i, g| x[i] *= g);
    SnsData {
        lfcb,
        hfcb,
        shape,
        gain,
        idx_a,
        idx_b,
        ls_a,
        ls_b,
    }
}

/// `lc3_sns_put_data`.
pub(crate) fn put_data(bits: &mut Writer, data: &SnsData) {
    bits.put_bits(data.lfcb as u32, 5);
    bits.put_bits(data.hfcb as u32, 5);
    let shape_msb = data.shape >> 1;
    bits.put_bit(shape_msb != 0);
    let submode = data.shape & 1;
    if shape_msb == 0 {
        let size_a = 2_390_004;
        let mux_high = if submode == 0 {
            2 * (data.idx_b + 1) + i32::from(data.ls_b)
        } else {
            (data.gain & 1) as i32
        };
        let mux_code = mux_high * size_a + data.idx_a;
        bits.put_bits((data.gain >> submode) as u32, 1);
        bits.put_bit(data.ls_a);
        bits.put_bits(mux_code as u32, 25);
    } else {
        let size_a = 15_158_272;
        let mux_code = if submode == 0 {
            data.idx_a
        } else {
            size_a + 2 * data.idx_a + (data.gain & 1) as i32
        };
        bits.put_bits((data.gain >> submode) as u32, 2);
        bits.put_bit(data.ls_a);
        bits.put_bits(mux_code as u32, 24);
    }
}

/// `lc3_sns_get_nbits`.
pub(crate) fn get_nbits() -> i32 {
    38
}

/// `lc3_sns_get_data`; `Err` on an out-of-range multiplexed code.
pub(crate) fn get_data(bits: &mut Bits) -> Result<SnsData, ()> {
    let mut data = SnsData {
        lfcb: bits.get_bits(5) as usize,
        hfcb: bits.get_bits(5) as usize,
        ..SnsData::default()
    };
    let shape_msb = bits.get_bit() as i32;
    data.gain = bits.get_bits(1 + shape_msb) as usize;
    data.ls_a = bits.get_bit() != 0;
    let mut mux_code = bits.get_bits(25 - shape_msb) as i32;
    if shape_msb == 0 {
        let size_a = 2_390_004;
        if mux_code >= size_a * 14 {
            return Err(());
        }
        data.idx_a = mux_code % size_a;
        mux_code /= size_a;
        data.shape = usize::from(mux_code < 2);
        if data.shape == 0 {
            data.idx_b = (mux_code - 2) / 2;
            data.ls_b = (mux_code - 2) % 2 != 0;
        } else {
            data.gain = (data.gain << 1) + (mux_code % 2) as usize;
        }
    } else {
        let size_a = 15_158_272;
        if mux_code >= size_a + 1_549_824 {
            return Err(());
        }
        data.shape = 2 + usize::from(mux_code >= size_a);
        if data.shape == 2 {
            data.idx_a = mux_code;
        } else {
            mux_code -= size_a;
            data.idx_a = mux_code / 2;
            data.gain = (data.gain << 1) + (mux_code % 2) as usize;
        }
    }
    Ok(data)
}
