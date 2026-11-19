//! Spectral coefficients (liblc3 `spec.c`): side data, arithmetic-coded quantized values,
//! residual or LSB bits, global gain and noise level. The encoder estimates the global gain
//! from the bit budget, quantizes, counts the bits and adjusts once; the decoder fills the
//! zeroed ranges with noise.

use crate::bits::{Bits, Writer};
use crate::common::{hr, ne};
use crate::fastmath::db_q16;
use crate::tables::{LC3_SPECTRUM_BITS, LC3_SPECTRUM_LOOKUP, LC3_SPECTRUM_MODELS, SPEC_IQ_TABLE};

/// `LC3_SRATE_48K_HR`, `LC3_SRATE_96K_HR`, `LC3_BANDWIDTH_FB`.
const SRATE_48K_HR: usize = 5;
const SRATE_96K_HR: usize = 6;
const BANDWIDTH_FB: usize = 4;
/// `LC3_DT_7M5`, `LC3_DT_10M`.
const DT_7M5: usize = 2;
const DT_10M: usize = 3;

/// `lc3_spec_side_t`.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub(crate) struct SpecSide {
    pub g_idx: i32,
    pub nq: usize,
    pub lsb_mode: bool,
}

/// `sr_ind` of `resolve_gain_offset` and `resolve_modes`.
fn sr_ind(sr: usize) -> i32 {
    if hr(sr) {
        4 + (sr - SRATE_48K_HR) as i32
    } else {
        sr as i32
    }
}

/// `resolve_gain_offset`.
fn resolve_gain_offset(sr: usize, nbytes: i32) -> i32 {
    let sr_ind = sr_ind(sr);
    let g_off = (nbytes * 8) / (10 * (1 + sr_ind));
    let max = if sr >= SRATE_96K_HR { 181 } else { 255 };
    max.min(105 + 5 * (1 + sr_ind) + g_off.min(115))
}

/// `unquantize_gain`: 10^(g/28).
fn unquantize_gain(mut g_int: i32) -> f32 {
    let mut g = 1f32;
    while g_int < 0 {
        g_int += 28;
        g *= 0.1;
    }
    while g_int >= 28 {
        g_int -= 28;
        g *= 10.0;
    }
    g * SPEC_IQ_TABLE[g_int as usize]
}

/// `unquantize`: scales the `nq` decoded values, clears the rest up to `ne`.
fn unquantize(dt: usize, sr: usize, g_int: i32, x: &mut [f32], nq: usize) -> f32 {
    let g = unquantize_gain(g_int);
    for v in &mut x[..nq] {
        *v *= g;
    }
    x[nq..ne(dt, sr)].fill(0.0);
    g
}

/// `resolve_modes`: (high-rate mode, LSB mode allowed).
fn resolve_modes(sr: usize, nbytes: i32) -> (bool, bool) {
    let sr_ind = sr_ind(sr);
    let lsb = nbytes >= 20 * (3 + sr_ind) && sr < SRATE_96K_HR;
    (nbytes > 20 * (1 + sr_ind) && sr < SRATE_96K_HR, lsb)
}

/// `get_quantized`: the arithmetic-coded values; `None` on an escape overflow.
fn get_quantized(
    bits: &mut Bits,
    dt: usize,
    sr: usize,
    nbytes: i32,
    nq: usize,
    lsb_mode: bool,
    x: &mut [f32],
) -> Option<u16> {
    let high_rate = resolve_modes(sr, nbytes).0;
    let ne = ne(dt, sr);
    let mut nf_seed = 0u16;
    let mut state = 0u8;
    let mut i = 0;
    for (h, lut_coeff) in LC3_SPECTRUM_LOOKUP[usize::from(high_rate)]
        .iter()
        .enumerate()
    {
        while i < nq.min((ne + 2) >> (1 - h)) {
            let lut = &lut_coeff[usize::from(state)];
            let max_shl = if hr(sr) { 22 } else { 14 };
            let (mut u, mut v) = (0i32, 0i32);
            let (mut k, mut shl) = (0usize, 0);
            let mut s = bits.get_symbol(&LC3_SPECTRUM_MODELS[usize::from(lut[k])]);
            if lsb_mode && s >= 16 {
                k += 1;
                s = bits.get_symbol(&LC3_SPECTRUM_MODELS[usize::from(lut[k])]);
                shl += 1;
            }
            while s >= 16 && shl < max_shl {
                u |= (bits.get_bit() as i32) << shl;
                v |= (bits.get_bit() as i32) << shl;
                k += usize::from(k < 3);
                s = bits.get_symbol(&LC3_SPECTRUM_MODELS[usize::from(lut[k])]);
                shl += 1;
            }
            if s >= 16 {
                return None;
            }
            let (a, b) = ((s % 4) as i32, (s / 4) as i32);
            u |= a << shl;
            v |= b << shl;
            x[i] = (if u != 0 && bits.get_bit() != 0 { -u } else { u }) as f32;
            x[i + 1] = (if v != 0 && bits.get_bit() != 0 { -v } else { v }) as f32;
            nf_seed = (i32::from(nf_seed) + (u & 0x7fff) * i as i32 + (v & 0x7fff) * (i as i32 + 1))
                as u16;
            state = (i32::from(state) << 4).wrapping_add(if k > 1 {
                12 + k as i32
            } else {
                1 + (a + b) * (k as i32 + 1)
            }) as u8;
            i += 2;
        }
    }
    Some(nf_seed)
}

/// `get_residual`: residual quantization bits, one per non-zero value.
fn get_residual(bits: &mut Bits, mut nbits: i32, hrmode: bool, x: &mut [f32], n: usize) {
    let (mut xq_off_1, mut xq_off_2) = if hrmode {
        (0.25f32, 0.25f32)
    } else {
        (5.0 / 16.0, 3.0 / 16.0)
    };
    let mut iter = 0;
    while iter < (if hrmode { 20 } else { 1 }) && nbits > 0 {
        let mut i = 0;
        while i < n && nbits > 0 {
            if x[i] != 0.0 {
                if bits.get_bit() == 0 {
                    x[i] -= if x[i] < 0.0 { xq_off_1 } else { xq_off_2 };
                } else {
                    x[i] += if x[i] > 0.0 { xq_off_1 } else { xq_off_2 };
                }
                nbits -= 1;
            }
            i += 1;
        }
        xq_off_1 *= 0.5;
        xq_off_2 *= 0.5;
        iter += 1;
    }
}

/// `get_lsb`: the LSB-mode least significant bits.
fn get_lsb(bits: &mut Bits, mut nbits: i32, x: &mut [f32], nq: usize, nf_seed: &mut u16) {
    let mut i = 0;
    while i < nq && nbits > 0 {
        let (a, b) = (x[i].abs(), x[i + 1].abs());
        if a.max(b) >= 4.0 {
            for (j, m) in [(i, a), (i + 1, b)] {
                let take = nbits > 0;
                nbits -= 1;
                if take && bits.get_bit() != 0 {
                    if m != 0.0 {
                        x[j] += if x[j] < 0.0 { -1.0 } else { 1.0 };
                        *nf_seed = nf_seed.wrapping_add(j as u16);
                    } else {
                        let take = nbits > 0;
                        nbits -= 1;
                        if take {
                            x[j] = if bits.get_bit() != 0 { -1.0 } else { 1.0 };
                            *nf_seed = nf_seed.wrapping_add(j as u16);
                        }
                    }
                }
            }
        }
        i += 2;
    }
}

/// `fill_noise`: noise filling of the zeroed runs, up to the bandwidth.
fn fill_noise(dt: usize, bw: usize, nf: i32, mut nf_seed: u16, g: f32, x: &mut [f32], nq: usize) {
    let bw_stop = ne(dt, bw.min(BANDWIDTH_FB));
    let w = 1 + usize::from(dt >= DT_7M5) + usize::from(dt >= DT_10M);
    let s = g * (8 - nf) as f32 / 16.0;
    let mut z = 0;
    let mut i = 6 * (1 + dt) - w;
    let next = |seed: &mut u16| {
        *seed = (13849u32 + u32::from(*seed) * 31821) as u16;
        if *seed & 0x8000 != 0 { -s } else { s }
    };
    while i < nq.min(bw_stop) {
        z = if x[i] != 0.0 { 0 } else { z + 1 };
        if z > 2 * w {
            x[i - w] = next(&mut nf_seed);
        }
        i += 1;
    }
    while i < bw_stop + w {
        z += 1;
        if z > 2 * w {
            x[i - w] = next(&mut nf_seed);
        }
        i += 1;
    }
}

/// `get_nbits_nq`.
fn get_nbits_nq(dt: usize, sr: usize) -> i32 {
    let ne = ne(dt, sr);
    4 + [32, 64, 128, 256, 512].iter().filter(|&&t| ne > t).count() as i32
}

/// `lc3_spec_get_side`; `Err` keeps the clamped side as the C does.
pub(crate) fn get_side(bits: &mut Bits, dt: usize, sr: usize) -> Result<SpecSide, SpecSide> {
    let ne = ne(dt, sr);
    let mut side = SpecSide {
        nq: ((bits.get_bits(get_nbits_nq(dt, sr)) + 1) << 1) as usize,
        ..SpecSide::default()
    };
    side.lsb_mode = bits.get_bit() != 0;
    side.g_idx = bits.get_bits(8) as i32;
    if side.nq > ne {
        side.nq = ne;
        return Err(side);
    }
    Ok(side)
}

/// `lc3_spec_decode`: the `ne` coefficients into `x`; `Err` on a bitstream error.
pub(crate) fn decode(
    bits: &mut Bits,
    dt: usize,
    sr: usize,
    bw: usize,
    nbytes: i32,
    side: &SpecSide,
    x: &mut [f32],
) -> Result<(), ()> {
    let (lsb_mode, nq) = (side.lsb_mode, side.nq);
    let nf = bits.get_bits(3) as i32;
    let mut nf_seed = get_quantized(bits, dt, sr, nbytes, nq, lsb_mode, x).ok_or(())?;
    let nbits_left = bits.get_bits_left();
    if lsb_mode {
        get_lsb(bits, nbits_left, x, nq, &mut nf_seed);
    } else {
        get_residual(bits, nbits_left, hr(sr), x, nq);
    }
    let g_int = side.g_idx - resolve_gain_offset(sr, nbytes);
    let g = unquantize(dt, sr, g_int, x, nq);
    if nq > 2 || x[0] != 0.0 || x[1] != 0.0 || side.g_idx > 0 || nf < 7 {
        fill_noise(dt, bw, nf, nf_seed, g, x, nq);
    }
    Ok(())
}

/// `lc3_spec_analysis_t`: the bit-budget offset carried across frames.
#[derive(Clone, Copy, Debug, Default)]
pub(crate) struct SpecAnalysis {
    nbits_off: f32,
    nbits_spare: i32,
}

/// `estimate_gain`: the global gain index (before the offset `g_off`) for `nbits_budget`,
/// and its minimum; also whether the bit offset resets.
fn estimate_gain(
    dt: usize,
    sr: usize,
    x: &[f32],
    nbits_budget: i32,
    nbits_off: f32,
    g_off: i32,
) -> (i32, i32, bool) {
    let n4 = ne(dt, sr) / 4;
    let mut e = [0i32; 400 / 4];
    let mut ef = [0f32; 400 / 4];

    // Energy (dB) by blocks of 4 coefficients
    let mut x2_max = 0f32;
    for i in 0..n4 {
        let x0 = x[4 * i] * x[4 * i];
        let x1 = x[4 * i + 1] * x[4 * i + 1];
        let x2 = x[4 * i + 2] * x[4 * i + 2];
        let x3 = x[4 * i + 3] * x[4 * i + 3];
        x2_max = x2_max.max(x0).max(x1).max(x2).max(x3);
        ef[i] = x0 + x1 + x2 + x3;
    }
    let x_max = libm::sqrtf(x2_max);
    let nf = 0f32;
    for i in 0..n4 {
        e[i] = db_q16((ef[i] + nf).max(1e-10));
    }

    // Gain index, by bisection
    let nbits = (nbits_budget as f32 + nbits_off + 0.5) as i32;
    let mut g_int = 255 - g_off;
    let k_20_28 = (20.0f32 / 28.0 * 65536.0 + 0.5) as i32;
    let k_2u7 = (2.7f32 * 65536.0 + 0.5) as i32;
    let k_1u4 = (1.4f32 * 65536.0 + 0.5) as i32;
    let mut j0 = n4 as i32 - 1;
    let mut i = 128;
    while i > 0 {
        let gn = (g_int - i) * k_20_28;
        let mut v = 0i32;
        let mut j = j0;
        while j >= 0 && e[j as usize] < gn {
            j -= 1;
        }
        let j1 = j;
        while j >= 0 {
            let e_diff = e[j as usize] - gn;
            // wrapping, as the C's int sum on the CPUs it runs on (it can exceed 2^31 only
            // for spectra far above 16-bit PCM)
            v = v.wrapping_add(if e_diff < 0 {
                k_2u7
            } else if e_diff < 43 << 16 {
                e_diff + (7 << 16)
            } else {
                2 * e_diff - (36 << 16)
            });
            j -= 1;
        }
        if v > nbits * k_1u4 {
            j0 = j1;
        } else {
            g_int -= i;
        }
        i >>= 1;
    }

    // Limit
    let x_lim = 32767f32;
    let mut g_min = 255 - g_off;
    let mut i = 128;
    while i > 0 {
        if x_lim * unquantize_gain(g_min - i) > x_max {
            g_min -= i;
        }
        i >>= 1;
    }
    let reset_off = g_int < g_min || x_max == 0.0;
    if reset_off {
        g_int = g_min;
    }
    (g_int, g_min, reset_off)
}

/// `adjust_gain`: the gain index step from the bits the quantized spectrum takes.
fn adjust_gain(sr: usize, g_idx: i32, nbits: i32, nbits_budget: i32, g_idx_min: i32) -> i32 {
    const T: [[i32; 3]; 5] = [
        [80, 500, 850],
        [230, 1025, 1700],
        [380, 1550, 2550],
        [530, 2075, 3400],
        [680, 2600, 4250],
    ];
    let t = &T[sr];
    let mut den = 48;
    let delta = if nbits < t[0] {
        3 * (nbits + 48)
    } else if nbits < t[1] {
        let (n0, range) = (3 * (t[0] + 48), t[1] - t[0]);
        den *= range;
        n0 * range + (nbits - t[0]) * (t[1] - n0)
    } else {
        nbits.min(t[2])
    };
    let delta = (delta + den / 2) / den;
    if nbits < nbits_budget - (delta + 2) {
        return -i32::from(g_idx > g_idx_min);
    }
    if nbits > nbits_budget {
        return i32::from(g_idx < 255) + i32::from(g_idx < 254 && nbits >= nbits_budget + delta);
    }
    0
}

/// `quantize`: the spectrum scaled by 10^(-g/28); returns the coded count (trailing zero pairs
/// dropped).
fn quantize(dt: usize, sr: usize, g_int: i32, x: &mut [f32]) -> usize {
    let g_inv = unquantize_gain(-g_int);
    let ne = ne(dt, sr);
    let mut n = ne;
    let xq_min = 10.0f32 / 16.0;
    for i in (0..ne).step_by(2) {
        x[i] *= g_inv;
        x[i + 1] *= g_inv;
        n = if x[i].abs() >= xq_min || x[i + 1].abs() >= xq_min {
            ne
        } else {
            n - 2
        };
    }
    n
}

/// A quantized magnitude: `fabsf(x) + 6/16`, truncated to `uint32_t`.
fn qabs(x: f32) -> u32 {
    (x.abs() + 6.0 / 16.0) as u32
}

/// `compute_nbits`: the bits (rounded up) of the first `n` quantized values within
/// `nbits_budget` (0: unbounded), `n` cut to the last pair that fits; with `p_lsb_mode`, the LSB
/// mode when the LSBs would not fit.
fn compute_nbits(
    dt: usize,
    sr: usize,
    nbytes: i32,
    x: &[f32],
    n: &mut usize,
    nbits_budget: i32,
    p_lsb_mode: Option<&mut bool>,
) -> i32 {
    let (high_rate, lsb_mode) = resolve_modes(sr, nbytes);
    let ne = ne(dt, sr);
    let (mut nbits, mut nbits_lsb) = (0i32, 0i32);
    let mut state = 0u8;
    let (mut nbits_end, mut n_end) = (0i32, 0usize);
    let nbits_budget = if nbits_budget != 0 {
        nbits_budget * 2048
    } else {
        i32::MAX
    };

    let mut i = 0;
    for (h, lut_coeff) in LC3_SPECTRUM_LOOKUP[usize::from(high_rate)]
        .iter()
        .enumerate()
    {
        while i < (*n).min((ne + 2) >> (1 - h)) && nbits <= nbits_budget {
            let (mut a, mut b) = (qabs(x[i]), qabs(x[i + 1]));
            let lut = &lut_coeff[state as usize];

            // Signs
            let s = i32::from(a != 0) + i32::from(b != 0);
            nbits += s * 2048;

            // LSBs: escape codes, one bit each (the first ones uncoded in LSB mode)
            let mut m = (a | b) >> 2;
            let mut k = 0usize;
            if m != 0 {
                if lsb_mode {
                    nbits += i32::from(LC3_SPECTRUM_BITS[lut[k] as usize][16]) - 2 * 2048;
                    k += 1;
                    nbits_lsb += 2 + i32::from(a == 1) + i32::from(b == 1);
                }
                m >>= u32::from(lsb_mode);
                while m != 0 {
                    nbits += i32::from(LC3_SPECTRUM_BITS[lut[k.min(3)] as usize][16]);
                    m >>= 1;
                    k += 1;
                }
                nbits += k as i32 * 2 * 2048;
                a >>= k;
                b >>= k;
                k = k.min(3);
            }

            // MSBs
            nbits += i32::from(LC3_SPECTRUM_BITS[lut[k] as usize][(a + 4 * b) as usize]);

            if s != 0 && nbits <= nbits_budget {
                n_end = i + 2;
                nbits_end = nbits;
            }
            state = (state << 4).wrapping_add(if k > 1 {
                12 + k as u8
            } else {
                (1 + (a + b) * (k as u32 + 1)) as u8
            });
            i += 2;
        }
    }

    *n = n_end;
    if let Some(p) = p_lsb_mode {
        *p = lsb_mode && nbits_end + nbits_lsb * 2048 > nbits_budget;
    }
    if nbits_budget == i32::MAX {
        nbits_end += nbits_lsb * 2048;
    }
    (nbits_end + 2047) / 2048
}

/// `put_quantized`: the first `nq` quantized values, arithmetic coded, LSBs and signs as bits.
fn put_quantized(
    bits: &mut Writer,
    dt: usize,
    sr: usize,
    nbytes: i32,
    x: &[f32],
    nq: usize,
    lsb_mode: bool,
) {
    let (high_rate, _) = resolve_modes(sr, nbytes);
    let ne = ne(dt, sr);
    let mut state = 0u8;
    let mut i = 0;
    for (h, lut_coeff) in LC3_SPECTRUM_LOOKUP[usize::from(high_rate)]
        .iter()
        .enumerate()
    {
        while i < nq.min((ne + 2) >> (1 - h)) {
            let (mut a, mut b) = (qabs(x[i]), qabs(x[i + 1]));
            let lut = &lut_coeff[state as usize];

            let mut m = (a | b) >> 2;
            let (mut k, mut shr) = (0usize, 0usize);
            if m != 0 {
                if lsb_mode {
                    bits.put_symbol(&LC3_SPECTRUM_MODELS[lut[k] as usize], 16);
                    k += 1;
                }
                m >>= u32::from(lsb_mode);
                while m != 0 {
                    bits.put_bit((a >> k) & 1 != 0);
                    bits.put_bit((b >> k) & 1 != 0);
                    bits.put_symbol(&LC3_SPECTRUM_MODELS[lut[k.min(3)] as usize], 16);
                    m >>= 1;
                    k += 1;
                }
                a >>= u32::from(lsb_mode);
                b >>= u32::from(lsb_mode);
                shr = k - usize::from(lsb_mode);
                k = k.min(3);
            }

            // Signs
            if a != 0 {
                bits.put_bit(x[i] < 0.0);
            }
            if b != 0 {
                bits.put_bit(x[i + 1] < 0.0);
            }

            // MSBs
            a >>= shr;
            b >>= shr;
            bits.put_symbol(&LC3_SPECTRUM_MODELS[lut[k] as usize], (a + 4 * b) as usize);

            state = (state << 4).wrapping_add(if k > 1 {
                12 + k as u8
            } else {
                (1 + (a + b) * (k as u32 + 1)) as u8
            });
            i += 2;
        }
    }
}

/// `put_residual`: one refinement bit per nonzero value while bits are left.
fn put_residual(bits: &mut Writer, mut nbits: i32, x: &mut [f32], n: usize) {
    let xq_lim = 10.0f32 / 16.0;
    let xq_off = xq_lim / 2.0;
    let mut i = 0;
    while i < n && nbits > 0 {
        let xr = x[i].abs();
        if xr >= xq_lim {
            let b = (xr - libm::truncf(xr) < xq_lim) ^ (x[i] < 0.0);
            bits.put_bit(b);
            nbits -= 1;
            x[i] += if b { -xq_off } else { xq_off };
        }
        i += 1;
    }
}

/// `put_lsb`: the LSBs (and signs of values 1) left out of the arithmetic coding.
fn put_lsb(bits: &mut Writer, mut nbits: i32, x: &[f32], n: usize) {
    let mut i = 0;
    while i < n && nbits > 0 {
        let (a, b) = (qabs(x[i]), qabs(x[i + 1]));
        if (a | b) >> 2 != 0 {
            let mut put = |bits: &mut Writer, v: bool| {
                let ok = nbits > 0;
                nbits -= 1;
                if ok {
                    bits.put_bit(v);
                }
            };
            put(bits, a & 1 != 0);
            if a == 1 {
                put(bits, x[i] < 0.0);
            }
            put(bits, b & 1 != 0);
            if b == 1 {
                put(bits, x[i + 1] < 0.0);
            }
        }
        i += 2;
    }
}

/// `estimate_noise`: the noise level (0 to 7) of the zeroed ranges.
fn estimate_noise(dt: usize, bw: usize, x: &[f32], n: usize) -> i32 {
    let bw_stop = ne(dt, bw.min(BANDWIDTH_FB));
    let w = 1 + usize::from(dt >= DT_7M5) + usize::from(dt >= DT_10M);
    let xq_lim = 10.0f32 / 16.0;
    let mut sum = 0f32;
    let (mut ns, mut z) = (0i32, 0usize);
    let mut i = 6 * (1 + dt) - w;
    while i < n.min(bw_stop) {
        z = if x[i].abs() < xq_lim { z + 1 } else { 0 };
        if z > 2 * w {
            sum += x[i - w].abs();
            ns += 1;
        }
        i += 1;
    }
    while i < bw_stop + w {
        z += 1;
        if z > 2 * w {
            sum += x[i - w].abs();
            ns += 1;
        }
        i += 1;
    }
    let nf = if ns != 0 {
        8 - ((16.0 * sum) / ns as f32 + 0.5) as i32
    } else {
        8
    };
    nf.clamp(0, 7)
}

/// `get_nbits_ac`.
fn get_nbits_ac(dt: usize, sr: usize, nbytes: i32) -> i32 {
    get_nbits_nq(dt, sr) + 3 + ((nbytes - 1) / 160).min(2)
}

/// `lc3_spec_analyze`: the side data of the spectrum `x` (quantized in place) within the
/// bits left by the other tools (`nbits_other`: bandwidth, LTPF, SNS, TNS).
pub(crate) fn analyze(
    dt: usize,
    sr: usize,
    nbytes: i32,
    nbits_other: i32,
    spec: &mut SpecAnalysis,
    x: &mut [f32],
) -> SpecSide {
    const NBITS_GAIN: i32 = 8;
    const NBITS_NF: i32 = 3;
    let nbits_budget =
        8 * nbytes - get_nbits_ac(dt, sr, nbytes) - nbits_other - NBITS_GAIN - NBITS_NF;

    // Global gain
    let mut nbits_off = spec.nbits_off + spec.nbits_spare as f32;
    nbits_off = nbits_off.clamp(-40.0, 40.0); // never NaN
    nbits_off = 0.8 * spec.nbits_off + 0.2 * nbits_off;
    let g_off = resolve_gain_offset(sr, nbytes);
    let (g_int, g_min, reset_off) = estimate_gain(dt, sr, x, nbits_budget, nbits_off, g_off);

    // Quantization
    let mut nq = quantize(dt, sr, g_int, x);
    let nbits = compute_nbits(dt, sr, nbytes, x, &mut nq, 0, None);
    spec.nbits_off = if reset_off { 0.0 } else { nbits_off };
    spec.nbits_spare = if reset_off { 0 } else { nbits_budget - nbits };

    // Adjust the gain and requantize
    let g_adj = adjust_gain(sr, g_off + g_int, nbits, nbits_budget, g_off + g_min);
    if g_adj != 0 {
        nq = quantize(dt, sr, g_adj, x);
    }
    let mut side = SpecSide {
        g_idx: g_int + g_adj + g_off,
        nq,
        lsb_mode: false,
    };
    compute_nbits(
        dt,
        sr,
        nbytes,
        x,
        &mut side.nq,
        nbits_budget,
        Some(&mut side.lsb_mode),
    );
    side
}

/// `lc3_spec_put_side`.
pub(crate) fn put_side(bits: &mut Writer, dt: usize, sr: usize, side: &SpecSide) {
    bits.put_bits(((side.nq >> 1).max(1) - 1) as u32, get_nbits_nq(dt, sr));
    bits.put_bit(side.lsb_mode);
    bits.put_bits(side.g_idx as u32, 8);
}

/// `lc3_spec_encode`: noise level, quantized values, then residual or LSB bits.
pub(crate) fn encode(
    bits: &mut Writer,
    dt: usize,
    sr: usize,
    bw: usize,
    nbytes: i32,
    side: &SpecSide,
    x: &mut [f32],
) {
    let (lsb_mode, nq) = (side.lsb_mode, side.nq);
    bits.put_bits(estimate_noise(dt, bw, x, nq) as u32, 3);
    put_quantized(bits, dt, sr, nbytes, x, nq, lsb_mode);
    let nbits_left = bits.get_bits_left();
    if lsb_mode {
        put_lsb(bits, nbits_left, x, nq);
    } else {
        put_residual(bits, nbits_left, x, nq);
    }
}
