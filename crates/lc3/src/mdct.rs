//! Low-delay MDCT and its inverse (liblc3 `mdct.c`): windowing, rotation, a mixed-radix FFT
//! (5 × 3^n3 × 2^n2 points). One function per C function, the C's operation order kept
//! expression by expression (float `+` is not associative; Rust never contracts to FMA), so
//! the outputs are bit-exact with liblc3 built without `-ffast-math` (upstream's `vs_c` test
//! against the C oracle is not carried in this no_std port; `tests/upstream.rs` compares the
//! whole codec against upstream lc3 0.1.0 instead).
//! Where the C works in place (window → rotation → FFT → rotation in one buffer viewed as
//! floats and as complexes), this uses two complex buffers: the values are the same.

use crate::common::{MAX_NS, nd, ns};
use crate::tables::{LC3_FFT_TWIDDLES_BF2, LC3_FFT_TWIDDLES_BF3, LC3_MDCT_ROT, LC3_MDCT_WIN};

/// `struct lc3_complex`.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub(crate) struct Complex {
    pub re: f32,
    pub im: f32,
}

/// A table entry (the generated tables' constructor).
pub(crate) const fn c(re: f32, im: f32) -> Complex {
    Complex { re, im }
}

/// The C's `static const float` constants: double literals rounded once to float.
const COS1: f32 = 0.3090169944_f64 as f32;
const COS2: f32 = -0.8090169944_f64 as f32;
const SIN1: f32 = -0.9510565163_f64 as f32;
const SIN2: f32 = -0.5877852523_f64 as f32;

/// `fft_5`: `n` interleaved 5-point transforms, `x[i + k n]` → `y[5 i + k]`.
fn fft_5(x: &[Complex], y: &mut [Complex], n: usize) {
    for i in 0..n {
        let x0 = x[i];
        let (x1, x2, x3, x4) = (x[i + n], x[i + 2 * n], x[i + 3 * n], x[i + 4 * n]);
        let s14 = c(x1.re + x4.re, x1.im + x4.im);
        let d14 = c(x1.re - x4.re, x1.im - x4.im);
        let s23 = c(x2.re + x3.re, x2.im + x3.im);
        let d23 = c(x2.re - x3.re, x2.im - x3.im);
        let y = &mut y[5 * i..5 * i + 5];

        y[0].re = x0.re + s14.re + s23.re;
        y[0].im = x0.im + s14.im + s23.im;

        y[1].re = x0.re + s14.re * COS1 - d14.im * SIN1 + s23.re * COS2 - d23.im * SIN2;
        y[1].im = x0.im + s14.im * COS1 + d14.re * SIN1 + s23.im * COS2 + d23.re * SIN2;

        y[2].re = x0.re + s14.re * COS2 - d14.im * SIN2 + s23.re * COS1 + d23.im * SIN1;
        y[2].im = x0.im + s14.im * COS2 + d14.re * SIN2 + s23.im * COS1 - d23.re * SIN1;

        y[3].re = x0.re + s14.re * COS2 + d14.im * SIN2 + s23.re * COS1 - d23.im * SIN1;
        y[3].im = x0.im + s14.im * COS2 - d14.re * SIN2 + s23.im * COS1 + d23.re * SIN1;

        y[4].re = x0.re + s14.re * COS1 + d14.im * SIN1 + s23.re * COS2 + d23.im * SIN2;
        y[4].im = x0.im + s14.im * COS1 - d14.re * SIN1 + s23.im * COS2 - d23.re * SIN2;
    }
}

/// Two complexes as one vector of lanes (re, im, re, im).
type C2 = [f32; 4];

#[inline(always)]
fn ld(x: &[Complex; 2]) -> C2 {
    [x[0].re, x[0].im, x[1].re, x[1].im]
}

#[inline(always)]
fn st(y: &mut [Complex; 2], v: C2) {
    *y = [c(v[0], v[1]), c(v[2], v[3])];
}

/// `x·w` as the two products the C sums in turn: (x.re·w.re, x.im·w.re) and
/// (−(x.im·w.im), x.re·w.im) (negating the product's operand is exact), lane-wise.
#[inline(always)]
fn cmul_terms(x: C2, w0: Complex, w1: Complex) -> (C2, C2) {
    (
        [x[0] * w0.re, x[1] * w0.re, x[2] * w1.re, x[3] * w1.re],
        [x[1] * -w0.im, x[0] * w0.im, x[3] * -w1.im, x[2] * w1.im],
    )
}

#[inline(always)]
fn add(a: C2, b: C2) -> C2 {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2], a[3] + b[3]]
}

#[inline(always)]
fn sub(a: C2, b: C2) -> C2 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2], a[3] - b[3]]
}

/// `x0 + x1·w0 + x2·w1` in the C's order: one radix-3 output.
#[inline(always)]
fn bf3_out(x0: C2, x1: C2, x2: C2, t0: [Complex; 2], t1: [Complex; 2]) -> C2 {
    let (a, b) = cmul_terms(x1, t0[0], t1[0]);
    let (c2, d) = cmul_terms(x2, t0[1], t1[1]);
    add(add(add(add(x0, a), b), c2), d)
}

/// `fft_bf3`: radix-3 butterflies over `n` groups of `n3` (`t`: 3 × n3 twiddle pairs). Two
/// butterflies per vector; each lane sums its terms in the C's order (the `−` terms as `+`
/// of a negated product: exact). An odd last one goes alone (its second lane zero).
fn fft_bf3(t: &[[Complex; 2]], x: &[Complex], y: &mut [Complex], n: usize) {
    let n3 = t.len() / 3;
    let (w0, w1, w2) = (&t[..n3], &t[n3..2 * n3], &t[2 * n3..]);
    let zero = [Complex::default(); 2];
    for i in 0..n {
        let x0 = &x[i * n3..][..n3];
        let x1 = &x[(n + i) * n3..][..n3];
        let x2 = &x[(2 * n + i) * n3..][..n3];
        let (y0, y12) = y[i * 3 * n3..][..3 * n3].split_at_mut(n3);
        let (y1, y2) = y12.split_at_mut(n3);
        let p = n3 / 2;
        let it = x0
            .as_chunks::<2>()
            .0
            .iter()
            .zip(x1.as_chunks::<2>().0)
            .zip(x2.as_chunks::<2>().0);
        let wt = w0
            .as_chunks::<2>()
            .0
            .iter()
            .zip(w1.as_chunks::<2>().0)
            .zip(w2.as_chunks::<2>().0);
        let yt = y0
            .as_chunks_mut::<2>()
            .0
            .iter_mut()
            .zip(y1.as_chunks_mut::<2>().0)
            .zip(y2.as_chunks_mut::<2>().0);
        for (((a, b), c3), (((u, v), w), ((o0, o1), o2))) in it.zip(wt.zip(yt)) {
            let (a, b, c3) = (ld(a), ld(b), ld(c3));
            st(o0, bf3_out(a, b, c3, u[0], u[1]));
            st(o1, bf3_out(a, b, c3, v[0], v[1]));
            st(o2, bf3_out(a, b, c3, w[0], w[1]));
        }
        if n3 % 2 == 1 {
            let j = 2 * p;
            let l = |x: &[Complex]| [x[j].re, x[j].im, 0.0, 0.0];
            let (a, b, c3) = (l(x0), l(x1), l(x2));
            for (wm, ym) in [(w0, &mut *y0), (w1, &mut *y1), (w2, &mut *y2)] {
                let v = bf3_out(a, b, c3, wm[j], zero);
                ym[j] = c(v[0], v[1]);
            }
        }
    }
}

/// `fft_bf2`: radix-2 butterflies over `n` groups of `n2` (`w`: n2 twiddles), two per vector
/// (as `fft_bf3`).
fn fft_bf2(w: &[Complex], x: &[Complex], y: &mut [Complex], n: usize) {
    let n2 = w.len();
    for i in 0..n {
        let x0 = &x[i * n2..][..n2];
        let x1 = &x[(n + i) * n2..][..n2];
        let (y0, y1) = y[i * 2 * n2..][..2 * n2].split_at_mut(n2);
        let it = x0.as_chunks::<2>().0.iter().zip(x1.as_chunks::<2>().0);
        let ot = y0
            .as_chunks_mut::<2>()
            .0
            .iter_mut()
            .zip(y1.as_chunks_mut::<2>().0);
        for (((a, b), wp), (o0, o1)) in it.zip(w.as_chunks::<2>().0).zip(ot) {
            let (a, (p, q)) = (ld(a), cmul_terms(ld(b), wp[0], wp[1]));
            st(o0, add(add(a, p), q));
            st(o1, sub(sub(a, p), q));
        }
        if n2 % 2 == 1 {
            let j = n2 - 1;
            let a = [x0[j].re, x0[j].im, 0.0, 0.0];
            let (p, q) = cmul_terms([x1[j].re, x1[j].im, 0.0, 0.0], w[j], Complex::default());
            let (u, v) = (add(add(a, p), q), sub(sub(a, p), q));
            y0[j] = c(u[0], u[1]);
            y1[j] = c(v[0], v[1]);
        }
    }
}

/// `fft`: `n` points from `a` (the C's `x` = `y0`), `b` the other buffer (`y1`); returns
/// whether the result is in `b`.
fn fft(a: &mut [Complex], b: &mut [Complex], mut n: usize) -> bool {
    let mut in_b = true;
    n /= 5;
    fft_5(a, b, n);
    let mut i3 = 0;
    while n & (n - 1) != 0 {
        n /= 3;
        let t = LC3_FFT_TWIDDLES_BF3[i3];
        if in_b {
            fft_bf3(t, b, a, n)
        } else {
            fft_bf3(t, a, b, n)
        }
        in_b = !in_b;
        i3 += 1;
    }
    let mut i2 = 0;
    while n > 1 {
        n >>= 1;
        let w = LC3_FFT_TWIDDLES_BF2[i2][i3].expect("FFT size has twiddles");
        if in_b {
            fft_bf2(w, b, a, n)
        } else {
            fft_bf2(w, a, b, n)
        }
        in_b = !in_b;
        i2 += 1;
    }
    in_b
}

/// `mdct_window`: windows `x` (ns samples) with the delayed `d` (nd, updated) into `y` (ns).
fn mdct_window(dt: usize, sr: usize, x: &[f32], d: &mut [f32], y: &mut [f32]) {
    // The C's two pointer walks as zipped forward/reversed slices (each step independent:
    // step k reads d[k] or d[nd-1-k] before writing it, and the halves never cross), each
    // element's operations as the C's.
    let win = LC3_MDCT_WIN[dt][sr].expect("window for (dt, sr)");
    let (ns, nd) = (ns(dt, sr), nd(dt, sr));
    let (m, h) = (ns - nd, ns / 2);
    let t = (ns - 2 * m) / 2;
    let (ylo, yhi) = y[..ns].split_at_mut(h);
    let (d_lo, d_hi) = d[..nd].split_at_mut(m + t);

    // first loop, k = 0..m: y[h-1-k] = d[k]·w[k] − x[m-1-k]·w[ns-1-k]; d[k] = x[m+k];
    // y[h+k] = d[k]·w[ns+k]
    let ys = ylo[h - m..].iter_mut().rev().zip(&mut yhi[..m]);
    let xs = x[..m].iter().rev().zip(&x[m..2 * m]);
    let ws = (win[..m].iter().zip(win[ns - m..ns].iter().rev())).zip(&win[ns..ns + m]);
    for (((y0, y1), d0), ((&xr, &xf), ((&wa, &wb), &wc))) in ys.zip(&mut d_lo[..m]).zip(xs.zip(ws))
    {
        *y0 = *d0 * wa - xr * wb;
        *d0 = xf;
        *y1 = *d0 * wc;
    }
    // second loop, k = 0..t, d0 = d[m+k] (up), d1 = d[nd-1-k] (down)
    let ys = ylo[h - m - t..h - m]
        .iter_mut()
        .rev()
        .zip(&mut yhi[m..m + t]);
    let ds = d_lo[m..].iter_mut().zip(d_hi.iter_mut().rev());
    let xs = x[2 * m..2 * m + t].iter().zip(x[ns - t..ns].iter().rev());
    let w0 = win[m..m + t]
        .iter()
        .zip(win[ns - m - t..ns - m].iter().rev());
    let w1 = win[ns + m..ns + m + t]
        .iter()
        .zip(win[ns + nd - t..ns + nd].iter().rev());
    for (((y0, y1), (d0, d1)), ((&xf, &xr), ((&wa, &wb), (&wc, &wd)))) in
        ys.zip(ds).zip(xs.zip(w0.zip(w1)))
    {
        *y0 = *d0 * wa - *d1 * wb;
        *d0 = xf;
        *d1 = xr;
        *y1 = *d0 * wc + *d1 * wd;
    }
}

/// `mdct_pre_fft`: folds the windowed `x` (2 n4 samples) into n4 rotated complexes.
fn mdct_pre_fft(w: &[Complex], x: &[f32], y: &mut [Complex]) {
    let n4 = w.len();
    let (mut x0, mut x1) = (0, 2 * n4);
    let (mut w0, mut w1) = (0, n4);
    let (mut y0, mut y1) = (0, n4);
    while x0 < x1 {
        let uw = w[w0];
        w0 += 1;
        x1 -= 1;
        let u = c(
            -x[x1] * uw.re + x[x0] * uw.im,
            x[x0] * uw.re + x[x1] * uw.im,
        );
        x0 += 1;
        w1 -= 1;
        let vw = w[w1];
        x1 -= 1;
        let v = c(
            -x[x1] * vw.im + x[x0] * vw.re,
            -x[x0] * vw.im - x[x1] * vw.re,
        );
        x0 += 1;
        y[y0] = u;
        y0 += 1;
        y1 -= 1;
        y[y1] = v;
    }
}

/// `mdct_post_fft`: rotates the n4 FFT outputs into 2 n4 coefficients.
fn mdct_post_fft(w: &[Complex], x: &[Complex], y: &mut [f32]) {
    let n4 = w.len();
    let n8 = n4 >> 1;
    let (mut w0, mut w1, mut x0, mut x1) = (n8, n8 - 1, n8, n8 - 1);
    let (mut y0, mut y1) = (n4, n4);
    while y1 > 0 {
        let u0 = x[x0].im * w[w0].im + x[x0].re * w[w0].re;
        let u1 = x[x1].re * w[w1].im - x[x1].im * w[w1].re;
        let v0 = x[x0].re * w[w0].im - x[x0].im * w[w0].re;
        let v1 = x[x1].im * w[w1].im + x[x1].re * w[w1].re;
        y[y0] = u0;
        y[y0 + 1] = u1;
        y[y1 - 1] = v0;
        y[y1 - 2] = v1;
        (y0, y1) = (y0 + 2, y1 - 2);
        (x0, w0) = (x0 + 1, w0 + 1);
        (x1, w1) = (x1.wrapping_sub(1), w1.wrapping_sub(1));
    }
}

/// `imdct_pre_fft`: rotates the 2 n4 coefficients `x` into n4 complexes.
fn imdct_pre_fft(w: &[Complex], x: &[f32], y: &mut [Complex]) {
    let n4 = w.len();
    let (mut x0, mut x1) = (0, 2 * n4);
    let (mut w0, mut w1) = (0, n4);
    let (mut y0, mut y1) = (0, n4);
    while x0 < x1 {
        let (u0, u1, v0, v1) = (x[x0], x[x1 - 1], x[x0 + 1], x[x1 - 2]);
        (x0, x1) = (x0 + 2, x1 - 2);
        w1 -= 1;
        let (uw, vw) = (w[w0], w[w1]);
        w0 += 1;
        y[y0] = c(-u0 * uw.re - u1 * uw.im, -u1 * uw.re + u0 * uw.im);
        y0 += 1;
        y1 -= 1;
        y[y1] = c(-v1 * vw.re - v0 * vw.im, -v0 * vw.re + v1 * vw.im);
    }
}

/// `imdct_post_fft`: rotates the n4 FFT outputs into 2 n4 samples.
fn imdct_post_fft(w: &[Complex], x: &[Complex], y: &mut [f32]) {
    let n4 = w.len();
    let (mut x0, mut x1, mut w0, mut w1) = (0, n4, 0, n4);
    let (mut y0, mut y1) = (0, 2 * n4);
    while x0 < x1 {
        x1 -= 1;
        w1 -= 1;
        let (uz, vz, uw, vw) = (x[x0], x[x1], w[w0], w[w1]);
        (x0, w0) = (x0 + 1, w0 + 1);
        y[y0] = uz.re * uw.im - uz.im * uw.re;
        y[y1 - 1] = uz.re * uw.re + uz.im * uw.im;
        y[y1 - 2] = vz.re * vw.im - vz.im * vw.re;
        y[y0 + 1] = vz.re * vw.re + vz.im * vw.im;
        (y0, y1) = (y0 + 2, y1 - 2);
    }
}

/// `imdct_window`: overlap-adds the inverse transform `x` (ns) with the delayed `d` (nd,
/// updated) into `y` (ns samples).
fn imdct_window(dt: usize, sr: usize, x: &[f32], d: &mut [f32], y: &mut [f32]) {
    // The C's four pointer walks as zipped forward/reversed slices (a = nd − n4; each
    // element's operations as the C's; the first two read `d`, the last two only write it).
    let win = LC3_MDCT_WIN[dt][sr].expect("window for (dt, sr)");
    let (n4, nd) = (ns(dt, sr) >> 1, nd(dt, sr));
    let a = nd - n4;
    let (y, d) = (&mut y[..2 * n4], &mut d[..nd]);

    // k < a: y[a-1-k] = d[a-1-k] − x[k]·w[3n4+k]; y[a+k] = d[a+k] + x[k]·w[3n4-1-k]
    let (ylo, yhi) = y[..2 * a].split_at_mut(a);
    let ys = ylo.iter_mut().rev().zip(yhi.iter_mut());
    let ds = d[..a].iter().rev().zip(&d[a..2 * a]);
    let ws = win[3 * n4..3 * n4 + a]
        .iter()
        .zip(win[3 * n4 - a..3 * n4].iter().rev());
    for (((y0, y1), (&d0, &d1)), (&xk, (&wa, &wb))) in ys.zip(ds).zip(x[..a].iter().zip(ws)) {
        *y0 = d0 - xk * wa;
        *y1 = d1 + xk * wb;
    }
    // k < nd − 2a: y[2a+k] = d[2a+k] + x[a+k]·w[3n4-a-1-k]
    let n2 = nd - 2 * a;
    let ws = win[3 * n4 - a - n2..3 * n4 - a].iter().rev();
    for ((yk, &dk), (&xk, &wk)) in y[2 * a..nd]
        .iter_mut()
        .zip(&d[2 * a..nd])
        .zip(x[a..a + n2].iter().zip(ws))
    {
        *yk = dk + xk * wk;
    }
    // k < 2n4 − nd: y[nd+k] = x[n4+k]·w[2n4-1-k]; d[nd-1-k] = x[n4+k]·w[k]
    let n3 = 2 * n4 - nd;
    let ws = win[2 * n4 - n3..2 * n4].iter().rev().zip(&win[..n3]);
    let os = y[nd..].iter_mut().zip(d[nd - n3..].iter_mut().rev());
    for ((yk, dk), (&xk, (&wa, &wb))) in os.zip(x[n4..n4 + n3].iter().zip(ws)) {
        *yk = xk * wa;
        *dk = xk * wb;
    }
    // k < a: d[k] = x[3n4-nd+k]·w[nd-1-k]; d[2a-1-k] = x[3n4-nd+k]·w[2n4-nd+k]
    let (dlo, dhi) = d[..2 * a].split_at_mut(a);
    let os = dlo.iter_mut().zip(dhi.iter_mut().rev());
    let ws = win[nd - a..nd]
        .iter()
        .rev()
        .zip(&win[2 * n4 - nd..2 * n4 - nd + a]);
    let xs = &x[3 * n4 - nd..3 * n4 - nd + a];
    for ((d0, d1), (&xk, (&wa, &wb))) in os.zip(xs.iter().zip(ws)) {
        *d0 = xk * wa;
        *d1 = xk * wb;
    }
}

/// `rescale`: scales the first `n & !3` values (the C's unrolled loop drops the remainder).
fn rescale(x: &mut [f32], n: usize, f: f32) {
    for v in &mut x[..n & !3] {
        *v *= f;
    }
}

/// `lc3_mdct_forward`: `x` ns samples and `d` nd delayed ones (updated) → `y` ns coefficients,
/// scaled to the destination rate `sr_dst`.
pub(crate) fn mdct_forward(
    dt: usize,
    sr: usize,
    sr_dst: usize,
    x: &[f32],
    d: &mut [f32],
    y: &mut [f32],
) {
    let rot = LC3_MDCT_ROT[dt][sr].expect("rotation for (dt, sr)");
    let (ns, ns_dst) = (ns(dt, sr), ns(dt, sr_dst));
    let mut u = [0f32; MAX_NS];
    let mut a = [Complex::default(); MAX_NS / 2];
    let mut b = [Complex::default(); MAX_NS / 2];

    mdct_window(dt, sr, x, d, &mut u);
    mdct_pre_fft(rot, &u, &mut a);
    let z = if fft(&mut a, &mut b, ns / 2) { &b } else { &a };
    mdct_post_fft(rot, z, y);

    if ns != ns_dst {
        rescale(y, ns_dst, libm::sqrtf(ns_dst as f32 / ns as f32));
    }
}

/// `lc3_mdct_inverse`: `x` ns coefficients (from rate `sr_src`) and `d` nd delayed samples
/// (updated) → `y` ns samples.
pub(crate) fn mdct_inverse(
    dt: usize,
    sr: usize,
    sr_src: usize,
    x: &[f32],
    d: &mut [f32],
    y: &mut [f32],
) {
    let rot = LC3_MDCT_ROT[dt][sr].expect("rotation for (dt, sr)");
    let (ns, ns_src) = (ns(dt, sr), ns(dt, sr_src));
    let mut u = [0f32; MAX_NS];
    let mut a = [Complex::default(); MAX_NS / 2];
    let mut b = [Complex::default(); MAX_NS / 2];

    imdct_pre_fft(rot, x, &mut a);
    let z = if fft(&mut a, &mut b, ns / 2) { &b } else { &a };
    imdct_post_fft(rot, z, &mut u);

    if ns != ns_src {
        rescale(&mut u, ns, libm::sqrtf(ns as f32 / ns_src as f32));
    }
    imdct_window(dt, sr, &u, d, y);
}
