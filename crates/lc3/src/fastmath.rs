//! liblc3's float approximations (`fastmath.h`): plain float operations in a fixed order, so
//! bit-exact on every CPU.

#[allow(clippy::approx_constant)] // the C's literals, as written
/// `lc3_exp2f`: 2^x from a degree-4 polynomial on |y| < 1/16 and 2^(k/8) steps.
pub(crate) fn exp2f(x: f32) -> f32 {
    // The C's `static const float` arrays: double literals rounded once to float.
    const E: [f32; 8] = [
        1.00000000e+00_f64 as f32,
        1.09050773e+00_f64 as f32,
        1.18920712e+00_f64 as f32,
        1.29683955e+00_f64 as f32,
        1.41421356e+00_f64 as f32,
        1.54221083e+00_f64 as f32,
        1.68179283e+00_f64 as f32,
        1.83400809e+00_f64 as f32,
    ];
    const P: [f32; 4] = [
        1.00448128e-02_f64 as f32,
        5.54563260e-02_f64 as f32,
        2.40228756e-01_f64 as f32,
        6.93147140e-01_f64 as f32,
    ];
    const MAGIC: f32 = 1572864.0; // 0x1.8p20: rounds to multiples of 1/8
    let v = x + MAGIC;
    let k = v.to_bits() as i32;
    let x = x - (v - MAGIC);
    let mut y = P[0] * x;
    y = (y + P[1]) * x;
    y = (y + P[2]) * x;
    y = (y + P[3]) * x;
    y = (y + 1.0) * E[(k & 7) as usize];
    // `y.s += (k >> 3) << 23`, in the C's wrapping 32-bit arithmetic
    f32::from_bits(y.to_bits().wrapping_add(((k >> 3) as u32).wrapping_shl(23)))
}

/// `lc3_ldexpf`: x · 2^exp by the exponent field (zero and subnormals unchanged).
pub(crate) fn ldexpf(x: f32, exp: i32) -> f32 {
    let s = x.to_bits();
    if s & 0x7f80_0000 != 0 {
        f32::from_bits(s.wrapping_add((exp as u32).wrapping_shl(23)))
    } else {
        x
    }
}

/// `lc3_frexpf`: the mantissa in [0.5, 1) and the exponent, by the exponent field.
fn frexpf(x: f32) -> (f32, i32) {
    let u = x.to_bits();
    let e = ((u & 0x7f80_0000) >> 23) as i32;
    (f32::from_bits((u & !0x7f80_0000) | (126 << 23)), e - 126)
}

/// `lc3_log2f`: log2(x), x > 0, from a degree-4 polynomial on the mantissa.
#[allow(clippy::approx_constant)]
pub(crate) fn log2f(x: f32) -> f32 {
    const C: [f32; 5] = [
        -1.29479677_f64 as f32,
        5.11769018_f64 as f32,
        -8.42295281_f64 as f32,
        8.10557963_f64 as f32,
        -3.50567360_f64 as f32,
    ];
    let (x, e) = frexpf(x);
    let mut y = C[0] * x;
    y = (y + C[1]) * x;
    y = (y + C[2]) * x;
    y = (y + C[3]) * x;
    y += C[4];
    e as f32 + y
}

/// `lc3_db_q16`: 10·log10(x²) in Q16, from the exponent and a 32-entry table on the mantissa.
pub(crate) fn db_q16(x: f32) -> i32 {
    // [n][0] = 10·log10(2)·log2(1 + n/32) (n ≥ 16: minus 10·log10(2)/2), Q15; [n][1] the step.
    const T: [[u16; 2]; 32] = [
        [0, 4379],
        [4379, 4248],
        [8627, 4125],
        [12753, 4009],
        [16762, 3899],
        [20661, 3795],
        [24456, 3697],
        [28153, 3603],
        [31755, 3514],
        [35269, 3429],
        [38699, 3349],
        [42047, 3272],
        [45319, 3198],
        [48517, 3128],
        [51645, 3061],
        [54705, 2996],
        [8381, 2934],
        [11315, 2875],
        [14190, 2818],
        [17008, 2763],
        [19772, 2711],
        [22482, 2660],
        [25142, 2611],
        [27754, 2564],
        [30318, 2519],
        [32837, 2475],
        [35312, 2433],
        [37744, 2392],
        [40136, 2352],
        [42489, 2314],
        [44803, 2277],
        [47080, 2241],
    ];
    let u = (x * x).to_bits();
    let e2 = (u >> 22) as i32 - 2 * 127;
    let hi = ((u >> 18) & 0x1f) as usize;
    let lo = ((u >> 2) & 0xffff) as i32;
    e2 * 49321 + i32::from(T[hi][0]) + ((i32::from(T[hi][1]) * lo) >> 16)
}
