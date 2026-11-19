//! Bandwidth detector (liblc3 `bwdet.c`): the widest band with energy, unless a drop above a
//! narrower cut-off holds it.

use crate::bits::Writer;
use crate::common::{DT_2M5, DT_5M, DT_7M5};

/// `lc3_bwdet_run`: the bandwidth (as `enum lc3_bandwidth`) from the band energies `e`.
pub(crate) fn run(dt: usize, sr: usize, e: &[f32]) -> usize {
    // [bwn - 1][bw]: band regions [start, end)
    const BWS_2M5: [[[usize; 2]; 4]; 4] = [
        [[24, 35], [0, 0], [0, 0], [0, 0]],
        [[24, 33], [35, 40], [0, 0], [0, 0]],
        [[24, 32], [33, 39], [39, 43], [0, 0]],
        [[22, 30], [31, 36], [37, 41], [41, 44]],
    ];
    const BWS_5M: [[[usize; 2]; 4]; 4] = [
        [[39, 50], [0, 0], [0, 0], [0, 0]],
        [[35, 45], [47, 52], [0, 0], [0, 0]],
        [[34, 43], [44, 50], [50, 54], [0, 0]],
        [[32, 41], [42, 47], [48, 52], [52, 55]],
    ];
    const BWS_7M5: [[[usize; 2]; 4]; 4] = [
        [[51, 64], [0, 0], [0, 0], [0, 0]],
        [[45, 56], [58, 64], [0, 0], [0, 0]],
        [[42, 52], [53, 59], [60, 64], [0, 0]],
        [[40, 49], [51, 56], [57, 61], [61, 64]],
    ];
    const BWS_10M: [[[usize; 2]; 4]; 4] = [
        [[53, 64], [0, 0], [0, 0], [0, 0]],
        [[47, 57], [59, 64], [0, 0], [0, 0]],
        [[44, 53], [54, 60], [60, 64], [0, 0]],
        [[41, 50], [51, 56], [57, 61], [61, 64]],
    ];
    const L: [[usize; 4]; 4] = [[4, 4, 3, 1], [4, 4, 3, 1], [4, 4, 3, 2], [4, 4, 3, 1]];

    // Stage 1: the bw0 candidate
    let mut bw0 = 0;
    let bwn = sr;
    if bwn <= bw0 {
        return bwn;
    }
    let bwr = &match dt {
        DT_2M5 => BWS_2M5,
        DT_5M => BWS_5M,
        DT_7M5 => BWS_7M5,
        _ => BWS_10M,
    }[bwn - 1];
    for (bw, &[is, ie]) in bwr.iter().enumerate().take(bwn) {
        let mut se = e[is];
        for &v in &e[is + 1..ie] {
            se += v;
        }
        if se >= ((10 << usize::from(bw == 0)) * (ie - is)) as f32 {
            bw0 = bw + 1;
        }
    }

    // Stage 2: a drop above the cut-off, Tc = 10^(n/10), n = 15, 23, 20, 20
    let mut hold = bw0 >= bwn;
    if !hold {
        let (i0, l) = (bwr[bw0][0], L[dt][bw0]);
        let tc = [
            31.62277660_f64 as f32,
            199.52623150_f64 as f32,
            100.0,
            100.0,
        ][bw0];
        let mut i = i0 + 1 - l;
        while !hold && i <= i0 + 1 {
            hold = e[i - l] > tc * e[i];
            i += 1;
        }
    }
    if hold { bw0 } else { bwn }
}

/// `lc3_bwdet_put_bw`.
pub(crate) fn put_bw(bits: &mut Writer, sr: usize, bw: usize) {
    let nbits = get_nbits(sr);
    if nbits > 0 {
        bits.put_bits(bw as u32, nbits);
    }
}

/// `lc3_bwdet_get_nbits`.
pub(crate) fn get_nbits(sr: usize) -> i32 {
    i32::from(sr > 0) + i32::from(sr > 1) + i32::from(sr > 3)
}
