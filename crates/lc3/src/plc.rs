//! Packet loss concealment (liblc3 `plc.c`): the last spectrum with random signs, attenuated.

use crate::common::ne;

/// `lc3_plc_state_t`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct PlcState {
    pub seed: u16,
    pub count: i32,
    pub alpha: f32,
}

impl PlcState {
    /// `lc3_plc_reset`.
    pub(crate) fn new() -> Self {
        PlcState {
            seed: 24607,
            count: 1,
            alpha: 1.0,
        }
    }

    /// `lc3_plc_suspend`: a frame was received.
    pub(crate) fn suspend(&mut self) {
        self.count = 1;
        self.alpha = 1.0;
    }

    /// `lc3_plc_synthesize`: `y` from the last spectrum `x`.
    pub(crate) fn synthesize(&mut self, dt: usize, sr: usize, x: &[f32], y: &mut [f32]) {
        let mut seed = self.seed;
        let mut alpha = self.alpha;
        alpha *= if self.count < 4 {
            1.0
        } else if self.count < 8 {
            0.9
        } else {
            0.85
        };
        for i in 0..ne(dt, sr) {
            seed = (16831u32 + u32::from(seed) * 12821) as u16;
            y[i] = alpha * if seed & 0x8000 != 0 { -x[i] } else { x[i] };
        }
        self.seed = seed;
        self.alpha = alpha;
        self.count += 1;
    }
}
