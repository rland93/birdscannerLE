//! Per-slot level metering and DC removal.

/// Full scale of a 24-bit sample. Slots arrive as Q31 and are shifted down by
/// 8 bits; MEMS mics with fewer bits (e.g. 18-bit SPH0645) pad with zeros.
const FULL_SCALE: f32 = 8_388_608.0;

/// Pole of the DC blocker; puts the -3 dB corner at ~25 Hz for 31.25 kHz.
const DC_POLE: f32 = 0.995;

/// Levels below this are reported as this (log of zero).
const FLOOR_DBFS: f32 = -140.0;

pub fn dbfs(amplitude: f32) -> f32 {
    if amplitude <= 0.0 {
        return FLOOR_DBFS;
    }
    (20.0 * libm::log10f(amplitude / FULL_SCALE)).max(FLOOR_DBFS)
}

/// Window summary for one I2S slot.
pub enum Report {
    /// Every word in the window had the same value: nothing is driving the
    /// slot (pull-down reads 0) or SDIN is stuck.
    Constant(i32),
    Signal {
        rms_dbfs: f32,
        peak_dbfs: f32,
        /// Mean of the raw samples in 24-bit counts (MEMS mics have an offset).
        dc: i32,
    },
}

impl Report {
    pub fn rms_dbfs(&self) -> Option<f32> {
        match self {
            Report::Constant(_) => None,
            Report::Signal { rms_dbfs, .. } => Some(*rms_dbfs),
        }
    }
}

impl defmt::Format for Report {
    fn format(&self, f: defmt::Formatter) {
        match self {
            Report::Constant(raw) => defmt::write!(f, "no data (constant {=i32:#x})", *raw),
            Report::Signal { rms_dbfs, peak_dbfs, dc } => defmt::write!(
                f,
                "rms {=i32} dBFS, peak {=i32} dBFS, dc {=i32}",
                libm::roundf(*rms_dbfs) as i32,
                libm::roundf(*peak_dbfs) as i32,
                *dc,
            ),
        }
    }
}

/// DC-blocks one slot and accumulates window and per-block statistics.
pub struct Slot {
    x1: f32,
    y1: f32,
    primed: bool,

    // Window statistics.
    n: u32,
    raw_sum: i64,
    raw_min: i32,
    raw_max: i32,
    sum_sq: f64,
    peak: f32,

    // Current block.
    block_sum_sq: f32,
    block_peak: f32,
}

impl Slot {
    pub const fn new() -> Self {
        Self {
            x1: 0.0,
            y1: 0.0,
            primed: false,
            n: 0,
            raw_sum: 0,
            raw_min: i32::MAX,
            raw_max: i32::MIN,
            sum_sq: 0.0,
            peak: 0.0,
            block_sum_sq: 0.0,
            block_peak: 0.0,
        }
    }

    /// Feed one Q31 slot word; returns the DC-free sample in 24-bit counts.
    pub fn push(&mut self, q31: i32) -> f32 {
        self.raw_min = self.raw_min.min(q31);
        self.raw_max = self.raw_max.max(q31);
        let raw = q31 >> 8;
        self.raw_sum += i64::from(raw);
        self.n += 1;

        // Exact in f32: |raw| < 2^23.
        let x = raw as f32;
        if !self.primed {
            // Start the blocker settled on the first sample instead of
            // ringing down from a full-scale step.
            self.x1 = x;
            self.primed = true;
        }
        let y = x - self.x1 + DC_POLE * self.y1;
        self.x1 = x;
        self.y1 = y;

        self.block_sum_sq += y * y;
        self.block_peak = self.block_peak.max(libm::fabsf(y));
        y
    }

    /// Close the current block; returns its peak in dBFS.
    pub fn end_block(&mut self) -> f32 {
        self.sum_sq += f64::from(self.block_sum_sq);
        self.peak = self.peak.max(self.block_peak);
        let peak = self.block_peak;
        self.block_sum_sq = 0.0;
        self.block_peak = 0.0;
        dbfs(peak)
    }

    /// Summarise and reset the window.
    pub fn take_report(&mut self) -> Report {
        let report = if self.n == 0 || self.raw_min == self.raw_max {
            Report::Constant(self.raw_min)
        } else {
            let rms = libm::sqrt(self.sum_sq / f64::from(self.n)) as f32;
            Report::Signal {
                rms_dbfs: dbfs(rms),
                peak_dbfs: dbfs(self.peak),
                dc: (self.raw_sum / i64::from(self.n)) as i32,
            }
        };
        self.n = 0;
        self.raw_sum = 0;
        self.raw_min = i32::MAX;
        self.raw_max = i32::MIN;
        self.sum_sq = 0.0;
        self.peak = 0.0;
        report
    }
}
