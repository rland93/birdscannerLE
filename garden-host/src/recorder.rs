//! WAV recording: `<recordings>/<mic>/YYYY-MM-DD/HH-MM-SS.wav`, 16-bit mono at
//! the stream rate, a new file every wall-clock hour.

use std::fs::{self, File};
use std::io::{self, BufWriter};
use std::path::{Path, PathBuf};

use chrono::{NaiveDateTime, Timelike};

/// The header is rewritten (so the file plays even if we crash) after this
/// much audio.
const FLUSH_EVERY_SECONDS: u32 = 2;

fn hound_err(e: hound::Error) -> io::Error {
    match e {
        hound::Error::IoError(e) => e,
        other => io::Error::other(other),
    }
}

pub struct Recorder {
    mic_dir: PathBuf,
    rate: u32,
    writer: Option<hound::WavWriter<BufWriter<File>>>,
    path: PathBuf,
    /// The date and hour the current file belongs to.
    hour: (chrono::NaiveDate, u32),
    unflushed: u32,
}

impl std::fmt::Debug for Recorder {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Recorder")
            .field("path", &self.path)
            .field("rate", &self.rate)
            .finish()
    }
}

impl Recorder {
    /// Opens the first file under `mic_dir`, named for `now`.
    pub fn start(mic_dir: &Path, rate: u32, now: NaiveDateTime) -> io::Result<Recorder> {
        let (writer, path) = open(mic_dir, rate, now)?;
        Ok(Recorder {
            mic_dir: mic_dir.to_path_buf(),
            rate,
            writer: Some(writer),
            path,
            hour: (now.date(), now.hour()),
            unflushed: 0,
        })
    }

    pub fn path(&self) -> &Path {
        &self.path
    }

    /// Appends `pcm`, first starting a new file if the hour changed.
    pub fn write(&mut self, pcm: &[i16], now: NaiveDateTime) -> io::Result<()> {
        if (now.date(), now.hour()) != self.hour {
            self.close()?;
            let (writer, path) = open(&self.mic_dir, self.rate, now)?;
            self.writer = Some(writer);
            self.path = path;
            self.hour = (now.date(), now.hour());
        }
        let Some(w) = self.writer.as_mut() else {
            return Err(io::Error::other("recorder closed"));
        };
        let mut s16 = w.get_i16_writer(pcm.len() as u32);
        for &s in pcm {
            s16.write_sample(s);
        }
        s16.flush().map_err(hound_err)?;
        self.unflushed += pcm.len() as u32;
        if self.unflushed >= self.rate * FLUSH_EVERY_SECONDS {
            w.flush().map_err(hound_err)?;
            self.unflushed = 0;
        }
        Ok(())
    }

    /// Completes the current file's header.
    pub fn finish(mut self) -> io::Result<PathBuf> {
        self.close()?;
        Ok(std::mem::take(&mut self.path))
    }

    fn close(&mut self) -> io::Result<()> {
        match self.writer.take() {
            Some(w) => w.finalize().map_err(hound_err),
            None => Ok(()),
        }
    }
}

impl Drop for Recorder {
    fn drop(&mut self) {
        if let Err(e) = self.close() {
            tracing::warn!("closing {}: {e}", self.path.display());
        }
    }
}

fn open(
    mic_dir: &Path,
    rate: u32,
    now: NaiveDateTime,
) -> io::Result<(hound::WavWriter<BufWriter<File>>, PathBuf)> {
    let dir = mic_dir.join(now.format("%Y-%m-%d").to_string());
    fs::create_dir_all(&dir)?;
    let stem = now.format("%H-%M-%S").to_string();
    let mut path = dir.join(format!("{stem}.wav"));
    let mut n = 1;
    while path.exists() {
        path = dir.join(format!("{stem}-{n}.wav"));
        n += 1;
    }
    let spec = hound::WavSpec {
        channels: 1,
        sample_rate: rate,
        bits_per_sample: 16,
        sample_format: hound::SampleFormat::Int,
    };
    let writer = hound::WavWriter::create(&path, spec).map_err(hound_err)?;
    tracing::info!("recording to {}", path.display());
    Ok((writer, path))
}

#[cfg(test)]
mod tests {
    use super::*;
    use chrono::NaiveDate;

    fn at(h: u32, m: u32, s: u32) -> NaiveDateTime {
        NaiveDate::from_ymd_opt(2026, 6, 21)
            .unwrap()
            .and_hms_opt(h, m, s)
            .unwrap()
    }

    #[test]
    fn rotates_on_the_hour() {
        let dir = tempfile::tempdir().unwrap();
        let mut r = Recorder::start(dir.path(), 16_000, at(5, 59, 58)).unwrap();
        let first = r.path().to_path_buf();
        assert_eq!(first, dir.path().join("2026-06-21/05-59-58.wav"));
        r.write(&[1; 160], at(5, 59, 59)).unwrap();
        r.write(&[2; 160], at(6, 0, 0)).unwrap();
        r.write(&[3; 160], at(6, 0, 0)).unwrap();
        let second = r.finish().unwrap();
        assert_eq!(second, dir.path().join("2026-06-21/06-00-00.wav"));
        let a = hound::WavReader::open(&first).unwrap();
        assert_eq!((a.spec().sample_rate, a.len()), (16_000, 160));
        let b = hound::WavReader::open(&second).unwrap();
        assert_eq!(b.len(), 320);
    }

    #[test]
    fn same_second_gets_a_suffix() {
        let dir = tempfile::tempdir().unwrap();
        let a = Recorder::start(dir.path(), 24_000, at(7, 0, 0))
            .unwrap()
            .finish()
            .unwrap();
        let b = Recorder::start(dir.path(), 24_000, at(7, 0, 0))
            .unwrap()
            .finish()
            .unwrap();
        assert_ne!(a, b);
        assert!(b.ends_with("2026-06-21/07-00-00-1.wav"));
    }
}
