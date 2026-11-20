//! The persisted settings: which mics we know, what they are called and what
//! the user wants them to do. Stored as JSON so it can be edited by hand.

use std::fs;
use std::io;
use std::path::{Path, PathBuf};

use garden_link::{Addr, Quality};
use serde::{Deserialize, Serialize};

/// Where the config file and the recordings live.
#[derive(Clone, Debug)]
pub struct Paths {
    pub config_file: PathBuf,
    pub recordings: PathBuf,
}

impl Paths {
    /// Everything under one directory (the `--data-dir` flag).
    pub fn in_dir(dir: &Path) -> Paths {
        Paths {
            config_file: dir.join("config.json"),
            recordings: dir.join("recordings"),
        }
    }

    /// The platform's config and data directories (e.g. `~/.config/garden-host`
    /// and `~/.local/share/garden-host` on Linux, `%APPDATA%` on Windows,
    /// `~/Library/Application Support` on macOS).
    pub fn platform() -> anyhow::Result<Paths> {
        let dirs = directories::ProjectDirs::from("org", "nrf53bird", "garden-host")
            .ok_or_else(|| anyhow::anyhow!("no home directory; pass --data-dir"))?;
        Ok(Paths {
            config_file: dirs.config_dir().join("config.json"),
            recordings: dirs.data_dir().join("recordings"),
        })
    }
}

#[derive(Serialize, Deserialize, Clone, Debug, PartialEq, Eq)]
pub struct MicConfig {
    #[serde(with = "addr_text")]
    pub addr: Addr,
    pub name: String,
    pub quality: Quality,
    /// Keep the stream running even when nobody listens or records.
    #[serde(default)]
    pub stream: bool,
    /// Record to WAV whenever the mic streams (which this also asks for).
    #[serde(default)]
    pub record: bool,
}

#[derive(Serialize, Deserialize, Clone, Debug, Default, PartialEq, Eq)]
pub struct Config {
    #[serde(default)]
    pub mics: Vec<MicConfig>,
}

impl Config {
    /// Reads `path`; a missing file is an empty config.
    pub fn load(path: &Path) -> anyhow::Result<Config> {
        match fs::read(path) {
            Ok(bytes) => serde_json::from_slice(&bytes)
                .map_err(|e| anyhow::anyhow!("{}: {e}", path.display())),
            Err(e) if e.kind() == io::ErrorKind::NotFound => Ok(Config::default()),
            Err(e) => Err(anyhow::anyhow!("{}: {e}", path.display())),
        }
    }

    /// Writes `path` atomically (a temporary file renamed over it).
    pub fn save(&self, path: &Path) -> io::Result<()> {
        if let Some(dir) = path.parent() {
            fs::create_dir_all(dir)?;
        }
        let tmp = path.with_extension("json.tmp");
        let json = serde_json::to_vec_pretty(self).map_err(io::Error::other)?;
        fs::write(&tmp, json)?;
        fs::rename(&tmp, path)
    }
}

/// `DE:10:EB:CE:60:9E/random`: the display form plus the address type.
pub fn addr_to_text(addr: &Addr) -> String {
    format!(
        "{addr}/{}",
        if addr.kind == 0 { "public" } else { "random" }
    )
}

pub fn addr_from_text(s: &str) -> Option<Addr> {
    let (hex, kind) = s.split_once('/')?;
    let kind = match kind {
        "public" => 0,
        "random" => 1,
        _ => return None,
    };
    let mut bytes = [0u8; 6];
    let mut parts = hex.split(':');
    for b in &mut bytes {
        let p = parts.next()?;
        if p.len() != 2 {
            return None;
        }
        *b = u8::from_str_radix(p, 16).ok()?;
    }
    parts.next().is_none().then_some(Addr { kind, bytes })
}

/// A URL- and id-safe key for an address: `de10ebce609e-r`.
pub fn addr_key(addr: &Addr) -> String {
    let mut s = String::with_capacity(14);
    for b in addr.bytes {
        s.push_str(&format!("{b:02x}"));
    }
    s.push_str(if addr.kind == 0 { "-p" } else { "-r" });
    s
}

pub fn addr_from_key(key: &str) -> Option<Addr> {
    let (hex, kind) = key.split_once('-')?;
    let kind = match kind {
        "p" => 0,
        "r" => 1,
        _ => return None,
    };
    if hex.len() != 12 || !hex.is_ascii() {
        return None;
    }
    let mut bytes = [0u8; 6];
    for (i, b) in bytes.iter_mut().enumerate() {
        *b = u8::from_str_radix(&hex[2 * i..2 * i + 2], 16).ok()?;
    }
    Some(Addr { kind, bytes })
}

/// A mic name as a directory name: letters, digits, space, `-` and `_` kept,
/// anything else replaced, so it is safe on every file system.
pub fn folder_name(name: &str, addr: &Addr) -> String {
    let cleaned: String = name
        .trim()
        .chars()
        .map(|c| {
            if c.is_alphanumeric() || matches!(c, ' ' | '-' | '_') {
                c
            } else {
                '_'
            }
        })
        .collect();
    let cleaned = cleaned.trim().trim_matches('_').to_string();
    if cleaned.is_empty() {
        addr_key(addr)
    } else {
        cleaned
    }
}

mod addr_text {
    use garden_link::Addr;
    use serde::{Deserialize, Deserializer, Serializer, de::Error};

    pub fn serialize<S: Serializer>(addr: &Addr, s: S) -> Result<S::Ok, S::Error> {
        s.serialize_str(&super::addr_to_text(addr))
    }

    pub fn deserialize<'de, D: Deserializer<'de>>(d: D) -> Result<Addr, D::Error> {
        let s = String::deserialize(d)?;
        super::addr_from_text(&s).ok_or_else(|| D::Error::custom(format!("bad address {s:?}")))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const A: Addr = Addr {
        kind: 1,
        bytes: [0xDE, 0x10, 0xEB, 0xCE, 0x60, 0x9E],
    };

    #[test]
    fn addresses_round_trip_as_text_and_key() {
        assert_eq!(addr_to_text(&A), "DE:10:EB:CE:60:9E/random");
        assert_eq!(addr_from_text("DE:10:EB:CE:60:9E/random"), Some(A));
        assert_eq!(addr_key(&A), "de10ebce609e-r");
        assert_eq!(addr_from_key("de10ebce609e-r"), Some(A));
        assert_eq!(addr_from_key("de10ebce609e"), None);
        assert_eq!(addr_from_text("DE:10:EB:CE:60/random"), None);
    }

    #[test]
    fn config_saves_and_loads() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("sub/config.json");
        assert_eq!(Config::load(&path).unwrap(), Config::default());
        let cfg = Config {
            mics: vec![MicConfig {
                addr: A,
                name: "Hedge".into(),
                quality: Quality::High,
                stream: false,
                record: true,
            }],
        };
        cfg.save(&path).unwrap();
        assert_eq!(Config::load(&path).unwrap(), cfg);
        let text = fs::read_to_string(&path).unwrap();
        assert!(text.contains("\"DE:10:EB:CE:60:9E/random\""), "{text}");
    }

    #[test]
    fn folder_names_are_safe() {
        assert_eq!(folder_name("Apple tree", &A), "Apple tree");
        assert_eq!(folder_name("../etc", &A), "etc");
        assert_eq!(folder_name("  ", &A), "de10ebce609e-r");
        assert_eq!(folder_name("a/b:c", &A), "a_b_c");
    }
}
