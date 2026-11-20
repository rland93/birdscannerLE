//! Host side of the nrf53bird garden microphones: talks to the receiver
//! dongle over USB serial (`garden-link` protocol), decodes the mics' LC3
//! audio, records it and serves a local web UI for listening and control.

pub mod audio;
pub mod config;
pub mod engine;
pub mod link;
pub mod recorder;
pub mod web;
