//! Data and state-machine side of LE Audio BAP unicast with Source ASEs (a
//! microphone): the Unicast Server that exposes them and the Unicast Client that
//! drives them.
//!
//! The crate performs no I/O. The integrator wires it to GATT and HCI:
//!
//! * [`pacs`]: Published Audio Capabilities Service characteristic values.
//! * [`ascs`]: Audio Stream Control Service ASE values and the ASE Control Point
//!   state machine, plus the CIS/ACL events it depends on.
//! * [`client`]: Unicast Client side: PAC/ASE/Control Point parsers, codec choice,
//!   ASE Control Point PDU builders and QoS derivation.
//! * [`adv`]: BAP/CAP announcement service data for extended advertising.
//! * [`uuid`]: 16-bit service and characteristic UUIDs.
//! * [`context`]: Context Type bit values.
//!
//! Specifications: PACS v1.0, ASCS v1.0.1, BAP v1.0, Bluetooth Assigned Numbers
//! (Generic Audio section).
#![no_std]

pub mod adv;
pub mod ascs;
pub mod client;
mod codec_config;
pub mod context;
mod ltv;
pub mod pacs;
pub mod uuid;
mod wire;
