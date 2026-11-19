//! Data and state-machine side of an LE Audio BAP Unicast Server that exposes only
//! Source ASEs (a microphone).
//!
//! The crate performs no I/O. The integrator wires it to GATT and HCI:
//!
//! * [`pacs`]: Published Audio Capabilities Service characteristic values.
//! * [`ascs`]: Audio Stream Control Service ASE values and the ASE Control Point
//!   state machine, plus the CIS/ACL events it depends on.
//! * [`adv`]: BAP/CAP announcement service data for extended advertising.
//! * [`uuid`]: 16-bit service and characteristic UUIDs.
//! * [`context`]: Context Type bit values.
//!
//! Specifications: PACS v1.0, ASCS v1.0.1, BAP v1.0, Bluetooth Assigned Numbers
//! (Generic Audio section).
#![no_std]

pub mod adv;
pub mod ascs;
pub mod context;
mod ltv;
pub mod pacs;
pub mod uuid;
