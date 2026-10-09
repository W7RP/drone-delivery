//! drone-delivery discrete-event core. `docs/spec.md` is the contract; module docs cite the
//! spec sections they implement.

pub mod config;
mod drone;
pub mod event;
pub mod flight;
pub mod fm;
pub mod ids;
pub mod metrics;
pub mod netfile;
pub mod network;
pub mod outcomes;
pub mod output;
mod pad;
pub mod rng;
pub mod scenario;
pub mod trace;
pub mod traffic;
pub mod weather;
pub mod world;
