//! Simulation module.
//!
//! This module contains the simulation environment, emitters and movement strategies.
//!

mod config;
mod emitters;
mod environment;
mod movement;
mod simulation;

use config::build_ship_from_config;
use emitters::{Emitter, Ship};
use movement::{
    seed_for_ship, CircleMovement, DriftMovement, DriftParams, LineMovement, MovementStrategy,
    RandomMovement, RouteLeg, RouteMovement, StationaryMovement, WaypointsMovement, ZigzagMovement,
};

pub use config::{ShipConfig, SimulationConfig};
pub use environment::Wave;
pub use simulation::Simulation;
