//! Small helpers shared by the movement strategies.
//!
//! Conventions (same as `Ship::update`): positions are in meters on a local
//! plane (x = east, y = north), angles are in radians measured
//! counter-clockwise from +x (0 = east, π/2 = north), speeds in m/s and the
//! time step is the emission interval in milliseconds.

use std::f64::consts::PI;

/// Advances `position` exactly as `Ship::update` does (same expression and
/// evaluation order, so the result is bit-identical). Strategies that need to
/// know where the ship is integrate their own copy of the position with this.
pub(super) fn advance(
    position: (f64, f64),
    speed: f64,
    angle: f64,
    interval_ms: u64,
) -> (f64, f64) {
    let interval = interval_ms as f64;
    let dx = speed * angle.cos() * interval / 1000.0;
    let dy = speed * angle.sin() * interval / 1000.0;
    (position.0 + dx, position.1 + dy)
}

/// Wraps an angle to the range [-π, π).
pub(super) fn wrap_pi(angle: f64) -> f64 {
    (angle + PI).rem_euclid(2.0 * PI) - PI
}

/// Angle of the vector going from `from` to `to`.
pub(super) fn bearing(from: (f64, f64), to: (f64, f64)) -> f64 {
    (to.1 - from.1).atan2(to.0 - from.0)
}

/// Euclidean distance between two points.
pub(super) fn distance(a: (f64, f64), b: (f64, f64)) -> f64 {
    (b.0 - a.0).hypot(b.1 - a.1)
}

/// Turns `heading` towards `target` by at most `max_step` radians, taking the
/// shortest way round. Returns the new heading wrapped to [-π, π).
pub(super) fn turn_towards(heading: f64, target: f64, max_step: f64) -> f64 {
    let error = wrap_pi(target - heading);
    wrap_pi(heading + error.clamp(-max_step, max_step))
}

#[cfg(test)]
pub(super) mod test_util {
    use super::super::{MovementCommand, MovementStrategy};
    use super::advance;

    /// One simulated tick: the position emitted *before* the update (what the
    /// radar sees) and the command used for the update.
    pub struct Sample {
        pub position: (f64, f64),
        pub command: MovementCommand,
    }

    /// Runs a strategy the same way `Simulation` + `Ship::update` do.
    pub fn simulate(
        strategy: &mut dyn MovementStrategy,
        start: (f64, f64),
        interval_ms: u64,
        ticks: usize,
    ) -> Vec<Sample> {
        let mut position = start;
        let mut samples = Vec::with_capacity(ticks);
        for _ in 0..ticks {
            let command = strategy.next_movement();
            let emitted = position;
            position = advance(position, command.speed, command.angle, interval_ms);
            samples.push(Sample {
                position: emitted,
                command,
            });
        }
        samples
    }
}
