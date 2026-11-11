use super::kinematics::{advance, bearing, distance, turn_towards, wrap_pi};
use super::{MovementCommand, MovementStrategy};
use rand::rngs::StdRng;
use rand::{Rng, SeedableRng};
use std::f64::consts::PI;

/// Fraction of `radius` beyond which the ship turns back towards the center.
const RETURN_THRESHOLD: f64 = 0.8;
/// Largest turning radius allowed while returning, as a fraction of `radius`.
/// With 0.1 the worst-case U-turn started at 80 % of the radius peaks at about
/// 91 % of it, so the ship never leaves the circle.
const MAX_RETURN_TURN_RADIUS: f64 = 0.1;

/// Parameters of a [`DriftMovement`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DriftParams {
    /// Mean speed, m/s.
    pub speed: f64,
    /// Relative speed variation (0.2 = speed varies within ±20 %).
    pub speed_jitter: f64,
    /// Maximum turn rate, rad/s.
    pub max_turn_rate: f64,
    /// Seconds between new random course/speed choices.
    pub change_every: f64,
    /// Center of the allowed area, meters.
    pub center: (f64, f64),
    /// Radius of the allowed area, meters.
    pub radius: f64,
}

/// # Drift movement (smooth, reproducible random walk)
///
/// Every `change_every` seconds the ship picks, with a seeded RNG:
/// - a new target course = current course + uniform(-Δ, Δ), with
///   `Δ = min(max_turn_rate · change_every, π)`, so the turn always fits in
///   one interval, and
/// - a new target speed = `speed · (1 + uniform(-speed_jitter, speed_jitter))`.
///
/// The course turns towards the target at most `max_turn_rate` rad/s and the
/// speed ramps linearly to the new target over the interval, so there are
/// never jumps (unlike `random`).
///
/// Containment: while the ship is farther than 80 % of `radius` from `center`
/// it steers towards the center. If `max_turn_rate` is too low to turn around
/// inside the remaining 20 %, the return turn uses the minimum rate that does
/// (`max_speed / (0.1 · radius)`); otherwise `max_turn_rate` is used.
///
/// Like `waypoints`, it integrates its own position from `initial_position`
/// with the same kinematics as `Ship::update`.
pub struct DriftMovement {
    params: DriftParams,
    time_delta: u64,
    rng: StdRng,
    position: (f64, f64),
    heading: f64,
    target_heading: f64,
    speed: f64,
    speed_from: f64,
    speed_to: f64,
    ticks_per_change: u64,
    tick_in_interval: u64,
    return_turn_rate: f64,
}

/// Seed used when the config does not give one: derived from the ship id
/// (its index in `initial_ships`) with SplitMix64, so it is stable between runs.
pub fn seed_for_ship(id: u64) -> u64 {
    let mut z = id.wrapping_add(0x9E37_79B9_7F4A_7C15);
    z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
    z ^ (z >> 31)
}

impl DriftMovement {
    /// - `params`: see [`DriftParams`]
    /// - `initial_position`: the ship's initial position, meters
    /// - `time_delta`: emission interval, ms
    /// - `seed`: RNG seed (same seed + same parameters = same trajectory)
    pub fn new(
        params: DriftParams,
        initial_position: (f64, f64),
        time_delta: u64,
        seed: u64,
    ) -> Self {
        let mut rng = StdRng::seed_from_u64(seed);
        let heading = wrap_pi(rng.gen::<f64>() * 2.0 * PI);
        let speed = Self::random_speed(&params, &mut rng);

        let dt = time_delta as f64 / 1000.0;
        let ticks_per_change = ((params.change_every / dt).round() as u64).max(1);
        let max_speed = params.speed * (1.0 + params.speed_jitter);
        let return_turn_rate = params
            .max_turn_rate
            .max(max_speed / (MAX_RETURN_TURN_RADIUS * params.radius));

        DriftMovement {
            params,
            time_delta,
            rng,
            position: initial_position,
            heading,
            target_heading: heading,
            speed,
            speed_from: speed,
            speed_to: speed,
            ticks_per_change,
            tick_in_interval: 0,
            return_turn_rate,
        }
    }

    fn random_speed(params: &DriftParams, rng: &mut StdRng) -> f64 {
        // Always draw, even with jitter 0, so the random stream does not
        // depend on the parameter values.
        let u: f64 = rng.gen();
        params.speed * (1.0 + params.speed_jitter * (2.0 * u - 1.0))
    }
}

impl MovementStrategy for DriftMovement {
    fn next_movement(&mut self) -> MovementCommand {
        let dt = self.time_delta as f64 / 1000.0;

        if self.tick_in_interval == 0 {
            let max_change = (self.params.max_turn_rate * self.params.change_every).min(PI);
            let u: f64 = self.rng.gen();
            self.target_heading = wrap_pi(self.heading + (2.0 * u - 1.0) * max_change);
            self.speed_from = self.speed;
            self.speed_to = Self::random_speed(&self.params, &mut self.rng);
        }
        self.tick_in_interval += 1;

        let progress = self.tick_in_interval as f64 / self.ticks_per_change as f64;
        self.speed = self.speed_from + (self.speed_to - self.speed_from) * progress;

        let returning =
            distance(self.position, self.params.center) > RETURN_THRESHOLD * self.params.radius;
        let turn_rate = if returning {
            // Keep the inward course as target after re-entering the inner area.
            self.target_heading = bearing(self.position, self.params.center);
            self.return_turn_rate
        } else {
            self.params.max_turn_rate
        };
        self.heading = turn_towards(self.heading, self.target_heading, turn_rate * dt);

        self.position = advance(self.position, self.speed, self.heading, self.time_delta);

        if self.tick_in_interval >= self.ticks_per_change {
            self.tick_in_interval = 0;
        }

        MovementCommand {
            angle: self.heading,
            speed: self.speed,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::super::kinematics::test_util::simulate;
    use super::*;
    use crate::simulation::emitters::Ship;

    const TEN_MINUTES: usize = 30_000; // ticks of 20 ms

    fn params() -> DriftParams {
        DriftParams {
            speed: 5.0,
            speed_jitter: 0.25,
            max_turn_rate: 0.1,
            change_every: 15.0,
            center: (0.0, 0.0),
            radius: 700.0,
        }
    }

    fn trajectory(p: DriftParams, start: (f64, f64), seed: u64) -> Vec<(f64, f64)> {
        let mut m = DriftMovement::new(p, start, 20, seed);
        simulate(&mut m, start, 20, TEN_MINUTES)
            .into_iter()
            .map(|s| s.position)
            .collect()
    }

    #[test]
    fn same_seed_same_trajectory() {
        let a = trajectory(params(), (100.0, -50.0), 42);
        let b = trajectory(params(), (100.0, -50.0), 42);
        assert_eq!(a, b); // bit-for-bit
    }

    #[test]
    fn different_seed_different_trajectory() {
        let a = trajectory(params(), (100.0, -50.0), 42);
        let b = trajectory(params(), (100.0, -50.0), 43);
        let last = distance(*a.last().unwrap(), *b.last().unwrap());
        assert!(last > 10.0, "trajectories end only {last} m apart");
    }

    #[test]
    fn derived_seed_is_stable_and_differs_per_ship() {
        assert_eq!(seed_for_ship(3), seed_for_ship(3));
        assert_ne!(seed_for_ship(0), seed_for_ship(1));
        let a = trajectory(params(), (0.0, 0.0), seed_for_ship(0));
        let b = trajectory(params(), (0.0, 0.0), seed_for_ship(1));
        assert_ne!(a, b);
    }

    #[test]
    fn stays_inside_radius_for_ten_minutes() {
        let cases = [
            params(),
            // Small area and a very low turn rate: the return turn must be
            // faster than max_turn_rate for the ship to stay inside.
            DriftParams {
                speed: 8.0,
                speed_jitter: 0.5,
                max_turn_rate: 0.01,
                change_every: 5.0,
                center: (300.0, -200.0),
                radius: 150.0,
            },
        ];
        for p in cases {
            for seed in 0..25 {
                let start = (p.center.0 + 0.5 * p.radius, p.center.1);
                for pos in trajectory(p, start, seed) {
                    let d = distance(pos, p.center);
                    assert!(
                        d <= p.radius,
                        "seed {seed}: {d} m from center, radius {}",
                        p.radius
                    );
                }
            }
        }
    }

    #[test]
    fn course_and_speed_change_smoothly() {
        let p = params();
        let mut m = DriftMovement::new(p, (0.0, 0.0), 20, 7);
        let samples = simulate(&mut m, (0.0, 0.0), 20, TEN_MINUTES);
        let max_rate = p.max_turn_rate.max(m.return_turn_rate);
        let mut turned = 0.0;
        for pair in samples.windows(2) {
            let step = wrap_pi(pair[1].command.angle - pair[0].command.angle);
            assert!(step.abs() <= max_rate * 0.02 + 1e-12, "heading jump {step}");
            turned += step.abs();

            let speed = pair[1].command.speed;
            assert!(speed >= p.speed * (1.0 - p.speed_jitter) - 1e-9);
            assert!(speed <= p.speed * (1.0 + p.speed_jitter) + 1e-9);
            // Linear ramp: at most |Δspeed| / ticks_per_change per tick.
            let ds = (speed - pair[0].command.speed).abs();
            assert!(ds <= 2.0 * p.speed * p.speed_jitter / m.ticks_per_change as f64 + 1e-9);
        }
        // It does wander (not a straight line).
        assert!(turned > 5.0, "only turned {turned} rad in 10 min");
    }

    #[test]
    fn internal_position_matches_ship() {
        let p = params();
        let mut m = DriftMovement::new(p, (10.0, 20.0), 20, 99);
        let mut ship = Ship {
            id: 0,
            position: (10.0, 20.0),
            emission_interval: 20,
            movement_strategy: Box::new(DriftMovement::new(p, (10.0, 20.0), 20, 99)),
        };
        for _ in 0..TEN_MINUTES {
            let _ = m.next_movement();
            ship.update();
            assert_eq!(m.position, ship.position);
        }
    }
}
