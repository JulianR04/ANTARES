use super::kinematics::{advance, bearing, distance, wrap_pi};
use super::{MovementCommand, MovementStrategy};
use std::f64::consts::FRAC_PI_2;

/// # Waypoints movement
///
/// Sails through a list of points (meters, same frame as `initial_position`)
/// at constant speed, turning towards the next point at most `turn_rate` rad/s.
/// A point counts as reached when the ship is within `arrival_radius` meters of
/// it; then the next point becomes the target.
///
/// - `repeat = true`: after the last point it heads back to the first one.
/// - `repeat = false`: after the last point the ship stops (speed 0).
///
/// The initial heading points straight at the first waypoint.
///
/// `MovementStrategy` does not receive the ship position, so this strategy
/// integrates its own copy from `initial_position` with exactly the same
/// kinematics as `Ship::update` (the two stay bit-identical).
///
/// If the target is inside the ship's minimum turning circle
/// (radius `speed / turn_rate`), turning at the maximum rate would orbit it
/// forever. In that case the ship holds its course until the point is outside
/// the circle, and then commits to a full-rate turn towards it until it is
/// aligned (a point outside the turning circle always has a tangent, so the
/// turn ends pointing at it). This way `arrival_radius` can be smaller than the
/// turning radius.
pub struct WaypointsMovement {
    speed: f64,
    points: Vec<(f64, f64)>,
    turn_rate: f64,
    arrival_radius: f64,
    repeat: bool,
    time_delta: u64,
    position: (f64, f64),
    heading: f64,
    target: usize,
    finished: bool,
    /// Side of the committed turn (> 0 left, < 0 right), if turning.
    committed_turn: Option<f64>,
}

impl WaypointsMovement {
    /// - `speed`: m/s
    /// - `points`: waypoints in meters
    /// - `turn_rate`: rad/s (must be > 0)
    /// - `arrival_radius`: m
    /// - `repeat`: loop over the points
    /// - `initial_position`: the ship's initial position, in meters
    /// - `time_delta`: emission interval in ms
    pub fn new(
        speed: f64,
        points: Vec<(f64, f64)>,
        turn_rate: f64,
        arrival_radius: f64,
        repeat: bool,
        initial_position: (f64, f64),
        time_delta: u64,
    ) -> Self {
        let heading = points
            .first()
            .map(|&p| bearing(initial_position, p))
            .unwrap_or(0.0);

        WaypointsMovement {
            speed,
            finished: points.is_empty(),
            points,
            turn_rate,
            arrival_radius,
            repeat,
            time_delta,
            position: initial_position,
            heading,
            target: 0,
            committed_turn: None,
        }
    }

    /// Index of the waypoint currently targeted.
    #[cfg(test)]
    fn target_index(&self) -> usize {
        self.target
    }

    /// Moves on to the next waypoint(s) if the current one has been reached.
    fn update_target(&mut self) {
        // Bounded loop: if every point is within the arrival radius we stop
        // skipping after one full pass.
        for _ in 0..self.points.len() {
            if distance(self.position, self.points[self.target]) > self.arrival_radius {
                return;
            }
            self.target += 1;
            self.committed_turn = None;
            if self.target == self.points.len() {
                if self.repeat {
                    self.target = 0;
                } else {
                    self.finished = true;
                    return;
                }
            }
        }
    }

    /// True if `point` lies inside the minimum turning circle on the given side
    /// (`side` > 0: left, < 0: right).
    fn inside_turning_circle(&self, point: (f64, f64), side: f64) -> bool {
        let radius = self.speed / self.turn_rate;
        let center_dir = self.heading + FRAC_PI_2.copysign(side);
        let center = (
            self.position.0 + radius * center_dir.cos(),
            self.position.1 + radius * center_dir.sin(),
        );
        distance(center, point) < radius
    }
}

impl MovementStrategy for WaypointsMovement {
    fn next_movement(&mut self) -> MovementCommand {
        if !self.finished {
            self.update_target();
        }
        if self.finished {
            return MovementCommand {
                angle: self.heading,
                speed: 0.0,
            };
        }

        let target = self.points[self.target];
        let desired = bearing(self.position, target);
        let error = wrap_pi(desired - self.heading);
        let max_step = self.turn_rate * self.time_delta as f64 / 1000.0;

        if error.abs() <= max_step {
            // Aligned (or nearly): point straight at the target.
            self.heading = desired;
            self.committed_turn = None;
        } else {
            if self.committed_turn.is_none() && !self.inside_turning_circle(target, error) {
                self.committed_turn = Some(error.signum());
            }
            // Otherwise the target is inside the turning circle: hold course.
            if let Some(side) = self.committed_turn {
                self.heading = wrap_pi(self.heading + max_step.copysign(side));
            }
        }

        self.position = advance(self.position, self.speed, self.heading, self.time_delta);

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

    const SQUARE: [(f64, f64); 4] = [(200.0, 0.0), (200.0, 200.0), (0.0, 200.0), (0.0, 0.0)];

    /// Runs the strategy and records (tick, point index, distance at arrival)
    /// every time a waypoint is reached.
    fn arrivals(m: &mut WaypointsMovement, ticks: usize) -> Vec<(usize, usize, f64)> {
        let mut out = Vec::new();
        for k in 0..ticks {
            if m.finished {
                break;
            }
            let (position, before) = (m.position, m.target_index());
            let _ = m.next_movement();
            // `target_index()` is points.len() once a non-repeating route ends.
            // Several points may be reached in the same tick if they are
            // closer to each other than the arrival radius.
            let (n, after) = (m.points.len(), m.target_index());
            let mut i = before;
            while i != after {
                out.push((k, i, distance(position, m.points[i])));
                i += 1;
                if i == n && after != n {
                    i = 0;
                }
            }
        }
        out
    }

    #[test]
    fn reaches_every_point_in_order_and_repeats() {
        let mut m =
            WaypointsMovement::new(5.0, SQUARE.to_vec(), 0.3, 15.0, true, (-50.0, -50.0), 20);
        // 10 minutes simulated.
        let events = arrivals(&mut m, 30_000);
        assert!(
            events.len() >= 13,
            "only {} arrivals: {events:?}",
            events.len()
        );
        for (i, &(_, index, dist)) in events.iter().enumerate() {
            assert_eq!(index, i % 4, "out of order: {events:?}");
            assert!(dist <= 15.0, "arrived at point {index} from {dist} m");
        }
        // Once in the loop (the first lap starts from a different approach),
        // successive laps (point 0 -> point 0) take the same time.
        let lap2 = events[8].0 - events[4].0;
        let lap3 = events[12].0 - events[8].0;
        assert!(lap2.abs_diff(lap3) <= 2, "laps of {lap2} and {lap3} ticks");
    }

    #[test]
    fn respects_turn_rate() {
        let mut m =
            WaypointsMovement::new(5.0, SQUARE.to_vec(), 0.3, 15.0, true, (-50.0, -50.0), 20);
        let samples = simulate(&mut m, (-50.0, -50.0), 20, 20_000);
        for pair in samples.windows(2) {
            let step = wrap_pi(pair[1].command.angle - pair[0].command.angle);
            assert!(step.abs() <= 0.3 * 0.02 + 1e-12);
        }
    }

    #[test]
    fn no_repeat_stops_at_last_point() {
        let mut m =
            WaypointsMovement::new(5.0, SQUARE.to_vec(), 0.3, 15.0, false, (-50.0, -50.0), 20);
        let samples = simulate(&mut m, (-50.0, -50.0), 20, 30_000);
        let last = samples.last().unwrap();
        assert!(m.finished);
        assert_eq!(last.command.speed, 0.0);
        assert!(distance(last.position, (0.0, 0.0)) <= 15.0);
    }

    #[test]
    fn tight_points_are_reached_without_orbiting() {
        // Turning radius = 6 / 0.2 = 30 m, arrival radius only 2 m, points
        // 20 m apart with sharp turns: pure pursuit would circle forever.
        let points = vec![(20.0, 0.0), (20.0, 20.0), (0.0, 0.0), (0.0, 20.0)];
        let mut m = WaypointsMovement::new(6.0, points, 0.2, 2.0, true, (0.0, 0.0), 20);
        let events = arrivals(&mut m, 30_000);
        assert!(
            events.len() >= 12,
            "only {} arrivals in 10 min",
            events.len()
        );
        for (i, &(_, index, dist)) in events.iter().enumerate() {
            assert_eq!(index, i % 4);
            assert!(dist <= 2.0);
        }
    }

    #[test]
    fn internal_position_matches_ship() {
        let start = (-50.0, -50.0);
        let mut m = WaypointsMovement::new(5.0, SQUARE.to_vec(), 0.3, 15.0, true, start, 20);
        let mut ship = Ship {
            id: 0,
            position: start,
            emission_interval: 20,
            movement_strategy: Box::new(WaypointsMovement::new(
                5.0,
                SQUARE.to_vec(),
                0.3,
                15.0,
                true,
                start,
                20,
            )),
        };
        for _ in 0..20_000 {
            let _ = m.next_movement();
            ship.update();
            assert_eq!(m.position, ship.position);
        }
    }

    #[test]
    fn random_routes_always_complete() {
        use rand::rngs::StdRng;
        use rand::{Rng, SeedableRng};
        // 50 random (but fixed) scenarios, including arrival radii much
        // smaller than the turning radius: each must do 2 laps in 30 min.
        let mut rng = StdRng::seed_from_u64(2026);
        for case in 0..50 {
            let n = rng.gen_range(2..6);
            let points: Vec<(f64, f64)> = (0..n)
                .map(|_| (rng.gen_range(-300.0..300.0), rng.gen_range(-300.0..300.0)))
                .collect();
            let speed = rng.gen_range(2.0..10.0);
            let turn_rate = rng.gen_range(0.05..0.5);
            let arrival = rng.gen_range(1.0..30.0);
            let mut m = WaypointsMovement::new(
                speed,
                points.clone(),
                turn_rate,
                arrival,
                true,
                (0.0, 0.0),
                20,
            );
            let events = arrivals(&mut m, 90_000);
            assert!(
                events.len() >= 2 * n,
                "case {case}: {} arrivals; points {points:?} v={speed} w={turn_rate} r={arrival}",
                events.len()
            );
            for (i, &(_, index, dist)) in events.iter().enumerate() {
                assert_eq!(index, i % n);
                assert!(dist <= arrival);
            }
        }
    }

    #[test]
    fn empty_points_stays_still() {
        let mut m = WaypointsMovement::new(5.0, vec![], 0.3, 15.0, true, (1.0, 2.0), 20);
        assert_eq!(m.next_movement().speed, 0.0);
    }
}
