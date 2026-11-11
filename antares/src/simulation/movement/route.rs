use super::{MovementCommand, MovementStrategy};

/// One leg of a [`RouteMovement`]: go straight for `duration` seconds, then
/// turn by `turn` radians.
///
/// Sign convention for `turn` (same as every angle in ANTARES, measured
/// counter-clockwise from east):
/// - `turn > 0`: turn left / counter-clockwise (port),
/// - `turn < 0`: turn right / clockwise (starboard),
/// - `turn = 0`: no turn, the next leg starts immediately.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RouteLeg {
    pub duration: f64,
    pub turn: f64,
}

/// Piece of the precomputed schedule: `heading(t) = heading0 + rate · (t - start)`
/// for `t` in `[start, end)`.
#[derive(Debug, Clone, Copy)]
struct Segment {
    start: f64,
    end: f64,
    heading0: f64,
    rate: f64,
}

/// # Route (legs) movement
///
/// Starts on course `angle` and runs the legs in order. Each leg goes straight
/// for `duration` seconds and then turns `turn` radians at a constant angular
/// speed of `turn_rate` rad/s (so the turn lasts `|turn| / turn_rate` seconds)
/// before the next leg starts. Speed is constant.
///
/// - `repeat = true`: after the last leg (including its turn) the route starts
///   again from the first leg, keeping the current heading.
/// - `repeat = false`: after the last leg the ship keeps going straight on the
///   final heading.
///
/// With no legs (or only zero-length legs) it behaves like a straight line.
///
/// The heading is a pure function of the simulated time `t = tick · emission_interval`,
/// so durations and turn rates are exact and do not accumulate rounding errors.
pub struct RouteMovement {
    speed: f64,
    angle: f64,
    repeat: bool,
    segments: Vec<Segment>,
    cycle: f64,
    net_turn: f64,
    time_delta: u64,
    tick: u64,
}

impl RouteMovement {
    /// - `speed`: m/s
    /// - `angle`: initial course, rad
    /// - `turn_rate`: rad/s (must be > 0)
    /// - `repeat`: loop over the legs
    /// - `legs`: durations in s (>= 0), turns in rad
    /// - `time_delta`: emission interval in ms
    pub fn new(
        speed: f64,
        angle: f64,
        turn_rate: f64,
        repeat: bool,
        legs: Vec<RouteLeg>,
        time_delta: u64,
    ) -> Self {
        let mut segments = Vec::new();
        let mut t = 0.0;
        let mut heading = angle;
        for leg in legs {
            if leg.duration > 0.0 {
                segments.push(Segment {
                    start: t,
                    end: t + leg.duration,
                    heading0: heading,
                    rate: 0.0,
                });
                t += leg.duration;
            }
            if leg.turn != 0.0 {
                let turn_time = leg.turn.abs() / turn_rate;
                segments.push(Segment {
                    start: t,
                    end: t + turn_time,
                    heading0: heading,
                    rate: turn_rate.copysign(leg.turn),
                });
                t += turn_time;
                heading += leg.turn;
            }
        }

        RouteMovement {
            speed,
            angle,
            repeat,
            segments,
            cycle: t,
            net_turn: heading - angle,
            time_delta,
            tick: 0,
        }
    }

    /// Heading at simulated time `t` (seconds since the ship was created).
    fn heading_at(&self, t: f64) -> f64 {
        // Segments are only created with a positive length, so cycle > 0 here.
        if self.segments.is_empty() {
            return self.angle;
        }

        let (laps, t_in_cycle) = if self.repeat {
            let laps = (t / self.cycle).floor();
            (laps, t - laps * self.cycle)
        } else if t >= self.cycle {
            return self.angle + self.net_turn;
        } else {
            (0.0, t)
        };

        let index = self
            .segments
            .partition_point(|s| s.end <= t_in_cycle)
            .min(self.segments.len() - 1);
        let segment = self.segments[index];
        let into = (t_in_cycle - segment.start).clamp(0.0, segment.end - segment.start);

        laps * self.net_turn + segment.heading0 + segment.rate * into
    }
}

impl MovementStrategy for RouteMovement {
    fn next_movement(&mut self) -> MovementCommand {
        let t = self.tick as f64 * self.time_delta as f64 / 1000.0;
        self.tick += 1;

        MovementCommand {
            angle: self.heading_at(t),
            speed: self.speed,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::super::kinematics::{test_util::simulate, wrap_pi};
    use super::*;
    use std::f64::consts::{FRAC_PI_2, PI};

    fn leg(duration: f64, turn: f64) -> RouteLeg {
        RouteLeg { duration, turn }
    }

    fn same_angle(a: f64, b: f64) -> bool {
        wrap_pi(a - b).abs() < 1e-9
    }

    /// Straight 10 s, turn +90° (left), straight 5 s, turn -90° (right).
    fn two_leg_route(repeat: bool) -> RouteMovement {
        RouteMovement::new(
            5.0,
            0.3,
            0.2,
            repeat,
            vec![leg(10.0, FRAC_PI_2), leg(5.0, -FRAC_PI_2)],
            20,
        )
    }

    #[test]
    fn respects_durations_and_turn_rate() {
        let turn_time = FRAC_PI_2 / 0.2; // 7.854 s
        let route = two_leg_route(true);
        let h = |t: f64| route.heading_at(t);

        // Straight for exactly 10 s on the initial course.
        assert!(same_angle(h(0.0), 0.3));
        assert!(same_angle(h(9.999), 0.3));
        // Turning left at 0.2 rad/s.
        assert!(same_angle(h(10.0 + 1.0), 0.3 + 0.2));
        assert!(same_angle(h(10.0 + turn_time / 2.0), 0.3 + FRAC_PI_2 / 2.0));
        // Turn complete: new course, held for exactly 5 s.
        let t1 = 10.0 + turn_time;
        assert!(same_angle(h(t1), 0.3 + FRAC_PI_2));
        assert!(same_angle(h(t1 + 4.999), 0.3 + FRAC_PI_2));
        // Turning right at 0.2 rad/s.
        assert!(same_angle(h(t1 + 5.0 + 1.0), 0.3 + FRAC_PI_2 - 0.2));
        // Back on the original course, and the route repeats.
        let cycle = t1 + 5.0 + turn_time;
        assert!(same_angle(h(cycle), 0.3));
        assert!(same_angle(h(cycle + 9.999), 0.3));
        assert!(same_angle(h(cycle + 11.0), 0.5));
    }

    #[test]
    fn per_tick_commands_match_schedule() {
        // cycle = 15 s straight + 2 · 7.854 s turning; simulate 6 full cycles.
        let cycle = 15.0 + PI / 0.2;
        let n = (6.0 * cycle / 0.02) as usize;
        let mut route = two_leg_route(true);
        let samples = simulate(&mut route, (0.0, 0.0), 20, n + 1);
        let (mut straight_ticks, mut turning_ticks) = (0, 0);
        for pair in samples.windows(2) {
            let step = wrap_pi(pair[1].command.angle - pair[0].command.angle);
            // Never turns faster than turn_rate.
            assert!(step.abs() <= 0.2 * 0.02 + 1e-9, "step {step}");
            if step.abs() < 1e-12 {
                straight_ticks += 1;
            } else {
                turning_ticks += 1;
            }
            assert_eq!(pair[0].command.speed, 5.0);
        }
        // Time spent turning / straight matches the leg plan (±1 tick per
        // turn boundary, 12 turns).
        let expected_turning = 6.0 * (PI / 0.2) / 0.02;
        let expected_straight = 6.0 * 15.0 / 0.02;
        assert!(
            (turning_ticks as f64 - expected_turning).abs() <= 25.0,
            "turning ticks {turning_ticks}, expected ~{expected_turning}"
        );
        assert!(
            (straight_ticks as f64 - expected_straight).abs() <= 25.0,
            "straight ticks {straight_ticks}, expected ~{expected_straight}"
        );
    }

    #[test]
    fn square_route_recovers_heading_and_closes() {
        // Four equal legs turning left 90°: a closed, rounded square.
        let legs = vec![leg(30.0, FRAC_PI_2); 4];
        let mut route = RouteMovement::new(4.0, 0.0, 0.25, true, legs, 20);
        let cycle = 4.0 * (30.0 + FRAC_PI_2 / 0.25);
        let ticks_per_cycle = (cycle / 0.02).round() as usize;
        let samples = simulate(&mut route, (10.0, -20.0), 20, 3 * ticks_per_cycle + 1);

        // Heading after each turn.
        for k in 0..4 {
            let t = (k + 1) as f64 * (30.0 + FRAC_PI_2 / 0.25) + 1.0;
            let tick = (t / 0.02).round() as usize;
            assert!(same_angle(
                samples[tick].command.angle,
                (k + 1) as f64 * FRAC_PI_2
            ));
        }
        // After full cycles: original heading and (almost) original position.
        for lap in 1..=3 {
            let s = &samples[lap * ticks_per_cycle];
            assert!(same_angle(s.command.angle, 0.0));
            let (dx, dy) = (s.position.0 - 10.0, s.position.1 + 20.0);
            assert!(dx.hypot(dy) < 0.5, "lap {lap}: off by ({dx}, {dy})");
        }
    }

    #[test]
    fn no_repeat_keeps_final_heading() {
        let route = RouteMovement::new(5.0, 0.0, 0.2, false, vec![leg(10.0, 1.0)], 20);
        assert!(same_angle(route.heading_at(10.0 + 5.0), 1.0));
        assert!(same_angle(route.heading_at(1_000.0), 1.0));
    }

    #[test]
    fn empty_route_is_a_straight_line() {
        let route = RouteMovement::new(5.0, 0.8, 0.2, true, vec![], 20);
        assert_eq!(route.heading_at(0.0), 0.8);
        assert_eq!(route.heading_at(500.0), 0.8);
        let zero = RouteMovement::new(5.0, 0.8, 0.2, true, vec![leg(0.0, 0.0)], 20);
        assert_eq!(zero.heading_at(500.0), 0.8);
    }
}
