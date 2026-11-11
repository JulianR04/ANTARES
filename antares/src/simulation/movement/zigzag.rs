use super::{MovementCommand, MovementStrategy};
use std::f64::consts::PI;

/// # Zigzag (S-shaped) movement
///
/// The heading oscillates sinusoidally around a base course:
///
/// `heading(t) = angle + amplitude · sin(2π · t / period + phase)`
///
/// where `t` is the simulated time since the ship was created
/// (`t = tick · emission_interval`). Speed is constant.
///
/// With `phase = π/2` (the config default) the track is centered on the base
/// line through the initial position: the lateral offset swings between
/// `±speed · amplitude · period / (2π)` meters at most (slightly less for large
/// amplitudes). With `phase = 0` the whole track lies on one side of the base
/// line (to the left for a positive amplitude), between 0 and twice that value.
pub struct ZigzagMovement {
    speed: f64,
    angle: f64,
    amplitude: f64,
    period: f64,
    phase: f64,
    time_delta: u64,
    tick: u64,
}

impl ZigzagMovement {
    /// - `speed`: m/s
    /// - `angle`: base course, rad
    /// - `amplitude`: maximum deviation from the base course, rad
    /// - `period`: seconds per full oscillation (must be > 0)
    /// - `phase`: initial phase of the oscillation, rad (π/2 = centered track)
    /// - `time_delta`: emission interval in ms
    pub fn new(
        speed: f64,
        angle: f64,
        amplitude: f64,
        period: f64,
        phase: f64,
        time_delta: u64,
    ) -> Self {
        ZigzagMovement {
            speed,
            angle,
            amplitude,
            period,
            phase,
            time_delta,
            tick: 0,
        }
    }
}

impl MovementStrategy for ZigzagMovement {
    fn next_movement(&mut self) -> MovementCommand {
        // Time is derived from the tick counter (not accumulated) so it does not drift.
        let t = self.tick as f64 * self.time_delta as f64 / 1000.0;
        self.tick += 1;

        MovementCommand {
            angle: self.angle + self.amplitude * (2.0 * PI * t / self.period + self.phase).sin(),
            speed: self.speed,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::super::kinematics::test_util::simulate;
    use super::*;
    use std::f64::consts::FRAC_PI_2;

    /// Signed lateral offset from the base line (positive = left of the course).
    fn lateral(p: (f64, f64), start: (f64, f64), angle: f64) -> f64 {
        -(p.0 - start.0) * angle.sin() + (p.1 - start.1) * angle.cos()
    }

    fn along(p: (f64, f64), start: (f64, f64), angle: f64) -> f64 {
        (p.0 - start.0) * angle.cos() + (p.1 - start.1) * angle.sin()
    }

    #[test]
    fn default_phase_is_centered_and_bounded_by_amplitude() {
        let (speed, angle, amplitude, period) = (6.0, 0.7, 0.5, 30.0);
        let start = (-400.0, 120.0);
        let mut m = ZigzagMovement::new(speed, angle, amplitude, period, FRAC_PI_2, 20);
        // 10 full periods (1500 ticks per period at 20 ms).
        let samples = simulate(&mut m, start, 20, 15_000);

        let bound = speed * amplitude * period / (2.0 * PI);
        let (mut min_lat, mut max_lat, mut sum_lat) = (0.0f64, 0.0f64, 0.0);
        let mut last_along = f64::NEG_INFINITY;
        for s in &samples {
            let lat = lateral(s.position, start, angle);
            assert!(
                lat.abs() <= bound + 1e-6,
                "lateral offset {lat}, bound ±{bound}"
            );
            min_lat = min_lat.min(lat);
            max_lat = max_lat.max(lat);
            sum_lat += lat;

            // Heading never deviates more than the amplitude.
            assert!((s.command.angle - angle).abs() <= amplitude + 1e-12);

            // Always making progress along the base course (amplitude < π/2).
            let a = along(s.position, start, angle);
            assert!(a >= last_along);
            last_along = a;
        }
        // It really oscillates to both sides, reaching > 90 % of the bound...
        assert!(
            max_lat > 0.9 * bound,
            "max lateral {max_lat}, bound {bound}"
        );
        assert!(
            min_lat < -0.9 * bound,
            "min lateral {min_lat}, bound {bound}"
        );
        // ...symmetrically around the base line.
        assert!((max_lat + min_lat).abs() < 0.01 * bound);
        let mean = sum_lat / samples.len() as f64;
        assert!(mean.abs() < 0.01 * bound, "mean lateral offset {mean}");
    }

    #[test]
    fn default_phase_crosses_base_line_every_half_period() {
        let (speed, angle, amplitude, period) = (5.0, 0.0, 0.4, 20.0);
        let mut m = ZigzagMovement::new(speed, angle, amplitude, period, FRAC_PI_2, 20);
        let samples = simulate(&mut m, (0.0, 0.0), 20, 5_001);
        let peak = 0.9 * speed * amplitude * period / (2.0 * PI);
        // At the half period the discretization error is below one step.
        let dt_step = speed * 0.02;
        // 20 s = 1000 ticks: on the base line every half period, at +peak
        // after a quarter and at -peak after three quarters.
        for n in 0..5 {
            let at = |tick: usize| lateral(samples[n * 1000 + tick].position, (0.0, 0.0), angle);
            assert!(at(1000).abs() < 1e-6, "period {n}: lateral {}", at(1000));
            assert!(
                at(500).abs() < dt_step,
                "half period {n}: lateral {}",
                at(500)
            );
            assert!(at(250) > peak, "quarter period {n}: lateral {}", at(250));
            assert!(at(750) < -peak, "3/4 period {n}: lateral {}", at(750));
        }
    }

    #[test]
    fn phase_zero_stays_on_one_side() {
        let (speed, angle, amplitude, period) = (6.0, 0.7, 0.5, 30.0);
        let start = (-400.0, 120.0);
        let mut m = ZigzagMovement::new(speed, angle, amplitude, period, 0.0, 20);
        let samples = simulate(&mut m, start, 20, 15_000);
        let bound = speed * amplitude * period / PI;
        let mut max_lat: f64 = 0.0;
        for s in &samples {
            let lat = lateral(s.position, start, angle);
            assert!((-1e-6..=bound + 1e-6).contains(&lat), "lateral {lat}");
            max_lat = max_lat.max(lat);
        }
        assert!(max_lat > 0.9 * bound);
    }

    #[test]
    fn heading_follows_formula() {
        for phase in [0.0, FRAC_PI_2, -1.0] {
            let mut m = ZigzagMovement::new(3.0, 1.0, 0.3, 12.0, phase, 20);
            for k in 0..2_000u64 {
                let t = k as f64 * 0.02;
                let cmd = m.next_movement();
                let expected = 1.0 + 0.3 * (2.0 * PI * t / 12.0 + phase).sin();
                assert!((cmd.angle - expected).abs() < 1e-12);
                assert_eq!(cmd.speed, 3.0);
            }
        }
    }
}
