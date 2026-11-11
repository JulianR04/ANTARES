use super::{
    seed_for_ship, CircleMovement, DriftMovement, DriftParams, LineMovement, MovementStrategy,
    RandomMovement, RouteLeg, RouteMovement, Ship, StationaryMovement, WaypointsMovement,
    ZigzagMovement,
};
use serde::de::{Error, Unexpected};
use serde::{Deserialize, Deserializer, Serialize};

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct SimulationConfig {
    pub emission_interval: u64,
    pub initial_ships: Vec<ShipConfig>,
    pub controller_bind_addr: String,
}

impl Default for SimulationConfig {
    fn default() -> Self {
        SimulationConfig {
            emission_interval: 20,
            initial_ships: Vec::new(),
            controller_bind_addr: "0.0.0.0:17394".into(),
        }
    }
}

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct ShipConfig {
    pub initial_position: (f64, f64),
    #[serde(flatten)]
    pub movement: MovementType,
}

#[derive(Debug, Serialize, Deserialize, Clone)]
#[serde(tag = "type", rename_all = "lowercase")]
pub enum MovementType {
    Line {
        angle: f64,
        speed: f64,
    },
    Circle {
        radius: f64,
        speed: f64,
    },
    Random {
        max_speed: f64,
    },
    Stationary,
    /// S-shaped course:
    /// `heading(t) = angle + amplitude · sin(2π · t / period + phase)`.
    /// The default `phase = π/2` centers the track on the base line.
    Zigzag {
        #[serde(deserialize_with = "non_negative")]
        speed: f64,
        #[serde(deserialize_with = "finite")]
        angle: f64,
        #[serde(deserialize_with = "finite")]
        amplitude: f64,
        #[serde(deserialize_with = "positive")]
        period: f64,
        #[serde(default = "default_zigzag_phase", deserialize_with = "finite")]
        phase: f64,
    },
    /// Straight legs separated by turns at a constant turn rate.
    Route {
        #[serde(deserialize_with = "non_negative")]
        speed: f64,
        #[serde(deserialize_with = "finite")]
        angle: f64,
        #[serde(default = "default_turn_rate", deserialize_with = "positive")]
        turn_rate: f64,
        #[serde(default = "default_repeat")]
        repeat: bool,
        legs: Vec<RouteLegConfig>,
    },
    /// Sails through a list of points with a limited turn rate.
    Waypoints {
        #[serde(deserialize_with = "non_negative")]
        speed: f64,
        #[serde(deserialize_with = "non_empty_points")]
        points: Vec<(f64, f64)>,
        #[serde(default = "default_turn_rate", deserialize_with = "positive")]
        turn_rate: f64,
        #[serde(deserialize_with = "positive")]
        arrival_radius: f64,
        #[serde(default = "default_repeat")]
        repeat: bool,
    },
    /// Smooth, seeded random walk kept inside a circle.
    Drift {
        #[serde(deserialize_with = "non_negative")]
        speed: f64,
        #[serde(deserialize_with = "fraction")]
        speed_jitter: f64,
        #[serde(deserialize_with = "positive")]
        max_turn_rate: f64,
        #[serde(deserialize_with = "positive")]
        change_every: f64,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        seed: Option<u64>,
        center: (f64, f64),
        #[serde(deserialize_with = "positive")]
        radius: f64,
    },
}

/// One leg of a `route`: straight for `duration` seconds, then turn `turn`
/// radians (positive = left / counter-clockwise, negative = right / clockwise).
#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct RouteLegConfig {
    #[serde(deserialize_with = "non_negative")]
    pub duration: f64,
    #[serde(deserialize_with = "finite")]
    pub turn: f64,
}

fn default_zigzag_phase() -> f64 {
    std::f64::consts::FRAC_PI_2
}

fn default_turn_rate() -> f64 {
    0.2
}

fn default_repeat() -> bool {
    true
}

/// Reads an `f64` (TOML integers are accepted too) and checks it.
fn checked_f64<'de, D: Deserializer<'de>>(
    deserializer: D,
    valid: fn(f64) -> bool,
    expected: &'static str,
) -> Result<f64, D::Error> {
    let value = f64::deserialize(deserializer)?;
    if value.is_finite() && valid(value) {
        Ok(value)
    } else {
        Err(D::Error::invalid_value(Unexpected::Float(value), &expected))
    }
}

fn finite<'de, D: Deserializer<'de>>(d: D) -> Result<f64, D::Error> {
    checked_f64(d, |_| true, "a finite number")
}

fn non_negative<'de, D: Deserializer<'de>>(d: D) -> Result<f64, D::Error> {
    checked_f64(d, |v| v >= 0.0, "a number >= 0")
}

fn positive<'de, D: Deserializer<'de>>(d: D) -> Result<f64, D::Error> {
    checked_f64(d, |v| v > 0.0, "a number > 0")
}

fn fraction<'de, D: Deserializer<'de>>(d: D) -> Result<f64, D::Error> {
    checked_f64(d, |v| (0.0..=1.0).contains(&v), "a number between 0 and 1")
}

fn non_empty_points<'de, D: Deserializer<'de>>(d: D) -> Result<Vec<(f64, f64)>, D::Error> {
    let points = Vec::<(f64, f64)>::deserialize(d)?;
    if points.is_empty() {
        Err(D::Error::invalid_length(0, &"at least one point"))
    } else {
        Ok(points)
    }
}

pub fn build_ship_from_config(id: u64, config: ShipConfig, emission_interval: u64) -> Ship {
    let movement_strategy: Box<dyn MovementStrategy> = match config.movement {
        MovementType::Line { angle, speed } => Box::new(LineMovement::new(angle, speed)),
        MovementType::Circle { radius, speed } => {
            Box::new(CircleMovement::new(radius, speed, emission_interval))
        }
        MovementType::Random { max_speed } => Box::new(RandomMovement::new(max_speed)),
        MovementType::Stationary => Box::new(StationaryMovement {}),
        MovementType::Zigzag {
            speed,
            angle,
            amplitude,
            period,
            phase,
        } => Box::new(ZigzagMovement::new(
            speed,
            angle,
            amplitude,
            period,
            phase,
            emission_interval,
        )),
        MovementType::Route {
            speed,
            angle,
            turn_rate,
            repeat,
            legs,
        } => {
            let legs = legs
                .into_iter()
                .map(|leg| RouteLeg {
                    duration: leg.duration,
                    turn: leg.turn,
                })
                .collect();
            Box::new(RouteMovement::new(
                speed,
                angle,
                turn_rate,
                repeat,
                legs,
                emission_interval,
            ))
        }
        MovementType::Waypoints {
            speed,
            points,
            turn_rate,
            arrival_radius,
            repeat,
        } => Box::new(WaypointsMovement::new(
            speed,
            points,
            turn_rate,
            arrival_radius,
            repeat,
            config.initial_position,
            emission_interval,
        )),
        MovementType::Drift {
            speed,
            speed_jitter,
            max_turn_rate,
            change_every,
            seed,
            center,
            radius,
        } => Box::new(DriftMovement::new(
            DriftParams {
                speed,
                speed_jitter,
                max_turn_rate,
                change_every,
                center,
                radius,
            },
            config.initial_position,
            emission_interval,
            seed.unwrap_or_else(|| seed_for_ship(id)),
        )),
    };

    Ship {
        id,
        position: config.initial_position,
        emission_interval,
        movement_strategy,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::{AntaresConfig, Config};
    use std::f64::consts::PI;

    /// Wraps a single ship table in a minimal but complete config file, so the
    /// ship goes through the same path as a real file (Config -> flatten -> tag).
    fn parse_ship(body: &str) -> Result<MovementType, String> {
        let text = format!(
            r#"
[antares.simulation]
emission_interval = 20
controller_bind_addr = "127.0.0.1:0"

[[antares.simulation.initial_ships]]
{body}

[antares.radar]
bind_addr = "127.0.0.1:0"

[antares.radar.detector]
range = 1000.0
speed = 0.0
angle = 0.0
start_coordinates = [4.0, -72.0]

[antares.radar.broadcast]
type = "tcp"
"#
        );
        toml::from_str::<Config>(&text)
            .map(|c| c.antares.simulation.initial_ships[0].movement.clone())
            .map_err(|e| e.to_string())
    }

    fn run(ship: &mut Ship, ticks: usize) -> Vec<(f64, f64)> {
        (0..ticks)
            .map(|_| {
                ship.update();
                ship.position
            })
            .collect()
    }

    // ---------- Compatibility with the existing configs ----------

    #[test]
    fn example_config_still_parses_to_the_same_ships() {
        let config: Config = toml::from_str(include_str!("../../config.example.toml")).unwrap();
        let sim = &config.antares.simulation;
        assert_eq!(sim.emission_interval, 20);
        let debug: Vec<String> = sim
            .initial_ships
            .iter()
            .map(|s| format!("{:?} {:?}", s.initial_position, s.movement))
            .collect();
        assert_eq!(
            debug,
            [
                "(0.0, 0.0) Line { angle: 0.785, speed: 5.0 }",
                "(30.0, -30.0) Circle { radius: 20.0, speed: 4.0 }",
                "(-20.0, 20.0) Random { max_speed: 10.0 }",
                "(50.0, 50.0) Stationary",
            ]
        );
    }

    #[test]
    fn example_config_ships_move_as_before() {
        let config: Config = toml::from_str(include_str!("../../config.example.toml")).unwrap();
        let mut ships = config
            .antares
            .simulation
            .initial_ships
            .into_iter()
            .enumerate()
            .map(|(i, s)| build_ship_from_config(i as u64, s, 20));

        // line: closed form x = v·cos(a)·t, y = v·sin(a)·t.
        let mut line = ships.next().unwrap();
        for (k, p) in run(&mut line, 1_000).into_iter().enumerate() {
            let t = (k + 1) as f64 * 0.02;
            assert!((p.0 - 5.0 * 0.785f64.cos() * t).abs() < 1e-9);
            assert!((p.1 - 5.0 * 0.785f64.sin() * t).abs() < 1e-9);
        }

        // circle: same recurrence as the original CircleMovement, bit for bit.
        let mut circle = ships.next().unwrap();
        let (mut x, mut y, mut a) = (30.0f64, -30.0f64, 0.0f64);
        for p in run(&mut circle, 1_000) {
            a = (a + 4.0 * (20.0 / 1000.0) / 20.0) % (2.0 * PI);
            x += 4.0 * a.cos() * 20.0 / 1000.0;
            y += 4.0 * a.sin() * 20.0 / 1000.0;
            assert_eq!(p, (x, y));
        }

        // random: each step is shorter than max_speed · dt.
        let mut random = ships.next().unwrap();
        let mut prev = random.position;
        for p in run(&mut random, 1_000) {
            assert!((p.0 - prev.0).hypot(p.1 - prev.1) <= 10.0 * 0.02 + 1e-12);
            prev = p;
        }

        // stationary: never moves.
        let mut stationary = ships.next().unwrap();
        assert!(run(&mut stationary, 1_000)
            .iter()
            .all(|&p| p == (50.0, 50.0)));
    }

    #[test]
    fn assets_config_ships_still_parse() {
        // assets/config.toml has no [antares] wrapper (it is not loadable with
        // --config, before or after this change); its content maps to AntaresConfig.
        let config: AntaresConfig =
            toml::from_str(include_str!("../../assets/config.toml")).unwrap();
        assert_eq!(config.simulation.initial_ships.len(), 4);
    }

    #[test]
    fn movimientos_config_has_one_ship_of_each_new_type() {
        let config: Config = toml::from_str(include_str!("../../config.movimientos.toml")).unwrap();
        let kinds: Vec<&str> = config
            .antares
            .simulation
            .initial_ships
            .iter()
            .map(|s| match s.movement {
                MovementType::Zigzag { .. } => "zigzag",
                MovementType::Route { .. } => "route",
                MovementType::Waypoints { .. } => "waypoints",
                MovementType::Drift { .. } => "drift",
                _ => "other",
            })
            .collect();
        assert_eq!(kinds, ["zigzag", "route", "waypoints", "drift"]);
    }

    // ---------- Integers vs floats through flatten + internal tag ----------

    #[test]
    fn integers_and_floats_are_accepted_alike() {
        let pairs = [
            (
                r#"type = "zigzag"
                initial_position = [0, 0]
                speed = 5
                angle = 0
                amplitude = 1
                period = 30
                phase = 1"#,
                r#"type = "zigzag"
                initial_position = [0.0, 0.0]
                speed = 5.0
                angle = 0.0
                amplitude = 1.0
                period = 30.0
                phase = 1.0"#,
            ),
            (
                r#"type = "route"
                initial_position = [0, 0]
                speed = 5
                angle = 1
                turn_rate = 1
                legs = [{ duration = 10, turn = 2 }, { duration = 3, turn = -1 }]"#,
                r#"type = "route"
                initial_position = [0.0, 0.0]
                speed = 5.0
                angle = 1.0
                turn_rate = 1.0
                legs = [{ duration = 10.0, turn = 2.0 }, { duration = 3.0, turn = -1.0 }]"#,
            ),
            (
                r#"type = "waypoints"
                initial_position = [0, 0]
                speed = 5
                points = [[100, 0], [0, 100]]
                turn_rate = 1
                arrival_radius = 10"#,
                r#"type = "waypoints"
                initial_position = [0.0, 0.0]
                speed = 5.0
                points = [[100.0, 0.0], [0.0, 100.0]]
                turn_rate = 1.0
                arrival_radius = 10.0"#,
            ),
            (
                r#"type = "drift"
                initial_position = [0, 0]
                speed = 5
                speed_jitter = 0
                max_turn_rate = 1
                change_every = 15
                seed = 7
                center = [0, 0]
                radius = 500"#,
                r#"type = "drift"
                initial_position = [0.0, 0.0]
                speed = 5.0
                speed_jitter = 0.0
                max_turn_rate = 1.0
                change_every = 15.0
                seed = 7
                center = [0.0, 0.0]
                radius = 500.0"#,
            ),
        ];
        for (ints, floats) in pairs {
            let a = parse_ship(ints).unwrap_or_else(|e| panic!("integers rejected:\n{e}"));
            let b = parse_ship(floats).unwrap_or_else(|e| panic!("floats rejected:\n{e}"));
            println!("{a:?}");
            assert_eq!(format!("{a:?}"), format!("{b:?}"));
        }
    }

    #[test]
    fn defaults_are_applied() {
        let zigzag = parse_ship(
            r#"type = "zigzag"
            initial_position = [0, 0]
            speed = 5
            angle = 0
            amplitude = 0.5
            period = 30"#,
        )
        .unwrap();
        assert!(
            matches!(zigzag, MovementType::Zigzag { phase, .. } if phase == std::f64::consts::FRAC_PI_2),
            "{zigzag:?}"
        );

        let route = parse_ship(
            r#"type = "route"
            initial_position = [0, 0]
            speed = 5
            angle = 0
            legs = []"#,
        )
        .unwrap();
        assert_eq!(
            format!("{route:?}"),
            "Route { speed: 5.0, angle: 0.0, turn_rate: 0.2, repeat: true, legs: [] }"
        );

        let waypoints = format!(
            "{:?}",
            parse_ship(
                r#"type = "waypoints"
                initial_position = [0, 0]
                speed = 5
                points = [[1, 2]]
                arrival_radius = 10"#,
            )
            .unwrap()
        );
        assert!(waypoints.contains("turn_rate: 0.2") && waypoints.contains("repeat: true"));

        let drift = format!(
            "{:?}",
            parse_ship(
                r#"type = "drift"
                initial_position = [0, 0]
                speed = 5
                speed_jitter = 0.2
                max_turn_rate = 0.1
                change_every = 10
                center = [0, 0]
                radius = 500"#,
            )
            .unwrap()
        );
        assert!(drift.contains("seed: None"));
    }

    // ---------- Readable errors ----------

    #[test]
    fn missing_field_error_is_readable() {
        let cases = [
            (
                r#"type = "zigzag"
                initial_position = [0, 0]
                speed = 5
                angle = 0
                amplitude = 0.5"#,
                "missing field `period`",
            ),
            (
                r#"type = "waypoints"
                initial_position = [0, 0]
                speed = 5
                points = [[1, 2]]"#,
                "missing field `arrival_radius`",
            ),
            (
                r#"type = "zigzg"
                initial_position = [0, 0]"#,
                "unknown variant `zigzg`",
            ),
        ];
        for (body, expected) in cases {
            let err = parse_ship(body).unwrap_err();
            println!("---\n{err}");
            assert!(err.contains(expected), "{err}");
        }
    }

    #[test]
    fn invalid_values_are_rejected() {
        let cases = [
            (
                "type = \"zigzag\"\ninitial_position = [0, 0]\nspeed = 5\nangle = 0\namplitude = 0.5\nperiod = 0",
                "expected a number > 0",
            ),
            (
                "type = \"route\"\ninitial_position = [0, 0]\nspeed = 5\nangle = 0\nturn_rate = -1\nlegs = []",
                "expected a number > 0",
            ),
            (
                "type = \"route\"\ninitial_position = [0, 0]\nspeed = 5\nangle = 0\nlegs = [{ duration = -3, turn = 1 }]",
                "expected a number >= 0",
            ),
            (
                "type = \"waypoints\"\ninitial_position = [0, 0]\nspeed = 5\npoints = []\narrival_radius = 10",
                "at least one point",
            ),
            (
                "type = \"drift\"\ninitial_position = [0, 0]\nspeed = 5\nspeed_jitter = 1.5\nmax_turn_rate = 0.1\nchange_every = 10\ncenter = [0, 0]\nradius = 500",
                "expected a number between 0 and 1",
            ),
            (
                "type = \"drift\"\ninitial_position = [0, 0]\nspeed = nan\nspeed_jitter = 0.1\nmax_turn_rate = 0.1\nchange_every = 10\ncenter = [0, 0]\nradius = 500",
                "expected a number >= 0",
            ),
        ];
        for (body, expected) in cases {
            let err = parse_ship(body).unwrap_err();
            println!("---\n{err}");
            assert!(err.contains(expected), "{err}");
        }
    }

    // ---------- Seeds ----------

    #[test]
    fn drift_without_seed_uses_ship_index() {
        let movement = parse_ship(
            r#"type = "drift"
            initial_position = [0, 0]
            speed = 5
            speed_jitter = 0.2
            max_turn_rate = 0.1
            change_every = 10
            center = [0, 0]
            radius = 500"#,
        )
        .unwrap();
        let trajectory = |id| {
            let config = ShipConfig {
                initial_position: (0.0, 0.0),
                movement: movement.clone(),
            };
            run(&mut build_ship_from_config(id, config, 20), 5_000)
        };
        assert_eq!(trajectory(2), trajectory(2));
        assert_ne!(trajectory(2), trajectory(3));
    }
}
