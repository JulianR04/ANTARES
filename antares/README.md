# Antares Simulator

**Antares** is a high-performance simulation engine for naval radar scenarios, written in Rust. It generates dynamic environments with ships, emitters, and wave conditions, and feeds simulated radar data to subscribers via TCP and WebSocket. The simulator is designed for modular experimentation, real-time control, and seamless integration with visualization tools such as [Antares Web](https://thesoftwaredesignlab.github.io/ANTARES/antares-web).

## Prerequisites

Make sure you have [Rust](https://www.rust-lang.org/tools/install) installed. You can install Rust and Cargo using:

```bash
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
```

## Running the Simulator

To launch the simulator with the default settings:

```bash
cargo run
```

To run the simulator with a custom configuration:

```bash
cargo run -- --config path/to/config.toml
```

You can use `config.example.toml` as a starting point.

## Ship Movement Types

Each entry in `[[antares.simulation.initial_ships]]` has an `initial_position = [x, y]` and a `type`.

**Units and conventions:** positions in meters on a local plane (x = east, y = north; the radar's
`start_coordinates` is the origin), angles in radians counter-clockwise from east (0 = east,
1.5708 = north), speeds in m/s, times in seconds, turn rates in rad/s. Integers are accepted wherever a
number is expected (`speed = 5` is the same as `speed = 5.0`).

| `type` | Fields | Behavior |
|---|---|---|
| `line` | `angle`, `speed` | Constant course and speed. |
| `circle` | `radius`, `speed` | Circle of the given radius. |
| `random` | `max_speed` | New random course and speed every tick (not reproducible between runs). |
| `stationary` | – | Does not move. |
| `zigzag` | `speed`, `angle` (base course), `amplitude` (rad), `period` (s), `phase` (rad, default `π/2`) | S-shaped course: `heading(t) = angle + amplitude · sin(2π·t/period + phase)`. With the default phase the track is centered on the base line, within `±speed·amplitude·period/(2π)` m; with `phase = 0` it stays on the left side. |
| `route` | `speed`, `angle` (initial course), `turn_rate` (default `0.2`), `repeat` (default `true`), `legs = [{ duration, turn }, ...]` | Each leg goes straight for `duration` s, then turns `turn` rad at `turn_rate`. `turn > 0` = left (counter-clockwise), `turn < 0` = right (clockwise). With `repeat = false` it keeps the last course. |
| `waypoints` | `speed`, `points = [[x, y], ...]`, `turn_rate` (default `0.2`), `arrival_radius` (m), `repeat` (default `true`) | Steers to each point in turn (turn rate limited); a point is reached within `arrival_radius`. With `repeat = false` the ship stops at the last point. |
| `drift` | `speed`, `speed_jitter` (0–1), `max_turn_rate`, `change_every` (s), `seed` (optional), `center = [x, y]`, `radius` (m) | Smooth random walk: every `change_every` s a new course (reachable within the interval) and a speed within `speed·(1 ± speed_jitter)`. Turns back to `center` beyond 80 % of `radius`. Same `seed` = same trajectory; without `seed` it is derived from the ship's index. |

`config.movimientos.toml` has one ship of each of the last four types.

## Building for Release

For optimized builds suitable for production or deployment:

```bash
cargo build --release
```

## Project Structure

```
antares/
├── src/                     # Source code
│   ├── config.rs            # Global configuration loader
│   ├── controller.rs        # Simulation runtime controller
│   ├── main.rs              # Entry point
│   ├── radar/               # Radar logic and broadcasting
│   │   ├── detector/        # Radar detection and plotting
│   │   ├── tracker/         # Track generation and filtering
│   │   ├── broadcaster/     # Data output via TCP/WebSocket
│   ├── simulation/          # Simulation models
│   │   ├── emitters/        # Emitter and ship definitions
│   │   ├── environment/     # Environmental effects (e.g. waves)
│   │   ├── movement/        # Movement patterns and strategies
│   │   └── simulation.rs    # Simulation loop
├── Cargo.toml               # Project metadata and dependencies
└── Cargo.lock               # Version lockfile
```

## Learn More

For demos, examples, and system overview, visit the official platform website:
👉 [https://thesoftwaredesignlab.github.io/ANTARES/antares-simulator](https://thesoftwaredesignlab.github.io/ANTARES/antares-simulator)

