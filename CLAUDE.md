# CLAUDE.md

Guidance for working in this repository.

## What this is

**FRC-Physical-Simulation** is a Java library that brings a *near-realistic physics environment*
to FRC simulation, designed for teams using **AdvantageKit** logging and **AdvantageScope**
visualization. It is **inspired by maple-sim** but is an independent implementation: the FRC
simulation layer is written from scratch on top of the open-source **dyn4j** 2D rigid-body physics
engine.

The library simulates, for the **2026 REBUILT** season as the reference game:

- Robot ↔ **FUEL** collisions (FUEL = 5.91 in / ~0.15 m foam balls)
- Robot ↔ field-element collisions (perimeter walls, the **HUB**, etc.)
- Driving over the **BUMP** (slowdown + visible pitch/tilt)
- **Intaking** FUEL off the floor
- **Shooting** FUEL (ballistic flight, HUB hit detection, landing)

It is a **library only** — it does not contain a robot project. Robot code consumes it and logs the
exposed poses (`Pose2d` / `Pose3d[]`) to AdvantageScope.

## Design principle: 2D physics + kinematic overlays

dyn4j is a **2D (top-down, X-Y plane)** engine. It owns collisions, friction, and rigid-body
dynamics in the floor plane. Anything **vertical** (projectile arcs, bump tilt/lift) is handled by
our own **kinematic overlay** layer and emitted as `Pose3d` (with rotation) for AdvantageScope.
This is the same trade-off maple-sim makes; it is intentional.

Units everywhere are **SI** (meters, kilograms, seconds, radians) — matches both WPILib and dyn4j.

## Build / test / format

Use the WPILib 2026 JDK (Java 17). On Windows the wrapper picks up `JAVA_HOME`:

```bash
export JAVA_HOME="C:/Users/Public/wpilib/2026/jdk"   # Git Bash
./gradlew build              # compile + test (runs spotlessCheck)
./gradlew test               # unit tests only
./gradlew spotlessApply      # auto-format (Palantir Java Format)
./gradlew publishToMavenLocal # publish for consumption by a robot project (added later)
```

- Build system: **GradleRIO 2026.2.1**, Gradle 8.11 (wrapper), Java 17.
- WPILib artifacts resolve offline from the local maven at `C:/Users/Public/wpilib/2026/maven`;
  dyn4j (`org.dyn4j:dyn4j:5.0.2`) and JUnit come from Maven Central (first build needs network).
- To skip formatting during rapid iteration: `./gradlew build -x spotlessCheck`.

## Architecture (`src/main/java/frc/physicssim/`)

Mirrors maple-sim's proven layering, as our own code. Packages marked *(planned)* are not yet
implemented — update this list as they land.

| Package | Role |
|---|---|
| `SimConstants` | SI constants + 2026 REBUILT geometry/specs (`Rebuilt2026`) |
| `arena/` *(planned)* | `SimulatedArena` (dyn4j `World`, stepping, game-piece registry, pose getters) and `Arena2026Rebuilt` (walls, HUB, BUMP, FUEL layout) |
| `gamepieces/` *(planned)* | `GamePiece`, `GamePieceOnField` (dyn4j body), `GamePieceProjectile` (3D ballistic overlay); `rebuilt/` FUEL types |
| `drivetrain/` *(planned)* | `AbstractDriveTrainSimulation`, `SwerveDriveSimulation` (uses WPILib `DCMotor`/kinematics) |
| `intake/` *(planned)* | `IntakeSimulation` (sensor region; `overTheBumperIntake` / `inTheFrameIntake`) |
| `terrain/` *(planned)* | `TerrainRegion`, `BumpRegion` (kinematic tilt/slowdown overlay) |
| `util/` *(planned)* | `GeometryConvert` (dyn4j ↔ WPILib), math helpers |
| `logging/` *(planned)* | `SimLogger` helper (optional; uses WPILib DataLog/NetworkTables, **not** a hard AdvantageKit dependency) |

`SimulatedComponent` *(planned)*: interface implemented by projectiles, intakes, and terrain
regions; `SimulatedArena` calls `simulationSubTick()` on each per physics sub-tick.

## AdvantageScope integration

The library does **not** depend on AdvantageKit. It exposes poses; the consumer logs them. With
AdvantageKit that is one line:

```java
Logger.recordOutput("FieldSimulation/Fuel", arena.getGamePiecesArrayByType("Fuel")); // Pose3d[]
Logger.recordOutput("FieldSimulation/RobotPose3d", driveSim.getActualPose3d());
```

Without AdvantageKit, `SimLogger` *(planned)* writes the same to a WPILOG / NetworkTables so
AdvantageScope can open the file or connect live. Recommended record-name convention:
`FieldSimulation/<Type>` for game pieces, `FieldSimulation/RobotPose3d` for the chassis,
`FieldSimulation/<Type>Projectiles` for airborne pieces.

## Testing in AdvantageScope

A headless simulation harness *(planned, under test/demo sources — not part of the published jar)*
runs a scripted scenario and either writes a `.wpilog` or serves NetworkTables on `localhost`.
Open `C:/Users/Public/wpilib/2026/advantagescope/"AdvantageScope (WPILib).exe"`, load the file or
connect to `localhost`, choose the REBUILT 3D field, and drag in the `FieldSimulation/*` fields.

## Conventions

- SI units; WPILib coordinate frame (origin at field corner, +X long axis, +Y left, CCW+).
- dyn4j and WPILib both use meters/radians, so conversions are mostly 1:1 (see `GeometryConvert`).
- Format with Palantir Java Format via `spotlessApply` before committing.
- Season-specific code is namespaced (`Rebuilt2026`, `rebuilt/`) so future seasons can be added
  alongside without breaking the evergreen core.
