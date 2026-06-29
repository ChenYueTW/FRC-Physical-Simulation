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
./gradlew runSimDemo         # run the AdvantageScope demo (NT4 server on localhost)
./gradlew publishToMavenLocal # publish for consumption by a robot project
```

- Build system: **GradleRIO 2026.2.1**, Gradle 8.11 (wrapper), Java 17.
- WPILib artifacts resolve offline from the local maven at `C:/Users/Public/wpilib/2026/maven`;
  dyn4j (`org.dyn4j:dyn4j:5.0.2`) and JUnit come from Maven Central (first build needs network).
- To skip formatting during rapid iteration: `./gradlew build -x spotlessCheck`.
- **Native libraries**: the unit tests stay pure-Java (geometry + dyn4j), so `test` needs no JNI.
  Anything touching NetworkTables/DataLog needs the WPILib desktop natives, which GradleRIO's
  `extractReleaseNative` unpacks to `build/jni/release`; the `runSimDemo` task puts that on the
  library path. (A plain `test` task crashes if it loads NetworkTables — keep NT/DataLog out of unit
  tests.)

## Architecture (`src/main/java/frc/physicssim/`)

Mirrors maple-sim's proven layering, as our own code.

| Package | Role |
|---|---|
| `SimConstants` | SI constants + 2026 REBUILT geometry/specs (`Rebuilt2026`) |
| `SimulatedComponent` | interface for high-frequency logic; `simulationSubTick(i, dt)` runs before each world step |
| `arena/` | `SimulatedArena` (dyn4j `World`, sub-tick stepping, game-piece/projectile/drivetrain registries, `Pose3d[]` getters), `FieldMap` (walls/obstacles), `Arena2026Rebuilt` (walls + HUB, `rebuiltBump()`, `resetFieldForAuto()`) |
| `gamepieces/` | `GamePiece`, `GamePieceOnField` (dyn4j body), `GamePieceProjectile` (3D ballistic overlay with target/landing callbacks); `rebuilt/` → `RebuiltFuelOnField`, `RebuiltFuelProjectile` |
| `drivetrain/` | `DriveTrainSimulationConfig`, `AbstractDriveTrainSimulation` (bumper body + traction-limited velocity controller + terrain slope force), `SwerveDriveSimulation` (holonomic setpoints) |
| `intake/` | `IntakeSimulation` (robot-frame pickup region; `overTheBumperIntake` / `inTheFrameIntake`, capacity, storage) |
| `terrain/` | `TerrainProvider` (elevate + gradient; `FLAT`), `BumpRegion` (sinusoidal crest → tilt/lift + slope gravity) |
| `util/` | `GeometryConvert` (dyn4j ↔ WPILib) |
| `logging/` | `SimLogger` — publishes `FieldSimulation/*` struct topics to NetworkTables (WPILib only, **no** AdvantageKit dependency) |
| `demo/` | `SimDemo` — runnable AdvantageScope harness (`runSimDemo`), **not** part of the API |

Key flow: build an `Arena2026Rebuilt`, add a `SwerveDriveSimulation` (`addDriveTrain`), attach an
`IntakeSimulation` and `robot.setTerrain(rebuiltBump())`, then call `arena.simulationPeriodic()`
once per robot loop and publish poses. Launch FUEL with `RebuiltFuelProjectile.fromLaunch(...)`.

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

`frc.physicssim.demo.SimDemo` (run via `./gradlew runSimDemo`) is a headless harness — no robot/HAL.
It starts a NetworkTables server on `localhost` and steps a scripted scenario (intake the FUEL
cluster → cross the BUMP → shoot at the HUB), publishing `FieldSimulation/*` struct topics. It runs
`SIM_DEMO_DURATION` seconds (env var, default 15; set `0` to run indefinitely for live viewing) and
prints a per-second status plus a peak bump lift/tilt summary.

To view it:

1. `SIM_DEMO_DURATION=0 ./gradlew runSimDemo` (keep it running).
2. Open `C:/Users/Public/wpilib/2026/advantagescope/"AdvantageScope (WPILib).exe"`.
3. Connect to `localhost` (NT4), choose the **3D Field** tab and the REBUILT field.
4. Drag in `FieldSimulation/RobotPose3d` (robot), `FieldSimulation/Fuel` (on-field balls),
   `FieldSimulation/FuelProjectiles` (shots in flight). Game pieces render as 3D objects; the robot
   tilts/rises crossing the bump.

The physics itself is covered headlessly by the JUnit suite (`./gradlew test`): collisions, intake,
projectiles, and bump traversal all have assertions.

## Conventions

- SI units; WPILib coordinate frame (origin at field corner, +X long axis, +Y left, CCW+).
- dyn4j and WPILib both use meters/radians, so conversions are mostly 1:1 (see `GeometryConvert`).
- Format with Palantir Java Format via `spotlessApply` before committing.
- Season-specific code is namespaced (`Rebuilt2026`, `rebuilt/`) so future seasons can be added
  alongside without breaking the evergreen core.
