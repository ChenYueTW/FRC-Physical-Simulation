# FRC-Physical-Simulation

A Java library that brings a **near-realistic physics environment** to FRC simulation, for teams
using **AdvantageKit** + **AdvantageScope**. Inspired by [maple-sim](https://github.com/Shenzhen-Robotics-Alliance/maple-sim),
it is an independent implementation built from scratch on the [dyn4j](https://dyn4j.org/) 2D
rigid-body physics engine, with the 2026 **REBUILT** game as its reference season.

It simulates the robot actually interacting with its environment:

- 🟡 Robot ↔ **FUEL** collisions (the 5.91 in foam balls)
- 🧱 Robot ↔ field collisions — perimeter walls and the **HUB**
- ⛰️ Driving over the **BUMP** — the robot slows, tips up, and crests it
- 🧲 **Intaking** FUEL off the floor
- 🚀 **Shooting** FUEL on a ballistic arc, with HUB hit detection

## How it works

dyn4j owns the 2D (top-down) world: collisions, friction, momentum. Vertical motion — projectile
arcs and bump tilt — is a kinematic overlay emitted as `Pose3d` for AdvantageScope. Everything is
reported as `Pose2d`/`Pose3d` so you log it however you like (AdvantageKit, NetworkTables, or the
built-in `SimLogger`).

## Quick start

Publish the library to your local Maven, then depend on it:

```bash
./gradlew publishToMavenLocal
```

Wire it into your robot's simulation:

```java
SimulatedArena arena = SimulatedArena.getInstance();        // Arena2026Rebuilt
arena.resetFieldForAuto();                                  // place starting FUEL

var robot = new SwerveDriveSimulation(
        new DriveTrainSimulationConfig().withMass(45).withBumperSize(0.8, 0.8),
        new Pose2d(2, 2, Rotation2d.kZero));
arena.addDriveTrain(robot);
robot.setTerrain(Arena2026Rebuilt.rebuiltBump());           // tilt + slow over the bump

var intake = IntakeSimulation.overTheBumperIntake(
        arena, robot, RebuiltFuelOnField.TYPE, Side.FRONT, 0.7, 0.4, 5);

// each loop, in simulationPeriodic():
robot.setRobotSpeeds(commandedChassisSpeeds);
intake.setRunning(intakeButtonHeld);
arena.simulationPeriodic();

// log for AdvantageScope (AdvantageKit shown; or use SimLogger for plain NetworkTables)
Logger.recordOutput("FieldSimulation/RobotPose3d", robot.getActualPose3d());
Logger.recordOutput("FieldSimulation/Fuel", arena.getGamePiecesArrayByType("Fuel"));

// shoot a stored FUEL at the HUB
if (shootButtonPressed && intake.obtainGamePieceFromStorage()) {
    RebuiltFuelProjectile.fromLaunch(arena, muzzlePose, 5.5, yawToHub, Math.toRadians(45),
            new Translation3d()).launch();
}
```

## See it in AdvantageScope

No robot project needed — a built-in demo starts a NetworkTables server and runs all three
scenarios:

```bash
SIM_DEMO_DURATION=0 ./gradlew runSimDemo
```

Then open AdvantageScope, connect to `localhost`, pick the REBUILT **3D Field**, and add the
`FieldSimulation/*` fields. See [CLAUDE.md](CLAUDE.md) for full build/test/architecture details.

## Requirements

WPILib 2026 (Java 17 + GradleRIO), built with the Gradle wrapper. dyn4j and JUnit are fetched from
Maven Central on the first build.
