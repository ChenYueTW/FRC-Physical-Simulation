package frc.physicssim.demo;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.physicssim.SimConstants.Rebuilt2026;
import frc.physicssim.arena.Arena2026Rebuilt;
import frc.physicssim.drivetrain.DriveTrainSimulationConfig;
import frc.physicssim.drivetrain.SwerveDriveSimulation;
import frc.physicssim.gamepieces.rebuilt.RebuiltFuelOnField;
import frc.physicssim.gamepieces.rebuilt.RebuiltFuelProjectile;
import frc.physicssim.intake.IntakeSimulation;
import frc.physicssim.logging.SimLogger;

/**
 * A standalone, scripted demonstration of the library, run with {@code ./gradlew runSimDemo}. It
 * needs <b>no robot/HAL</b> — it starts a NetworkTables server on {@code localhost} and steps the
 * simulation in a real-time loop, so AdvantageScope can connect live and render the {@code
 * FieldSimulation/*} struct topics.
 *
 * <p>The robot intakes the FUEL cluster, drives over the BUMP, then shoots FUEL at the HUB. Runs for
 * {@code SIM_DEMO_DURATION} seconds (env var, default 15); set it to {@code 0} to run forever.
 *
 * <p>This is a test/visualization harness, not part of the library API.
 */
public final class SimDemo {
    private static final Translation2d HUB = Rebuilt2026.FIELD_CENTER;
    private static final String FUEL = RebuiltFuelOnField.TYPE;
    private static final double DT = 0.02;

    private final Arena2026Rebuilt arena = new Arena2026Rebuilt();
    private final SwerveDriveSimulation robot =
            new SwerveDriveSimulation(new DriveTrainSimulationConfig(), new Pose2d(2.0, 2.0, Rotation2d.kZero));
    private IntakeSimulation intake;

    private double elapsed = 0.0;
    private double lastShot = 0.0;
    private int shotsFired = 0;
    private int scored = 0;
    private double maxBumpZ = 0.0;
    private double maxBumpPitchDeg = 0.0;

    public static void main(String[] args) throws InterruptedException {
        new SimDemo().run();
    }

    private void run() throws InterruptedException {
        double duration = readDuration();

        NetworkTableInstance nt = NetworkTableInstance.getDefault();
        nt.startServer();
        SimLogger logger = new SimLogger(nt, "FieldSimulation");

        arena.resetFieldForAuto();
        arena.addDriveTrain(robot);
        robot.setTerrain(Arena2026Rebuilt.rebuiltBump());
        intake = IntakeSimulation.overTheBumperIntake(arena, robot, FUEL, IntakeSimulation.Side.FRONT, 0.7, 0.4, 5);

        System.out.println("FRC-Physical-Simulation demo: NetworkTables server up on localhost.");
        System.out.println("Connect AdvantageScope to 'localhost' and add the FieldSimulation/* fields. duration="
                + duration + "s");

        double nextStatus = 0.0;
        while (duration <= 0 || elapsed < duration) {
            runScenario();
            arena.simulationPeriodic();
            logger.update(arena, robot, FUEL);
            nt.flush();

            maxBumpZ = Math.max(maxBumpZ, robot.getActualPose3d().getZ());
            maxBumpPitchDeg = Math.max(
                    maxBumpPitchDeg,
                    Math.abs(
                            Math.toDegrees(robot.getActualPose3d().getRotation().getY())));

            if (elapsed >= nextStatus) {
                nextStatus += 1.0;
                printStatus();
            }
            Thread.sleep((long) (DT * 1000));
            elapsed += DT;
        }

        System.out.printf(
                "DEMO COMPLETE: shots=%d scored=%d  peak bump lift=%.3f m, peak tilt=%.1f deg%n",
                shotsFired, scored, maxBumpZ, maxBumpPitchDeg);
        nt.close();
    }

    /** Phase 1: intake the FUEL cluster. Phase 2: cross the BUMP. Phase 3: shoot at the HUB. */
    private void runScenario() {
        if (elapsed < 3.0) {
            intake.setRunning(true);
            robot.setRobotSpeeds(new ChassisSpeeds(1.0, 0.4, 0.0)); // drive into the FUEL cluster
        } else if (elapsed < 4.3) {
            intake.setRunning(false);
            robot.setRobotSpeeds(new ChassisSpeeds(1.6, 0.0, 0.0)); // cross the bump, stop short of the HUB
        } else if (elapsed < 5.2) {
            aimAtHub(); // settle and turn toward the HUB
        } else {
            aimAtHub();
            if (elapsed - lastShot > 0.6) {
                lastShot = elapsed;
                shoot();
            }
        }
    }

    private void aimAtHub() {
        Pose2d pose = robot.getActualPose();
        Rotation2d toHub = new Rotation2d(HUB.getX() - pose.getX(), HUB.getY() - pose.getY());
        double headingError = toHub.minus(pose.getRotation()).getRadians();
        robot.setRobotSpeeds(new ChassisSpeeds(0.0, 0.0, 4.0 * headingError));
    }

    private void shoot() {
        if (!intake.obtainGamePieceFromStorage()) {
            return;
        }
        shotsFired++;
        Pose2d pose = robot.getActualPose();
        Rotation2d yaw = new Rotation2d(HUB.getX() - pose.getX(), HUB.getY() - pose.getY());
        RebuiltFuelProjectile shot = RebuiltFuelProjectile.fromLaunch(
                arena,
                new Translation3d(pose.getX(), pose.getY(), 0.6),
                5.5,
                yaw,
                Math.toRadians(45),
                new Translation3d());
        shot.withTarget(new Translation3d(HUB.getX(), HUB.getY(), 1.2), 0.7, () -> scored++);
        shot.launch();
    }

    private void printStatus() {
        System.out.printf(
                "t=%4.1fs  robot=(%.2f, %.2f) z=%.3f pitch=%5.1f deg  stored=%d  fuelOnField=%2d  airborne=%d  shots=%d scored=%d%n",
                elapsed,
                robot.getActualPose().getX(),
                robot.getActualPose().getY(),
                robot.getActualPose3d().getZ(),
                Math.toDegrees(robot.getActualPose3d().getRotation().getY()),
                intake.getStoredCount(),
                arena.getGamePiecesArrayByType(FUEL).length,
                arena.getProjectilesArrayByType(FUEL).length,
                shotsFired,
                scored);
    }

    private static double readDuration() {
        String env = System.getenv("SIM_DEMO_DURATION");
        if (env != null) {
            try {
                return Double.parseDouble(env);
            } catch (NumberFormatException ignored) {
                // fall through
            }
        }
        return 15.0;
    }

    private SimDemo() {}
}
