package frc.physicssim.terrain;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.physicssim.arena.Arena2026Rebuilt;
import frc.physicssim.drivetrain.DriveTrainSimulationConfig;
import frc.physicssim.drivetrain.SwerveDriveSimulation;
import org.junit.jupiter.api.Test;

class BumpRegionTest {
    // The REBUILT bump spans x in [~3.46, ~4.59], y in [~1.57, ~3.43] (blue -Y side of the HUB).
    private static final double ON_BUMP_Y = 2.5;

    private static BumpRegion bump() {
        return Arena2026Rebuilt.rebuiltBump();
    }

    private static Pose2d at(double x) {
        return new Pose2d(x, ON_BUMP_Y, Rotation2d.kZero);
    }

    @Test
    void flatOffTheBump() {
        BumpRegion bump = bump();
        Pose3d pose = bump.elevate(at(1.0));
        assertEquals(0.0, pose.getZ(), 1e-9);
        assertEquals(0.0, pose.getRotation().getY(), 1e-9); // pitch
        assertEquals(0.0, bump.gradient(new Translation2d(1.0, ON_BUMP_Y)).getNorm(), 1e-9);
    }

    @Test
    void noseUpOnTheClimbAndRaised() {
        BumpRegion bump = bump();
        Pose3d pose = bump.elevate(at(3.7)); // first half -> climbing
        assertTrue(pose.getZ() > 0.0, "robot should be raised, z=" + pose.getZ());
        assertTrue(
                pose.getRotation().getY() < 0.0,
                "nose should pitch up, pitch=" + pose.getRotation().getY());
        assertTrue(bump.gradient(new Translation2d(3.7, ON_BUMP_Y)).getX() > 0.0, "uphill gradient");
    }

    @Test
    void noseDownOnTheDescent() {
        BumpRegion bump = bump();
        Pose3d pose = bump.elevate(at(4.4)); // second half -> descending
        assertTrue(
                pose.getRotation().getY() > 0.0,
                "nose should pitch down, pitch=" + pose.getRotation().getY());
        assertTrue(bump.gradient(new Translation2d(4.4, ON_BUMP_Y)).getX() < 0.0, "downhill gradient");
    }

    @Test
    void robotCrossesBumpAndIsElevatedDuringCrossing() {
        Arena2026Rebuilt arena = new Arena2026Rebuilt();
        SwerveDriveSimulation robot = new SwerveDriveSimulation(
                new DriveTrainSimulationConfig(), new Pose2d(2.5, ON_BUMP_Y, Rotation2d.kZero));
        arena.addDriveTrain(robot);
        robot.setTerrain(Arena2026Rebuilt.rebuiltBump());

        robot.setRobotSpeeds(new ChassisSpeeds(2.0, 0.0, 0.0));
        double maxZ = 0.0;
        for (int i = 0; i < 200; i++) {
            arena.simulationPeriodic();
            maxZ = Math.max(maxZ, robot.getActualPose3d().getZ());
        }

        assertTrue(maxZ > 0.05, "robot should have been lifted near the crest, maxZ=" + maxZ);
        assertTrue(robot.getActualPose().getX() > 5.5, "robot should have crossed the bump");
    }

    @Test
    void slidesBackWhenTractionCannotHoldTheSlope() {
        Arena2026Rebuilt arena = new Arena2026Rebuilt();
        // Weak, low-grip robot on a steep bump it cannot hold.
        DriveTrainSimulationConfig weak = new DriveTrainSimulationConfig()
                .withWheelCoefficientOfFriction(0.2)
                .withMaxLinearAcceleration(2.0);
        SwerveDriveSimulation robot = new SwerveDriveSimulation(weak, new Pose2d(5.1, 1.5, Rotation2d.kZero));
        arena.addDriveTrain(robot);
        BumpRegion steep = new BumpRegion(5.0, 5.5, 0.0, 8.0, 0.2);
        robot.setTerrain(steep);

        robot.setRobotSpeeds(new ChassisSpeeds(0.0, 0.0, 0.0)); // try to hold position
        for (int i = 0; i < 50; i++) {
            arena.simulationPeriodic();
        }

        assertTrue(
                robot.getActualPose().getX() < 5.05,
                "robot should have slid back down, x=" + robot.getActualPose().getX());
    }
}
