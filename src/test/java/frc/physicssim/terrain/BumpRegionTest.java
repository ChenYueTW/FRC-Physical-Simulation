package frc.physicssim.terrain;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.physicssim.SimConstants.Rebuilt2026;
import frc.physicssim.arena.Arena2026Rebuilt;
import frc.physicssim.drivetrain.DriveTrainSimulationConfig;
import frc.physicssim.drivetrain.SwerveDriveSimulation;
import frc.physicssim.gamepieces.rebuilt.RebuiltFuelOnField;
import org.junit.jupiter.api.Test;

class BumpRegionTest {
    // A standalone bump for pure geometry tests, independent of the REBUILT field layout: x in
    // [3.0, 4.0], y in [0.0, 5.0], 0.2 m crest.
    private static final double ON_BUMP_Y = 2.5;

    private static BumpRegion bump() {
        return new BumpRegion(3.0, 4.0, 0.0, 5.0, 0.2);
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
        Pose3d pose = bump.elevate(at(3.2)); // first half -> climbing
        assertTrue(pose.getZ() > 0.0, "robot should be raised, z=" + pose.getZ());
        assertTrue(
                pose.getRotation().getY() < 0.0,
                "nose should pitch up, pitch=" + pose.getRotation().getY());
        assertTrue(bump.gradient(new Translation2d(3.2, ON_BUMP_Y)).getX() > 0.0, "uphill gradient");
    }

    @Test
    void noseDownOnTheDescent() {
        BumpRegion bump = bump();
        Pose3d pose = bump.elevate(at(3.8)); // second half -> descending
        assertTrue(
                pose.getRotation().getY() > 0.0,
                "nose should pitch down, pitch=" + pose.getRotation().getY());
        assertTrue(bump.gradient(new Translation2d(3.8, ON_BUMP_Y)).getX() < 0.0, "downhill gradient");
    }

    /** {@link Arena2026Rebuilt} sets its own terrain automatically; addDriveTrain picks it up. */
    @Test
    void robotCrossesRealBumpAndIsElevatedDuringCrossing() {
        Arena2026Rebuilt arena = new Arena2026Rebuilt();
        double laneY = Rebuilt2026.FIELD_CENTER_Y - Rebuilt2026.BUMP_CENTER_Y_OFFSET_METERS;
        SwerveDriveSimulation robot =
                new SwerveDriveSimulation(new DriveTrainSimulationConfig(), new Pose2d(0.3, laneY, Rotation2d.kZero));
        arena.addDriveTrain(robot);

        robot.setRobotSpeeds(new ChassisSpeeds(1.5, 0.0, 0.0));
        double maxZ = 0.0;
        for (int i = 0; i < 200; i++) {
            arena.simulationPeriodic();
            maxZ = Math.max(maxZ, robot.getActualPose3d().getZ());
        }

        assertTrue(maxZ > 0.05, "robot should have been lifted near the crest, maxZ=" + maxZ);
        assertTrue(
                robot.getActualPose().getX() > Rebuilt2026.BUMP_DISTANCE_FROM_WALL_METERS,
                "robot should have crossed the bump");
    }

    /** Both alliance sides get their own pair of BUMPs (left and right of each HUB). */
    @Test
    void bothAlliancesBumpsElevateRobots() {
        Arena2026Rebuilt arena = new Arena2026Rebuilt();
        double blueX = Rebuilt2026.BUMP_DISTANCE_FROM_WALL_METERS;
        double redX = Rebuilt2026.FIELD_LENGTH_METERS - Rebuilt2026.BUMP_DISTANCE_FROM_WALL_METERS;
        double leftY = Rebuilt2026.FIELD_CENTER_Y - Rebuilt2026.BUMP_CENTER_Y_OFFSET_METERS;
        double rightY = Rebuilt2026.FIELD_CENTER_Y + Rebuilt2026.BUMP_CENTER_Y_OFFSET_METERS;

        assertTrue(arena.getTerrain()
                        .elevate(new Pose2d(blueX, leftY, Rotation2d.kZero))
                        .getZ()
                > 0.0);
        assertTrue(arena.getTerrain()
                        .elevate(new Pose2d(blueX, rightY, Rotation2d.kZero))
                        .getZ()
                > 0.0);
        assertTrue(arena.getTerrain()
                        .elevate(new Pose2d(redX, leftY, Rotation2d.kZero))
                        .getZ()
                > 0.0);
        assertTrue(arena.getTerrain()
                        .elevate(new Pose2d(redX, rightY, Rotation2d.kZero))
                        .getZ()
                > 0.0);
    }

    /** FUEL rolling over a BUMP rides up and tilts, same as a drivetrain. */
    @Test
    void fuelIsElevatedCrossingABump() {
        Arena2026Rebuilt arena = new Arena2026Rebuilt();
        double laneY = Rebuilt2026.FIELD_CENTER_Y - Rebuilt2026.BUMP_CENTER_Y_OFFSET_METERS;
        double bumpX = Rebuilt2026.BUMP_DISTANCE_FROM_WALL_METERS;
        RebuiltFuelOnField fuel = new RebuiltFuelOnField(new Translation2d(bumpX, laneY));
        arena.addGamePiece(fuel);

        assertTrue(
                fuel.pose3d().getZ() > Rebuilt2026.FUEL_RADIUS_METERS,
                "fuel should be raised above its flat resting height, z="
                        + fuel.pose3d().getZ());
    }

    /** FUEL placed on a BUMP rolls off it under gravity, same as the drivetrain slope force. */
    @Test
    void fuelRollsOffABumpUnderGravity() {
        Arena2026Rebuilt arena = new Arena2026Rebuilt();
        double laneY = Rebuilt2026.FIELD_CENTER_Y - Rebuilt2026.BUMP_CENTER_Y_OFFSET_METERS;
        double bumpX = Rebuilt2026.BUMP_DISTANCE_FROM_WALL_METERS;
        // Placed on the uphill (near) half of the bump, at rest.
        double startX = bumpX - Rebuilt2026.BUMP_DEPTH_METERS / 4.0;
        RebuiltFuelOnField fuel = new RebuiltFuelOnField(new Translation2d(startX, laneY));
        arena.addGamePiece(fuel);

        for (int i = 0; i < 100; i++) {
            arena.simulationPeriodic();
        }

        assertTrue(
                fuel.pose2d().getX() < startX,
                "fuel should have rolled back downhill, x=" + fuel.pose2d().getX());
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
        robot.setTerrain(steep); // overrides the arena's auto-applied terrain for this test

        robot.setRobotSpeeds(new ChassisSpeeds(0.0, 0.0, 0.0)); // try to hold position
        for (int i = 0; i < 50; i++) {
            arena.simulationPeriodic();
        }

        assertTrue(
                robot.getActualPose().getX() < 5.05,
                "robot should have slid back down, x=" + robot.getActualPose().getX());
    }
}
