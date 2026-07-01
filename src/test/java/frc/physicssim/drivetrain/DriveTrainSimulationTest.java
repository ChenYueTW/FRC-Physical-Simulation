package frc.physicssim.drivetrain;

import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.physicssim.SimConstants.Rebuilt2026;
import frc.physicssim.arena.Arena2026Rebuilt;
import frc.physicssim.gamepieces.rebuilt.RebuiltFuelOnField;
import org.junit.jupiter.api.Test;

class DriveTrainSimulationTest {
    // A free lane along y = 1.5 m that avoids the central HUB.
    private static final double OPEN_LANE_Y = 1.5;

    private static SwerveDriveSimulation robotAt(Arena2026Rebuilt arena, double x, double y) {
        SwerveDriveSimulation robot =
                new SwerveDriveSimulation(new DriveTrainSimulationConfig(), new Pose2d(x, y, Rotation2d.kZero));
        arena.addDriveTrain(robot);
        return robot;
    }

    private static void step(Arena2026Rebuilt arena, int periods) {
        for (int i = 0; i < periods; i++) {
            arena.simulationPeriodic();
        }
    }

    @Test
    void reachesCommandedVelocityOnOpenFloor() {
        Arena2026Rebuilt arena = new Arena2026Rebuilt();
        SwerveDriveSimulation robot = robotAt(arena, 4.0, OPEN_LANE_Y);

        robot.setRobotSpeeds(new ChassisSpeeds(3.0, 0.0, 0.0));
        step(arena, 100); // 2 s

        double vx = robot.getActualFieldSpeeds().vxMetersPerSecond;
        assertTrue(Math.abs(vx - 3.0) < 0.3, "expected ~3 m/s, got " + vx);
        assertTrue(robot.getActualPose().getX() > 5.0, "robot should have driven +X");
    }

    @Test
    void stopsAtPerimeterWallInsteadOfTunneling() {
        Arena2026Rebuilt arena = new Arena2026Rebuilt();
        SwerveDriveSimulation robot = robotAt(arena, 15.0, OPEN_LANE_Y);

        robot.setRobotSpeeds(new ChassisSpeeds(4.0, 0.0, 0.0));
        step(arena, 100); // 2 s

        double x = robot.getActualPose().getX();
        assertTrue(x > 15.1, "robot should have driven toward the wall, x=" + x);
        // Center cannot pass the inner wall face minus half the bumper length.
        assertTrue(x < Rebuilt2026.FIELD_LENGTH_METERS - 0.3, "robot tunneled through the wall, x=" + x);
    }

    @Test
    void collidesWithHubInsteadOfDrivingThrough() {
        Arena2026Rebuilt arena = new Arena2026Rebuilt();
        // Approach the blue HUB head-on from the west.
        SwerveDriveSimulation robot = robotAt(arena, 1.5, Rebuilt2026.BLUE_HUB_CENTER.getY());

        robot.setRobotSpeeds(new ChassisSpeeds(3.5, 0.0, 0.0));
        step(arena, 150); // 3 s

        double x = robot.getActualPose().getX();
        assertTrue(x > 2.0, "robot should have driven toward the HUB, x=" + x);
        // Stopped on the near (west) side of the HUB, not through it.
        assertTrue(x < Rebuilt2026.BLUE_HUB_CENTER.getX() - 0.4, "robot drove through the HUB, x=" + x);
    }

    @Test
    void pushesFuelWhenDrivingIntoIt() {
        Arena2026Rebuilt arena = new Arena2026Rebuilt();
        SwerveDriveSimulation robot = robotAt(arena, 4.0, OPEN_LANE_Y);
        RebuiltFuelOnField fuel = new RebuiltFuelOnField(new Translation2d(5.0, OPEN_LANE_Y));
        arena.addGamePiece(fuel);

        robot.setRobotSpeeds(new ChassisSpeeds(2.0, 0.0, 0.0));
        step(arena, 90); // 1.8 s

        assertTrue(
                fuel.pose2d().getX() > 5.1,
                "fuel should have been pushed +X, x=" + fuel.pose2d().getX());
    }
}
