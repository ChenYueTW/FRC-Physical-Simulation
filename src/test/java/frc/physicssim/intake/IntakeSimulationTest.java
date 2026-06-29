package frc.physicssim.intake;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.physicssim.arena.Arena2026Rebuilt;
import frc.physicssim.drivetrain.DriveTrainSimulationConfig;
import frc.physicssim.drivetrain.SwerveDriveSimulation;
import frc.physicssim.gamepieces.rebuilt.RebuiltFuelOnField;
import org.junit.jupiter.api.Test;

class IntakeSimulationTest {
    private static final double LANE_Y = 1.5;
    private static final String FUEL = RebuiltFuelOnField.TYPE;

    private static SwerveDriveSimulation robotAt(Arena2026Rebuilt arena, double x, double y) {
        SwerveDriveSimulation robot =
                new SwerveDriveSimulation(new DriveTrainSimulationConfig(), new Pose2d(x, y, Rotation2d.kZero));
        arena.addDriveTrain(robot);
        return robot;
    }

    private static IntakeSimulation frontIntake(Arena2026Rebuilt arena, SwerveDriveSimulation robot, int capacity) {
        return IntakeSimulation.overTheBumperIntake(
                arena, robot, FUEL, IntakeSimulation.Side.FRONT, 0.7, 0.4, capacity);
    }

    private static void step(Arena2026Rebuilt arena, int periods) {
        for (int i = 0; i < periods; i++) {
            arena.simulationPeriodic();
        }
    }

    @Test
    void capturesFuelInFrontRegionWhenRunning() {
        Arena2026Rebuilt arena = new Arena2026Rebuilt();
        SwerveDriveSimulation robot = robotAt(arena, 4.0, LANE_Y);
        IntakeSimulation intake = frontIntake(arena, robot, 5);
        arena.addGamePiece(new RebuiltFuelOnField(new Translation2d(4.6, LANE_Y))); // in the front region

        intake.setRunning(true);
        step(arena, 5);

        assertEquals(1, intake.getStoredCount());
        assertEquals(0, arena.getGamePiecesArrayByType(FUEL).length);
    }

    @Test
    void doesNotCaptureWhenIdle() {
        Arena2026Rebuilt arena = new Arena2026Rebuilt();
        SwerveDriveSimulation robot = robotAt(arena, 4.0, LANE_Y);
        IntakeSimulation intake = frontIntake(arena, robot, 5);
        arena.addGamePiece(new RebuiltFuelOnField(new Translation2d(4.6, LANE_Y)));

        intake.setRunning(false);
        step(arena, 10);

        assertEquals(0, intake.getStoredCount());
        assertEquals(1, arena.getGamePiecesArrayByType(FUEL).length);
    }

    @Test
    void stopsCapturingAtCapacity() {
        Arena2026Rebuilt arena = new Arena2026Rebuilt();
        SwerveDriveSimulation robot = robotAt(arena, 4.0, LANE_Y);
        IntakeSimulation intake = frontIntake(arena, robot, 2);
        arena.addGamePiece(new RebuiltFuelOnField(new Translation2d(5.0, LANE_Y)));
        arena.addGamePiece(new RebuiltFuelOnField(new Translation2d(6.0, LANE_Y)));
        arena.addGamePiece(new RebuiltFuelOnField(new Translation2d(7.0, LANE_Y)));

        intake.setRunning(true);
        robot.setRobotSpeeds(new ChassisSpeeds(2.0, 0.0, 0.0));
        step(arena, 150); // 3 s — drive through all three

        assertEquals(2, intake.getStoredCount(), "should cap at capacity");
        assertTrue(
                arena.getGamePiecesArrayByType(FUEL).length >= 1,
                "the third FUEL should remain on the field once the intake is full");
    }
}
