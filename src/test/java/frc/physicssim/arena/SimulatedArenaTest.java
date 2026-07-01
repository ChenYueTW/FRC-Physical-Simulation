package frc.physicssim.arena;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.physicssim.SimConstants.Rebuilt2026;
import frc.physicssim.gamepieces.rebuilt.RebuiltFuelOnField;
import org.dyn4j.geometry.Vector2;
import org.junit.jupiter.api.Test;

class SimulatedArenaTest {
    private static void step(SimulatedArena arena, int periods) {
        for (int i = 0; i < periods; i++) {
            arena.simulationPeriodic();
        }
    }

    @Test
    void regenerateGamePiecesPlacesNeutralZoneAndBothDepots() {
        Arena2026Rebuilt arena = new Arena2026Rebuilt();
        arena.regenerateFieldGamePieces(20, 6); // neutral + 2 depots
        assertEquals(20 + 6 + 6, arena.getGamePiecesArrayByType(RebuiltFuelOnField.TYPE).length);

        arena.clearGamePieces();
        assertEquals(0, arena.getGamePiecesArrayByType(RebuiltFuelOnField.TYPE).length);
    }

    @Test
    void undisturbedFuelStaysPut() {
        Arena2026Rebuilt arena = new Arena2026Rebuilt();
        Translation2d start = new Translation2d(7.0, 4.0); // clear of the HUBs, BUMPs, TRENCHes, and walls
        RebuiltFuelOnField fuel = new RebuiltFuelOnField(start);
        arena.addGamePiece(fuel);

        step(arena, 50); // 1 second

        // Zero gravity + no applied force -> the piece should not drift.
        assertEquals(start.getX(), fuel.pose2d().getX(), 1e-6);
        assertEquals(start.getY(), fuel.pose2d().getY(), 1e-6);
        // And it rests one radius above the carpet.
        assertEquals(Rebuilt2026.FUEL_RADIUS_METERS, fuel.pose3d().getZ(), 1e-9);
    }

    @Test
    void fuelDoesNotTunnelThroughPerimeterWall() {
        Arena2026Rebuilt arena = new Arena2026Rebuilt();
        // Start near the right wall and drive the ball into it.
        RebuiltFuelOnField fuel =
                new RebuiltFuelOnField(new Pose2d(16.0, 4.0, new edu.wpi.first.math.geometry.Rotation2d()));
        arena.addGamePiece(fuel);
        fuel.setLinearVelocity(new Vector2(3.0, 0.0));

        step(arena, 50); // 1 second

        double x = fuel.pose2d().getX();
        // It moved (physics actually ran)...
        assertTrue(Math.abs(x - 16.0) > 0.05, "fuel should have moved toward the wall, x=" + x);
        // ...but never passed through the wall's inner face at x = FIELD_LENGTH.
        assertTrue(x < Rebuilt2026.FIELD_LENGTH_METERS, "fuel tunneled through the wall, x=" + x);
    }
}
