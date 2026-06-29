package frc.physicssim.gamepieces;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.physicssim.arena.Arena2026Rebuilt;
import frc.physicssim.gamepieces.rebuilt.RebuiltFuelOnField;
import frc.physicssim.gamepieces.rebuilt.RebuiltFuelProjectile;
import org.junit.jupiter.api.Test;

class ProjectileTest {
    private static final String FUEL = RebuiltFuelOnField.TYPE;

    private static void step(Arena2026Rebuilt arena, int periods) {
        for (int i = 0; i < periods; i++) {
            arena.simulationPeriodic();
        }
    }

    @Test
    void landsAndBecomesFuelDownrange() {
        Arena2026Rebuilt arena = new Arena2026Rebuilt();
        RebuiltFuelProjectile shot =
                new RebuiltFuelProjectile(arena, new Translation3d(4.0, 1.5, 1.0), new Translation3d(5.0, 0.0, 0.0));
        shot.launch();

        step(arena, 50); // 1 s — well past the ~0.43 s flight time

        assertTrue(shot.isFinished());
        assertEquals(0, arena.getProjectilesArrayByType(FUEL).length, "projectile should have landed");
        assertEquals(1, arena.getGamePiecesArrayByType(FUEL).length, "a FUEL should rest where it landed");
        // Analytic range from h=1.0, resting 0.075, v=5 -> ~2.17 m downrange.
        double landedX = arena.getGamePiecesArrayByType(FUEL)[0].getX();
        assertTrue(landedX > 5.9 && landedX < 6.5, "unexpected landing x=" + landedX);
    }

    @Test
    void isTrackedAsProjectileWhileAirborne() {
        Arena2026Rebuilt arena = new Arena2026Rebuilt();
        RebuiltFuelProjectile shot =
                new RebuiltFuelProjectile(arena, new Translation3d(4.0, 1.5, 1.0), new Translation3d(5.0, 0.0, 0.0));
        shot.launch();

        step(arena, 1); // 20 ms

        assertFalse(shot.isFinished());
        assertEquals(1, arena.getProjectilesArrayByType(FUEL).length);
        assertEquals(0, arena.getGamePiecesArrayByType(FUEL).length);
        assertTrue(shot.position().getZ() < 1.0, "should be descending");
        assertTrue(shot.position().getX() > 4.0, "should have moved downrange");
    }

    @Test
    void hittingTargetConsumesPieceWithoutLanding() {
        Arena2026Rebuilt arena = new Arena2026Rebuilt();
        boolean[] hit = {false};
        RebuiltFuelProjectile shot =
                new RebuiltFuelProjectile(arena, new Translation3d(4.0, 1.5, 1.0), new Translation3d(5.0, 0.0, 0.0));
        // A point on the arc at t~0.3 s: x=5.5, z~0.56.
        shot.withTarget(new Translation3d(5.5, 1.5, 0.559), 0.3, () -> hit[0] = true);
        shot.launch();

        step(arena, 50);

        assertTrue(hit[0], "target hit callback should have fired");
        assertTrue(shot.isFinished());
        assertEquals(0, arena.getProjectilesArrayByType(FUEL).length);
        assertEquals(
                0,
                arena.getGamePiecesArrayByType(FUEL).length,
                "a piece that hits its target is not left on the field");
    }

    @Test
    void fromLaunchArcLandsDownrange() {
        Arena2026Rebuilt arena = new Arena2026Rebuilt();
        RebuiltFuelProjectile shot = RebuiltFuelProjectile.fromLaunch(
                arena,
                new Translation3d(2.0, 1.5, 0.5),
                8.0,
                Rotation2d.kZero,
                Math.toRadians(45),
                new Translation3d());
        shot.launch();

        step(arena, 200); // 4 s

        assertTrue(shot.isFinished());
        assertEquals(1, arena.getGamePiecesArrayByType(FUEL).length);
        assertTrue(arena.getGamePiecesArrayByType(FUEL)[0].getX() > 2.5, "should land downrange of the muzzle");
    }
}
