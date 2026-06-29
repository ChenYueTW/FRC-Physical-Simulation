package frc.physicssim.arena;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.physicssim.SimConstants;
import frc.physicssim.SimConstants.Rebuilt2026;
import frc.physicssim.gamepieces.rebuilt.RebuiltFuelOnField;
import org.dyn4j.geometry.Geometry;

/**
 * The 2026 <b>REBUILT</b> arena: a {@value Rebuilt2026#FIELD_LENGTH_METERS}-by-{@value
 * Rebuilt2026#FIELD_WIDTH_METERS} m field bounded by perimeter walls, with the central HUB as a
 * collision obstacle.
 *
 * <p>Several dimensions and positions here (HUB size/location, FUEL starting layout, and — added
 * later — the BUMP and TRENCH) are reasonable approximations marked {@code TODO}, pending
 * confirmation against the official 2026 Game Manual field drawings.
 */
public class Arena2026Rebuilt extends SimulatedArena {
    // TODO: confirm HUB geometry/placement against the official field drawings.
    private static final Translation2d HUB_CENTER = Rebuilt2026.FIELD_CENTER;
    private static final double HUB_RADIUS_METERS = 0.6;
    private static final double WALL_THICKNESS_METERS = 0.1;

    public Arena2026Rebuilt() {
        super(buildFieldMap());
    }

    private static FieldMap buildFieldMap() {
        FieldMap map = new FieldMap();
        map.addPerimeterWalls(Rebuilt2026.FIELD_LENGTH_METERS, Rebuilt2026.FIELD_WIDTH_METERS, WALL_THICKNESS_METERS);
        // Central HUB: robots and FUEL collide with it.
        map.addObstacle(
                Geometry.createCircle(HUB_RADIUS_METERS),
                new Pose2d(HUB_CENTER, new edu.wpi.first.math.geometry.Rotation2d()),
                SimConstants.CARPET_FRICTION_COEFFICIENT);
        return map;
    }

    /**
     * Clears the field and lays out the starting FUEL for autonomous.
     *
     * <p>TODO: replace this placeholder cluster with the official REBUILT starting configuration.
     */
    public void resetFieldForAuto() {
        clearGamePieces();
        // A small staging cluster near the blue-side Depot, away from the HUB.
        double x = 2.5;
        for (int row = 0; row < 3; row++) {
            for (int col = 0; col < 3; col++) {
                double y = 2.0 + row * 0.4;
                addGamePiece(new RebuiltFuelOnField(new Translation2d(x + col * 0.4, y)));
            }
        }
    }
}
