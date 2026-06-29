package frc.physicssim.arena;

import edu.wpi.first.math.geometry.Pose2d;
import frc.physicssim.SimConstants;
import java.util.ArrayList;
import java.util.List;
import org.dyn4j.dynamics.Body;
import org.dyn4j.geometry.Convex;
import org.dyn4j.geometry.Geometry;
import org.dyn4j.geometry.MassType;
import org.dyn4j.geometry.Rectangle;

/**
 * The static collision geometry of a field: perimeter walls and fixed obstacles (e.g. the HUB).
 *
 * <p>Every obstacle is an {@link MassType#INFINITE infinite-mass} dyn4j body, so robots and game
 * pieces bounce off it but it never moves. A season arena builds its {@code FieldMap} and hands it
 * to {@link SimulatedArena}.
 */
public class FieldMap {
    private final List<Body> obstacles = new ArrayList<>();

    /** All obstacle bodies, to be added to the physics world. */
    public List<Body> obstacles() {
        return obstacles;
    }

    /**
     * Adds a static obstacle made of one convex shape at the given floor-plane pose.
     *
     * @param friction coefficient of friction of the obstacle surface
     * @return this, for chaining
     */
    public FieldMap addObstacle(Convex shape, Pose2d pose, double friction) {
        Body body = new Body();
        body.addFixture(shape, 1.0, friction, 0.4);
        body.setMass(MassType.INFINITE);
        body.getTransform().setTranslation(pose.getX(), pose.getY());
        body.getTransform().setRotation(pose.getRotation().getRadians());
        obstacles.add(body);
        return this;
    }

    /**
     * Adds the four perimeter walls of a rectangular field whose lower-left corner is the field-frame
     * origin. Walls are thin rectangles placed just outside the playable area so their inner faces
     * sit on the field boundary.
     *
     * @param fieldLength field size along +X (m)
     * @param fieldWidth field size along +Y (m)
     * @param wallThickness wall thickness (m)
     * @return this, for chaining
     */
    public FieldMap addPerimeterWalls(double fieldLength, double fieldWidth, double wallThickness) {
        Body walls = new Body();
        double t = wallThickness;
        double halfT = t / 2.0;

        // Bottom (y = 0) and top (y = fieldWidth) walls run the full length.
        Rectangle bottom = Geometry.createRectangle(fieldLength + 2 * t, t);
        bottom.translate(fieldLength / 2.0, -halfT);
        Rectangle top = Geometry.createRectangle(fieldLength + 2 * t, t);
        top.translate(fieldLength / 2.0, fieldWidth + halfT);

        // Left (x = 0) and right (x = fieldLength) walls run the full width.
        Rectangle left = Geometry.createRectangle(t, fieldWidth);
        left.translate(-halfT, fieldWidth / 2.0);
        Rectangle right = Geometry.createRectangle(t, fieldWidth);
        right.translate(fieldLength + halfT, fieldWidth / 2.0);

        for (Rectangle r : new Rectangle[] {bottom, top, left, right}) {
            walls.addFixture(r, 1.0, SimConstants.CARPET_FRICTION_COEFFICIENT, 0.3);
        }
        walls.setMass(MassType.INFINITE);
        obstacles.add(walls);
        return this;
    }
}
