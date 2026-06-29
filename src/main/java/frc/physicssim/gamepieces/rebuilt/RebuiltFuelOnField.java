package frc.physicssim.gamepieces.rebuilt;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.physicssim.SimConstants.Rebuilt2026;
import frc.physicssim.gamepieces.GamePieceOnField;
import org.dyn4j.geometry.Geometry;

/**
 * A 2026 REBUILT <b>FUEL</b> ball resting on the field: a {@value #TYPE}-typed circular body sized
 * and weighted to the official 5.91 in foam ball.
 */
public class RebuiltFuelOnField extends GamePieceOnField {
    /** Logging/grouping type tag for FUEL. */
    public static final String TYPE = "Fuel";

    /** Friction of foam against carpet / robot surfaces. */
    private static final double FRICTION = 0.6;

    /** Velocity damping that models the ball slowing on carpet. */
    private static final double LINEAR_DAMPING = 0.8;

    public RebuiltFuelOnField(Translation2d position) {
        this(new Pose2d(position, new edu.wpi.first.math.geometry.Rotation2d()));
    }

    public RebuiltFuelOnField(Pose2d initialPose) {
        super(
                TYPE,
                Geometry.createCircle(Rebuilt2026.FUEL_RADIUS_METERS),
                density(),
                FRICTION,
                Rebuilt2026.FUEL_RESTITUTION,
                LINEAR_DAMPING,
                Rebuilt2026.FUEL_RADIUS_METERS,
                initialPose);
    }

    /** Density (kg/m^2) chosen so {@code density * circleArea == FUEL_MASS}. */
    private static double density() {
        double area = Math.PI * Rebuilt2026.FUEL_RADIUS_METERS * Rebuilt2026.FUEL_RADIUS_METERS;
        return Rebuilt2026.FUEL_MASS_KG / area;
    }
}
