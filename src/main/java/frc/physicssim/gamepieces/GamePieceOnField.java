package frc.physicssim.gamepieces;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.physicssim.util.GeometryConvert;
import org.dyn4j.dynamics.Body;
import org.dyn4j.geometry.Convex;
import org.dyn4j.geometry.MassType;

/**
 * A game piece resting on (and sliding/rolling across) the field floor, simulated as a dyn4j rigid
 * body in the top-down X-Y plane. It collides with robots, field walls, and other pieces.
 *
 * <p>The piece's height above the carpet ({@code zHeight}) is constant in this 2D model and is only
 * used to build the reported {@link #pose3d()} — vertical motion is reserved for {@link
 * GamePieceProjectile}.
 */
public class GamePieceOnField extends Body implements GamePiece {
    private final String type;
    private final double zHeight;

    /**
     * @param type type tag used for logging/grouping
     * @param shape dyn4j collision shape in the floor plane (e.g. a circle for a ball)
     * @param density material density (kg/m^2) — chosen so {@code density * area} equals the desired
     *     mass
     * @param friction coefficient of friction with contacting bodies
     * @param restitution coefficient of restitution (bounciness), 0..1
     * @param linearDamping velocity damping that models drag against the carpet
     * @param zHeight height of the piece's center above the carpet, for {@link #pose3d()}
     * @param initialPose initial floor-plane pose
     */
    public GamePieceOnField(
            String type,
            Convex shape,
            double density,
            double friction,
            double restitution,
            double linearDamping,
            double zHeight,
            Pose2d initialPose) {
        this.type = type;
        this.zHeight = zHeight;

        addFixture(shape, density, friction, restitution);
        setMass(MassType.NORMAL);
        setLinearDamping(linearDamping);
        setAngularDamping(linearDamping);
        getTransform().setTranslation(initialPose.getX(), initialPose.getY());
        getTransform().setRotation(initialPose.getRotation().getRadians());
    }

    @Override
    public String type() {
        return type;
    }

    /** Floor-plane pose from the underlying dyn4j body transform. */
    public Pose2d pose2d() {
        return GeometryConvert.toWpilibPose(getTransform());
    }

    @Override
    public Pose3d pose3d() {
        Pose2d pose2d = pose2d();
        return new Pose3d(
                pose2d.getX(),
                pose2d.getY(),
                zHeight,
                new Rotation3d(0.0, 0.0, pose2d.getRotation().getRadians()));
    }
}
