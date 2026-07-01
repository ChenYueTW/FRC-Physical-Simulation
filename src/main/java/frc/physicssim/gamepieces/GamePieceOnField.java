package frc.physicssim.gamepieces;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import frc.physicssim.terrain.TerrainProvider;
import frc.physicssim.util.GeometryConvert;
import org.dyn4j.dynamics.Body;
import org.dyn4j.geometry.Convex;
import org.dyn4j.geometry.MassType;

/**
 * A game piece resting on (and sliding/rolling across) the field floor, simulated as a dyn4j rigid
 * body in the top-down X-Y plane. It collides with robots, field walls, and other pieces.
 *
 * <p>The piece's height above the carpet ({@code zHeight}) is constant relative to the local terrain
 * surface — flat carpet by default, but if a {@link TerrainProvider} is set (see {@link
 * #setTerrain}), it rides up and tilts over that terrain (e.g. a BUMP) in the reported {@link
 * #pose3d()}, the same way a drivetrain does. Vertical *flight* is reserved for {@link
 * GamePieceProjectile}.
 */
public class GamePieceOnField extends Body implements GamePiece {
    private final String type;
    private final double zHeight;
    private TerrainProvider terrain = TerrainProvider.FLAT;

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

    /** Sets the terrain this piece rests on (default {@link TerrainProvider#FLAT}). */
    public void setTerrain(TerrainProvider terrain) {
        this.terrain = terrain;
    }

    public TerrainProvider getTerrain() {
        return terrain;
    }

    /** Height of this piece's center above the local terrain surface (its radius, for a ball). */
    public double zHeight() {
        return zHeight;
    }

    /** Floor-plane pose from the underlying dyn4j body transform. */
    public Pose2d pose2d() {
        return GeometryConvert.toWpilibPose(getTransform());
    }

    @Override
    public Pose3d pose3d() {
        Pose3d surface = terrain.elevate(pose2d());
        return new Pose3d(surface.getX(), surface.getY(), surface.getZ() + zHeight, surface.getRotation());
    }
}
