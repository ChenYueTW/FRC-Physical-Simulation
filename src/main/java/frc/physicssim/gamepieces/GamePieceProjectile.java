package frc.physicssim.gamepieces;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.physicssim.SimConstants;
import frc.physicssim.SimulatedComponent;
import frc.physicssim.arena.SimulatedArena;
import java.util.function.Function;

/**
 * A game piece flying through the air, simulated as a 3D ballistic point (constant horizontal
 * velocity, gravity in Z). It lives <b>outside</b> the dyn4j world — the 2D engine has no vertical
 * axis — and is advanced by its own kinematics each sub-tick.
 *
 * <p>A projectile ends its flight in one of two ways:
 *
 * <ul>
 *   <li><b>Hits a target</b> (e.g. the HUB): if a target is set and the projectile passes within
 *       tolerance, the hit callback fires and the piece is consumed.
 *   <li><b>Lands</b>: when it drops to its resting height, the miss callback fires (if a target was
 *       set) and — if configured — it becomes a {@link GamePieceOnField} where it touched down.
 * </ul>
 *
 * <p>Build one, configure it with the fluent {@code with*}/{@code becomes*} methods, then {@link
 * #launch()}.
 */
public abstract class GamePieceProjectile implements SimulatedComponent, GamePiece {
    private final SimulatedArena arena;
    private final String type;

    private Translation3d position;
    private Translation3d velocity;
    private double timeAirborneSeconds = 0.0;
    private boolean launched = false;
    private boolean finished = false;

    // Landing behavior.
    private double restingHeightMeters = 0.0;
    private Function<Translation3d, GamePieceOnField> onLandingFactory = null;

    // Optional target.
    private Translation3d targetPosition = null;
    private double targetToleranceMeters = 0.0;
    private Runnable onHitTarget = null;
    private Runnable onMissTarget = null;

    protected GamePieceProjectile(
            SimulatedArena arena, String type, Translation3d initialPosition, Translation3d initialVelocity) {
        this.arena = arena;
        this.type = type;
        this.position = initialPosition;
        this.velocity = initialVelocity;
    }

    /** Sets a target; if the projectile passes within {@code tolerance} of it, {@code onHit} fires. */
    public GamePieceProjectile withTarget(Translation3d target, double toleranceMeters, Runnable onHit) {
        this.targetPosition = target;
        this.targetToleranceMeters = toleranceMeters;
        this.onHitTarget = onHit;
        return this;
    }

    /** Sets a callback for when a targeted projectile lands without hitting its target. */
    public GamePieceProjectile withMissCallback(Runnable onMiss) {
        this.onMissTarget = onMiss;
        return this;
    }

    /** Sets the center height at which the piece is considered to have landed (default 0). */
    public GamePieceProjectile withRestingHeight(double restingHeightMeters) {
        this.restingHeightMeters = restingHeightMeters;
        return this;
    }

    /** Configures the projectile to spawn an on-field piece (built by {@code factory}) on landing. */
    public GamePieceProjectile becomesOnFieldOnLanding(Function<Translation3d, GamePieceOnField> factory) {
        this.onLandingFactory = factory;
        return this;
    }

    /** Registers the projectile with the arena so it begins flying. */
    public void launch() {
        if (!launched) {
            launched = true;
            arena.addProjectile(this);
        }
    }

    public boolean isFinished() {
        return finished;
    }

    public double timeAirborneSeconds() {
        return timeAirborneSeconds;
    }

    public Translation3d position() {
        return position;
    }

    public Translation3d velocity() {
        return velocity;
    }

    @Override
    public String type() {
        return type;
    }

    @Override
    public Pose3d pose3d() {
        double horizontalSpeed = Math.hypot(velocity.getX(), velocity.getY());
        double yaw = Math.atan2(velocity.getY(), velocity.getX());
        double pitch = -Math.atan2(velocity.getZ(), horizontalSpeed);
        return new Pose3d(position, new Rotation3d(0.0, pitch, yaw));
    }

    @Override
    public void simulationSubTick(int subTickNum, double subTickSeconds) {
        if (finished) {
            return;
        }

        // Semi-implicit Euler ballistic integration.
        double vz = velocity.getZ() - SimConstants.GRAVITY * subTickSeconds;
        velocity = new Translation3d(velocity.getX(), velocity.getY(), vz);
        position = position.plus(velocity.times(subTickSeconds));
        timeAirborneSeconds += subTickSeconds;

        if (targetPosition != null && position.getDistance(targetPosition) <= targetToleranceMeters) {
            if (onHitTarget != null) {
                onHitTarget.run();
            }
            finish();
            return;
        }

        if (position.getZ() <= restingHeightMeters && velocity.getZ() < 0.0) {
            onLand();
            finish();
        }
    }

    private void onLand() {
        if (targetPosition != null && onMissTarget != null) {
            onMissTarget.run();
        }
        if (onLandingFactory != null) {
            Translation3d landingSpot = new Translation3d(position.getX(), position.getY(), restingHeightMeters);
            arena.addGamePiece(onLandingFactory.apply(landingSpot));
        }
    }

    private void finish() {
        finished = true;
        arena.removeProjectile(this);
    }
}
