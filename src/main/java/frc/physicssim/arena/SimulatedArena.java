package frc.physicssim.arena;

import edu.wpi.first.math.geometry.Pose3d;
import frc.physicssim.SimConstants;
import frc.physicssim.SimulatedComponent;
import frc.physicssim.drivetrain.AbstractDriveTrainSimulation;
import frc.physicssim.gamepieces.GamePieceOnField;
import java.util.ArrayList;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Set;
import org.dyn4j.dynamics.Body;
import org.dyn4j.world.PhysicsWorld;
import org.dyn4j.world.World;

/**
 * The heart of the simulator: a dyn4j physics world plus the registry of everything in it (field
 * obstacles, game pieces, drivetrains, and high-frequency components).
 *
 * <p>This class is abstract; instantiate a concrete season arena such as {@link Arena2026Rebuilt}.
 * A convenience singleton is available via {@link #getInstance()}.
 *
 * <h2>Stepping</h2>
 *
 * <p>Call {@link #simulationPeriodic()} exactly once per robot loop (from {@code
 * simulationPeriodic()} of your {@code TimedRobot}/{@code LoggedRobot}). Each call runs several
 * physics sub-ticks (default 5 -> 250 Hz). Gravity is zero because the world is top-down (X-Y);
 * vertical motion is handled by overlay components such as projectiles and bump regions.
 */
public abstract class SimulatedArena {
    private static SimulatedArena instance;

    /**
     * The convenience singleton arena, lazily created as an {@link Arena2026Rebuilt}. Override with
     * {@link #setInstance(SimulatedArena)} to use a different field.
     */
    public static synchronized SimulatedArena getInstance() {
        if (instance == null) {
            instance = new Arena2026Rebuilt();
        }
        return instance;
    }

    /** Replaces the convenience singleton. */
    public static synchronized void setInstance(SimulatedArena arena) {
        instance = arena;
    }

    private final World<Body> world;
    private final Set<GamePieceOnField> gamePieces = new LinkedHashSet<>();
    private final List<SimulatedComponent> components = new ArrayList<>();

    private double periodSeconds = SimConstants.DEFAULT_PERIOD_SECONDS;
    private int subTicks = SimConstants.DEFAULT_SUB_TICKS;

    /** Builds the world from a season field map. */
    protected SimulatedArena(FieldMap fieldMap) {
        this.world = new World<>();
        this.world.setGravity(PhysicsWorld.ZERO_GRAVITY);
        for (Body obstacle : fieldMap.obstacles()) {
            this.world.addBody(obstacle);
        }
    }

    /** The underlying dyn4j world — for advanced use (contact listeners, custom bodies). */
    public World<Body> world() {
        return world;
    }

    /**
     * Sets the robot loop period and number of physics sub-ticks per period.
     *
     * <p><b>If you log with AdvantageKit, leave this at the defaults</b> (20 ms / 5 sub-ticks);
     * AdvantageKit's deterministic replay assumes a fixed cadence.
     */
    public synchronized void setTiming(double robotPeriodSeconds, int subTicksPerPeriod) {
        this.periodSeconds = robotPeriodSeconds;
        this.subTicks = subTicksPerPeriod;
    }

    /** Advances the simulation by one robot period. Call once per robot loop. */
    public synchronized void simulationPeriodic() {
        double subDt = periodSeconds / subTicks;
        for (int i = 0; i < subTicks; i++) {
            // Components set forces/velocities (and may add/remove pieces) before the world steps,
            // so we iterate a snapshot to tolerate concurrent registration changes.
            for (SimulatedComponent component : new ArrayList<>(components)) {
                component.simulationSubTick(i, subDt);
            }
            world.step(1, subDt);
        }
    }

    // ---- Game pieces ----

    /** Adds a game piece to the field; it becomes an active collision body immediately. */
    public synchronized void addGamePiece(GamePieceOnField gamePiece) {
        world.addBody(gamePiece);
        gamePieces.add(gamePiece);
    }

    /** Removes a game piece from the field. Returns whether it was present. */
    public synchronized boolean removeGamePiece(GamePieceOnField gamePiece) {
        gamePieces.remove(gamePiece);
        return world.removeBody(gamePiece);
    }

    /** Removes every game piece from the field (does not affect obstacles or drivetrains). */
    public synchronized void clearGamePieces() {
        for (GamePieceOnField gamePiece : gamePieces) {
            world.removeBody(gamePiece);
        }
        gamePieces.clear();
    }

    /** A live view of the game pieces currently on the field. */
    public synchronized Set<GamePieceOnField> gamePiecesOnField() {
        return gamePieces;
    }

    /** Poses of all on-field game pieces of the given type, e.g. for {@code Logger.recordOutput}. */
    public synchronized List<Pose3d> getGamePiecesByType(String type) {
        List<Pose3d> poses = new ArrayList<>();
        for (GamePieceOnField gamePiece : gamePieces) {
            if (gamePiece.type().equals(type)) {
                poses.add(gamePiece.pose3d());
            }
        }
        return poses;
    }

    /** Array form of {@link #getGamePiecesByType(String)}, convenient for AdvantageScope logging. */
    public synchronized Pose3d[] getGamePiecesArrayByType(String type) {
        return getGamePiecesByType(type).toArray(new Pose3d[0]);
    }

    // ---- High-frequency components (drivetrains, projectiles, terrain) ----

    /** Registers a component to be sub-ticked each physics step. */
    public synchronized void addComponent(SimulatedComponent component) {
        components.add(component);
    }

    /** Unregisters a component. Returns whether it was present. */
    public synchronized boolean removeComponent(SimulatedComponent component) {
        return components.remove(component);
    }

    // ---- Drivetrains ----

    /**
     * Adds a drivetrain to the field: its body becomes an active collision body and it is sub-ticked
     * each physics step so its commanded motion is applied.
     */
    public synchronized void addDriveTrain(AbstractDriveTrainSimulation driveTrain) {
        world.addBody(driveTrain);
        components.add(driveTrain);
    }

    /** Removes a drivetrain from the field. Returns whether it was present. */
    public synchronized boolean removeDriveTrain(AbstractDriveTrainSimulation driveTrain) {
        components.remove(driveTrain);
        return world.removeBody(driveTrain);
    }
}
