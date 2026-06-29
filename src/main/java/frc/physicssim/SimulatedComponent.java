package frc.physicssim;

/**
 * A piece of simulation logic that needs to run at the high physics sub-tick frequency rather than
 * once per robot loop.
 *
 * <p>Implementations are registered with the arena (see {@code SimulatedArena.addComponent}) and are
 * invoked once per sub-tick, <b>before</b> the dyn4j world is stepped, so that any forces or
 * velocities they set are integrated by that step. Examples: applying drivetrain wheel forces,
 * advancing a projectile's ballistic arc, or applying a bump region's drag.
 */
public interface SimulatedComponent {
    /**
     * Called once per physics sub-tick, before the world is stepped.
     *
     * @param subTickNum index of this sub-tick within the current robot period, counting from 0
     * @param subTickSeconds duration of this sub-tick in seconds
     */
    void simulationSubTick(int subTickNum, double subTickSeconds);
}
