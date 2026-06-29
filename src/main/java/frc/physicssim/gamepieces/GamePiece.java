package frc.physicssim.gamepieces;

import edu.wpi.first.math.geometry.Pose3d;

/**
 * Anything that can be reported to AdvantageScope as a positioned game piece — whether resting on
 * the field, held by a robot, or flying through the air.
 */
public interface GamePiece {
    /**
     * The game-piece type, used to group pieces for logging (e.g. {@code "Fuel"}). Pieces sharing a
     * type are published together as one {@code Pose3d[]}.
     */
    String type();

    /** Current 3D pose of the piece in the WPILib field frame. */
    Pose3d pose3d();
}
