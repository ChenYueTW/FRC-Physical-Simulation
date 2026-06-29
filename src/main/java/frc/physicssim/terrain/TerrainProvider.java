package frc.physicssim.terrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * A vertical-terrain overlay on the otherwise-flat 2D field. dyn4j only knows the floor plane, so a
 * provider supplies the two things vertical terrain adds:
 *
 * <ul>
 *   <li>a 3D pose ({@link #elevate}) — the robot's height and tilt for visualization, and
 *   <li>a surface {@link #gradient} — so the drivetrain can apply the along-slope component of
 *       gravity, which slows a climbing robot and lets it roll back if it stalls.
 * </ul>
 */
public interface TerrainProvider {
    /** Flat ground: no height, no tilt, no slope. */
    TerrainProvider FLAT = new TerrainProvider() {
        @Override
        public Pose3d elevate(Pose2d flatPose) {
            return new Pose3d(flatPose);
        }

        @Override
        public Translation2d gradient(Translation2d fieldPosition) {
            return Translation2d.kZero;
        }
    };

    /** Maps a flat floor pose to the 3D pose the robot actually assumes on the terrain. */
    Pose3d elevate(Pose2d flatPose);

    /**
     * Surface height gradient {@code (dz/dx, dz/dy)} at a field position; {@code (0, 0)} on the flat.
     * The along-slope gravity force on a body of mass {@code m} is approximately {@code -m*g*}this.
     */
    Translation2d gradient(Translation2d fieldPosition);
}
