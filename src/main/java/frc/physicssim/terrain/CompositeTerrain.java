package frc.physicssim.terrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;

/**
 * Combines several non-overlapping {@link BumpRegion}s (e.g. all the BUMPs on a field) into one
 * {@link TerrainProvider}. At any field position, the first region containing that position governs;
 * positions outside every region are flat.
 */
public class CompositeTerrain implements TerrainProvider {
    private final List<BumpRegion> regions;

    public CompositeTerrain(List<BumpRegion> regions) {
        this.regions = regions;
    }

    @Override
    public Pose3d elevate(Pose2d flatPose) {
        for (BumpRegion region : regions) {
            if (region.contains(flatPose.getTranslation())) {
                return region.elevate(flatPose);
            }
        }
        return new Pose3d(flatPose);
    }

    @Override
    public Translation2d gradient(Translation2d fieldPosition) {
        for (BumpRegion region : regions) {
            if (region.contains(fieldPosition)) {
                return region.gradient(fieldPosition);
            }
        }
        return Translation2d.kZero;
    }
}
