package frc.physicssim.terrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * A raised bump spanning the field laterally (in Y) that the robot drives over along X. Its height
 * follows a smooth hump, {@code h(x) = peak * sin(pi * u)} where {@code u} runs 0..1 across the
 * bump, so the robot climbs, crests, and descends.
 *
 * <p>Attach it to a drivetrain with {@code drivetrain.setTerrain(bump)}. The drivetrain then (a)
 * reports a tilted, raised {@link Pose3d} while on the bump — so AdvantageScope shows it pitch up
 * and over — and (b) feels the along-slope component of gravity, which slows the climb and pushes it
 * back if it stalls.
 *
 * <p>The tilt is exact for a head-on (heading ≈ 0) crossing — the typical case — and approximate at
 * an angle.
 */
public class BumpRegion implements TerrainProvider {
    private final double xStart;
    private final double xEnd;
    private final double yMin;
    private final double yMax;
    private final double peakHeightMeters;
    private final double width;

    /**
     * @param xStart field X where the bump begins (m)
     * @param xEnd field X where the bump ends (m)
     * @param yMin field Y of one lateral edge (m)
     * @param yMax field Y of the other lateral edge (m)
     * @param peakHeightMeters crest height above the carpet (m)
     */
    public BumpRegion(double xStart, double xEnd, double yMin, double yMax, double peakHeightMeters) {
        this.xStart = xStart;
        this.xEnd = xEnd;
        this.yMin = Math.min(yMin, yMax);
        this.yMax = Math.max(yMin, yMax);
        this.peakHeightMeters = peakHeightMeters;
        this.width = xEnd - xStart;
    }

    /** Whether a field position is over the bump footprint. */
    public boolean contains(Translation2d position) {
        return position.getX() >= xStart
                && position.getX() <= xEnd
                && position.getY() >= yMin
                && position.getY() <= yMax;
    }

    /** Crest-profile height at a field position (0 off the bump). */
    public double heightAt(Translation2d position) {
        if (!contains(position)) {
            return 0.0;
        }
        double u = (position.getX() - xStart) / width;
        return peakHeightMeters * Math.sin(Math.PI * u);
    }

    @Override
    public Translation2d gradient(Translation2d position) {
        if (!contains(position)) {
            return Translation2d.kZero;
        }
        double u = (position.getX() - xStart) / width;
        double dzdx = peakHeightMeters * (Math.PI / width) * Math.cos(Math.PI * u);
        return new Translation2d(dzdx, 0.0);
    }

    @Override
    public Pose3d elevate(Pose2d flatPose) {
        Translation2d g = gradient(flatPose.getTranslation());
        if (g.getNorm() == 0.0) {
            return new Pose3d(flatPose);
        }
        double z = heightAt(flatPose.getTranslation());
        // Climbing (+dz/dx) pitches the nose up, which is a negative rotation about field +Y.
        double pitch = -Math.atan(g.getX());
        double roll = Math.atan(g.getY());
        return new Pose3d(
                flatPose.getX(),
                flatPose.getY(),
                z,
                new Rotation3d(roll, pitch, flatPose.getRotation().getRadians()));
    }
}
