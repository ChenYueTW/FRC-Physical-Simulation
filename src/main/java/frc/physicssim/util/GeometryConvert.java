package frc.physicssim.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.dyn4j.geometry.Transform;
import org.dyn4j.geometry.Vector2;

/**
 * Conversions between dyn4j geometry types and WPILib geometry types.
 *
 * <p>Both libraries use meters and radians in a right-handed frame, so conversions are 1:1 with no
 * scaling. WPILib's field frame has its origin at a field corner with +X along the long axis, +Y to
 * the left, and counter-clockwise-positive rotation — identical to dyn4j's world frame as we use it.
 */
public final class GeometryConvert {
    private GeometryConvert() {}

    /** dyn4j {@link Vector2} to WPILib {@link Translation2d}. */
    public static Translation2d toWpilibTranslation(Vector2 v) {
        return new Translation2d(v.x, v.y);
    }

    /** WPILib {@link Translation2d} to dyn4j {@link Vector2}. */
    public static Vector2 toDyn4jVector(Translation2d t) {
        return new Vector2(t.getX(), t.getY());
    }

    /** dyn4j {@link Transform} (translation + rotation) to a WPILib {@link Pose2d}. */
    public static Pose2d toWpilibPose(Transform transform) {
        return new Pose2d(
                transform.getTranslationX(), transform.getTranslationY(), new Rotation2d(transform.getRotationAngle()));
    }

    /** WPILib {@link Pose2d} to a dyn4j {@link Transform}. */
    public static Transform toDyn4jTransform(Pose2d pose) {
        Transform transform = new Transform();
        transform.setTranslation(pose.getX(), pose.getY());
        transform.setRotation(pose.getRotation().getRadians());
        return transform;
    }
}
