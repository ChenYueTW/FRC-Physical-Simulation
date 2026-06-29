package frc.physicssim.drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.physicssim.SimConstants;
import frc.physicssim.SimulatedComponent;
import frc.physicssim.terrain.TerrainProvider;
import frc.physicssim.util.GeometryConvert;
import org.dyn4j.dynamics.Body;
import org.dyn4j.geometry.Geometry;
import org.dyn4j.geometry.MassType;
import org.dyn4j.geometry.Rectangle;
import org.dyn4j.geometry.Vector2;

/**
 * A robot chassis as a dyn4j rigid body (a bumper rectangle with the configured mass and moment of
 * inertia). It collides with walls, the HUB, game pieces, and other drivetrains.
 *
 * <p>Subclasses decide how the robot is commanded (see {@link SwerveDriveSimulation}); this base
 * provides the body, pose/velocity readouts, and {@link #applyDriveTowardFieldVelocity} — a
 * traction-limited controller that drives the body toward a target field-relative velocity without
 * exceeding the wheels' grip. Because propulsion is force-based, the robot carries real momentum and
 * can be pushed by (and can push) obstacles and other robots.
 */
public abstract class AbstractDriveTrainSimulation extends Body implements SimulatedComponent {
    protected final DriveTrainSimulationConfig config;

    /** Maximum propelling force before the wheels slip: {@code COF * m * g}. */
    protected final double maxTractionForceNewtons;

    private TerrainProvider terrain = TerrainProvider.FLAT;

    protected AbstractDriveTrainSimulation(DriveTrainSimulationConfig config, Pose2d initialPose) {
        this.config = config;

        Rectangle bumper = Geometry.createRectangle(config.bumperLengthMeters, config.bumperWidthMeters);
        double density = config.massKg / (config.bumperLengthMeters * config.bumperWidthMeters);
        addFixture(bumper, density, 0.5, 0.2);
        setMass(MassType.NORMAL);
        getTransform().setTranslation(initialPose.getX(), initialPose.getY());
        getTransform().setRotation(initialPose.getRotation().getRadians());

        this.maxTractionForceNewtons = config.wheelCoefficientOfFriction * config.massKg * SimConstants.GRAVITY;
    }

    /** Current floor-plane pose of the chassis. */
    public Pose2d getActualPose() {
        return GeometryConvert.toWpilibPose(getTransform());
    }

    /**
     * Current 3D pose of the chassis. Flat (z=0, no tilt) on open carpet; over a {@link
     * TerrainProvider} such as a bump, this is raised and tilted for visualization.
     */
    public Pose3d getActualPose3d() {
        return terrain.elevate(getActualPose());
    }

    /** Sets the terrain the robot is driving on (default {@link TerrainProvider#FLAT}). */
    public void setTerrain(TerrainProvider terrain) {
        this.terrain = terrain;
    }

    /** Current heading of the chassis. */
    public Rotation2d getHeading() {
        return new Rotation2d(getTransform().getRotationAngle());
    }

    /** Bumper-to-bumper length along the robot's forward (+X) axis (m). */
    public double getBumperLengthMeters() {
        return config.bumperLengthMeters;
    }

    /** Bumper-to-bumper width along the robot's left (+Y) axis (m). */
    public double getBumperWidthMeters() {
        return config.bumperWidthMeters;
    }

    /** Actual field-relative velocity, as measured from the physics body. */
    public ChassisSpeeds getActualFieldSpeeds() {
        Vector2 v = getLinearVelocity();
        return new ChassisSpeeds(v.x, v.y, getAngularVelocity());
    }

    /** Actual robot-relative velocity, as measured from the physics body. */
    public ChassisSpeeds getActualRobotSpeeds() {
        return ChassisSpeeds.fromFieldRelativeSpeeds(getActualFieldSpeeds(), getHeading());
    }

    /** Teleports the chassis to a pose and zeroes its velocity (e.g. for odometry resets). */
    public void setPose(Pose2d pose) {
        getTransform().setTranslation(pose.getX(), pose.getY());
        getTransform().setRotation(pose.getRotation().getRadians());
        setLinearVelocity(0, 0);
        setAngularVelocity(0);
    }

    /**
     * Applies force and torque to drive the chassis toward a target <b>field-relative</b> velocity,
     * limited by the configured acceleration and the wheels' traction. Call once per sub-tick before
     * the world steps.
     */
    protected void applyDriveTowardFieldVelocity(
            double targetVxMps, double targetVyMps, double targetOmegaRadPerSec, double dt) {
        // ---- Translation: traction-limited velocity controller ----
        Vector2 currentVelocity = getLinearVelocity();
        double ax = (targetVxMps - currentVelocity.x) / dt;
        double ay = (targetVyMps - currentVelocity.y) / dt;
        double accelMag = Math.hypot(ax, ay);
        if (accelMag > config.maxLinearAccelMps2 && accelMag > 1e-9) {
            double scale = config.maxLinearAccelMps2 / accelMag;
            ax *= scale;
            ay *= scale;
        }
        double fx = config.massKg * ax;
        double fy = config.massKg * ay;
        double forceMag = Math.hypot(fx, fy);
        if (forceMag > maxTractionForceNewtons && forceMag > 1e-9) {
            double scale = maxTractionForceNewtons / forceMag;
            fx *= scale;
            fy *= scale;
        }
        applyForce(new Vector2(fx, fy));

        // ---- Terrain: along-slope component of gravity (resists climbing, assists descending) ----
        Translation2d gradient = terrain.gradient(getActualPose().getTranslation());
        if (gradient.getNorm() > 1e-9) {
            applyForce(new Vector2(
                    -config.massKg * SimConstants.GRAVITY * gradient.getX(),
                    -config.massKg * SimConstants.GRAVITY * gradient.getY()));
        }

        // ---- Rotation: acceleration-limited angular velocity controller ----
        double alpha = (targetOmegaRadPerSec - getAngularVelocity()) / dt;
        alpha = Math.max(-config.maxAngularAccelRadPerSec2, Math.min(config.maxAngularAccelRadPerSec2, alpha));
        applyTorque(config.momentOfInertia() * alpha);
    }
}
