package frc.physicssim.drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * A holonomic (swerve) drivetrain simulation. Commands are chassis-level velocity setpoints; the
 * traction-limited controller in {@link AbstractDriveTrainSimulation} turns them into forces, so the
 * robot accelerates, slips, and collides realistically.
 *
 * <p>This is a deliberately simplified swerve model — it does not simulate individual modules. It
 * captures the behaviors that matter for field interaction (momentum, traction limits, and
 * collision push-back). A per-module dynamics model can be layered on later behind the same body.
 */
public class SwerveDriveSimulation extends AbstractDriveTrainSimulation {
    private ChassisSpeeds desiredRobotRelativeSpeeds = new ChassisSpeeds();

    public SwerveDriveSimulation(DriveTrainSimulationConfig config, Pose2d initialPose) {
        super(config, initialPose);
    }

    /** Commands a robot-relative chassis velocity (vx forward, vy left, omega CCW+). */
    public void setRobotSpeeds(ChassisSpeeds robotRelative) {
        this.desiredRobotRelativeSpeeds = clampToLimits(robotRelative);
    }

    /** Commands a field-relative chassis velocity. */
    public void setFieldSpeeds(ChassisSpeeds fieldRelative) {
        setRobotSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelative, getHeading()));
    }

    /** Commands the robot to stop. */
    public void stop() {
        setRobotSpeeds(new ChassisSpeeds());
    }

    /** The most recently commanded robot-relative setpoint (after clamping to limits). */
    public ChassisSpeeds getCommandedRobotSpeeds() {
        return desiredRobotRelativeSpeeds;
    }

    private ChassisSpeeds clampToLimits(ChassisSpeeds speeds) {
        double vx = speeds.vxMetersPerSecond;
        double vy = speeds.vyMetersPerSecond;
        double linearMag = Math.hypot(vx, vy);
        if (linearMag > config.maxLinearVelocityMps && linearMag > 1e-9) {
            double scale = config.maxLinearVelocityMps / linearMag;
            vx *= scale;
            vy *= scale;
        }
        double omega = Math.max(
                -config.maxAngularVelocityRadPerSec,
                Math.min(config.maxAngularVelocityRadPerSec, speeds.omegaRadiansPerSecond));
        return new ChassisSpeeds(vx, vy, omega);
    }

    @Override
    public void simulationSubTick(int subTickNum, double subTickSeconds) {
        // Convert the robot-relative setpoint to field frame using the live heading, so commands
        // track correctly while the robot rotates.
        ChassisSpeeds field = ChassisSpeeds.fromRobotRelativeSpeeds(desiredRobotRelativeSpeeds, getHeading());
        applyDriveTowardFieldVelocity(
                field.vxMetersPerSecond, field.vyMetersPerSecond, field.omegaRadiansPerSecond, subTickSeconds);
    }
}
