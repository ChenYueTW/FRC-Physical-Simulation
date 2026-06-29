package frc.physicssim.drivetrain;

import edu.wpi.first.math.util.Units;

/**
 * Physical configuration of a simulated drivetrain. Reasonable defaults approximate a ~45 kg
 * competition swerve robot; override with the fluent {@code with...} setters.
 */
public class DriveTrainSimulationConfig {
    /** Robot mass including bumpers and battery (kg). */
    public double massKg = 45.0;

    /** Bumper-to-bumper length along the robot's +X (forward) axis (m). */
    public double bumperLengthMeters = 0.9;

    /** Bumper-to-bumper width along the robot's +Y (left) axis (m). */
    public double bumperWidthMeters = 0.9;

    /** Maximum achievable chassis translational speed (m/s). */
    public double maxLinearVelocityMps = 4.5;

    /** Maximum chassis translational acceleration on open carpet (m/s^2). */
    public double maxLinearAccelMps2 = 8.0;

    /** Maximum chassis rotational speed (rad/s). */
    public double maxAngularVelocityRadPerSec = Units.degreesToRadians(540);

    /** Maximum chassis rotational acceleration (rad/s^2). */
    public double maxAngularAccelRadPerSec2 = Units.degreesToRadians(1440);

    /**
     * Coefficient of friction between the wheels and carpet. Caps the propelling force the wheels can
     * apply before slipping ({@code F_max = COF * m * g}); also governs how hard the robot can push
     * obstacles and other robots.
     */
    public double wheelCoefficientOfFriction = 1.1;

    public DriveTrainSimulationConfig withMass(double massKg) {
        this.massKg = massKg;
        return this;
    }

    public DriveTrainSimulationConfig withBumperSize(double lengthMeters, double widthMeters) {
        this.bumperLengthMeters = lengthMeters;
        this.bumperWidthMeters = widthMeters;
        return this;
    }

    public DriveTrainSimulationConfig withMaxLinearVelocity(double mps) {
        this.maxLinearVelocityMps = mps;
        return this;
    }

    public DriveTrainSimulationConfig withMaxLinearAcceleration(double mps2) {
        this.maxLinearAccelMps2 = mps2;
        return this;
    }

    public DriveTrainSimulationConfig withMaxAngularVelocity(double radPerSec) {
        this.maxAngularVelocityRadPerSec = radPerSec;
        return this;
    }

    public DriveTrainSimulationConfig withMaxAngularAcceleration(double radPerSec2) {
        this.maxAngularAccelRadPerSec2 = radPerSec2;
        return this;
    }

    public DriveTrainSimulationConfig withWheelCoefficientOfFriction(double cof) {
        this.wheelCoefficientOfFriction = cof;
        return this;
    }

    /** Approximate moment of inertia of the bumper rectangle about its center (kg·m^2). */
    public double momentOfInertia() {
        return massKg * (bumperLengthMeters * bumperLengthMeters + bumperWidthMeters * bumperWidthMeters) / 12.0;
    }
}
