package frc.physicssim.gamepieces.rebuilt;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.physicssim.SimConstants.Rebuilt2026;
import frc.physicssim.arena.SimulatedArena;
import frc.physicssim.gamepieces.GamePieceProjectile;

/**
 * A 2026 REBUILT <b>FUEL</b> ball in flight. When it lands without hitting a target it becomes a
 * {@link RebuiltFuelOnField} where it touched down.
 */
public class RebuiltFuelProjectile extends GamePieceProjectile {
    public RebuiltFuelProjectile(SimulatedArena arena, Translation3d launchPosition, Translation3d launchVelocity) {
        super(arena, RebuiltFuelOnField.TYPE, launchPosition, launchVelocity);
        withRestingHeight(Rebuilt2026.FUEL_RADIUS_METERS);
        becomesOnFieldOnLanding(landing -> new RebuiltFuelOnField(new Translation2d(landing.getX(), landing.getY())));
    }

    /**
     * Builds a FUEL projectile from launch kinematics.
     *
     * @param launchPosition the muzzle position in the field frame (m)
     * @param launchSpeedMps exit speed of the ball (m/s)
     * @param yaw horizontal launch direction in the field frame
     * @param pitchRadians launch elevation above horizontal (rad)
     * @param addedChassisVelocity robot velocity added to the shot (use zero to ignore); only X/Y
     *     are physically meaningful for a ground robot
     */
    public static RebuiltFuelProjectile fromLaunch(
            SimulatedArena arena,
            Translation3d launchPosition,
            double launchSpeedMps,
            Rotation2d yaw,
            double pitchRadians,
            Translation3d addedChassisVelocity) {
        double horizontal = launchSpeedMps * Math.cos(pitchRadians);
        double vz = launchSpeedMps * Math.sin(pitchRadians);
        Translation3d velocity = new Translation3d(
                horizontal * yaw.getCos() + addedChassisVelocity.getX(),
                horizontal * yaw.getSin() + addedChassisVelocity.getY(),
                vz + addedChassisVelocity.getZ());
        return new RebuiltFuelProjectile(arena, launchPosition, velocity);
    }
}
