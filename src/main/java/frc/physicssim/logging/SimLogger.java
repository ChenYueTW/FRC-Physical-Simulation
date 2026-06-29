package frc.physicssim.logging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import frc.physicssim.arena.SimulatedArena;
import frc.physicssim.drivetrain.AbstractDriveTrainSimulation;
import java.util.HashMap;
import java.util.Map;

/**
 * Publishes simulation state to NetworkTables as WPILib struct topics, so AdvantageScope can render
 * it on a 3D field (connect live to {@code localhost}, or open a WPILOG that captured these topics).
 *
 * <p>This helper depends only on WPILib NetworkTables — <b>not</b> on AdvantageKit. Teams using
 * AdvantageKit can skip it and instead feed the arena's pose getters straight to {@code
 * Logger.recordOutput(...)}; the record-name convention is the same ({@code FieldSimulation/*}).
 */
public class SimLogger {
    private final NetworkTable table;
    private final Map<String, StructArrayPublisher<Pose3d>> arrayPublishers = new HashMap<>();
    private StructPublisher<Pose3d> robotPose3dPublisher;
    private StructPublisher<Pose2d> robotPose2dPublisher;

    /** Publishes under the default NT instance, table {@code FieldSimulation}. */
    public SimLogger() {
        this(NetworkTableInstance.getDefault(), "FieldSimulation");
    }

    public SimLogger(NetworkTableInstance instance, String tableName) {
        this.table = instance.getTable(tableName);
    }

    /** Publishes the poses of all on-field pieces of a type to {@code <table>/<type>}. */
    public void publishGamePieces(String type, Pose3d[] poses) {
        arrayPublisher(type).set(poses);
    }

    /** Publishes the poses of all airborne pieces of a type to {@code <table>/<type>Projectiles}. */
    public void publishProjectiles(String type, Pose3d[] poses) {
        arrayPublisher(type + "Projectiles").set(poses);
    }

    /** Publishes the robot's 3D pose (raised/tilted on terrain) to {@code <table>/RobotPose3d}. */
    public void publishRobotPose(Pose3d pose) {
        if (robotPose3dPublisher == null) {
            robotPose3dPublisher =
                    table.getStructTopic("RobotPose3d", Pose3d.struct).publish();
        }
        robotPose3dPublisher.set(pose);
    }

    /** Publishes the robot's 2D pose to {@code <table>/RobotPose2d}. */
    public void publishRobotPose2d(Pose2d pose) {
        if (robotPose2dPublisher == null) {
            robotPose2dPublisher =
                    table.getStructTopic("RobotPose2d", Pose2d.struct).publish();
        }
        robotPose2dPublisher.set(pose);
    }

    /**
     * Publishes the robot pose plus, for each listed type, both the on-field pieces and the airborne
     * projectiles — the whole field state in one call.
     */
    public void update(SimulatedArena arena, AbstractDriveTrainSimulation robot, String... gamePieceTypes) {
        if (robot != null) {
            publishRobotPose(robot.getActualPose3d());
            publishRobotPose2d(robot.getActualPose());
        }
        for (String type : gamePieceTypes) {
            publishGamePieces(type, arena.getGamePiecesArrayByType(type));
            publishProjectiles(type, arena.getProjectilesArrayByType(type));
        }
    }

    private StructArrayPublisher<Pose3d> arrayPublisher(String key) {
        return arrayPublishers.computeIfAbsent(
                key, k -> table.getStructArrayTopic(k, Pose3d.struct).publish());
    }
}
