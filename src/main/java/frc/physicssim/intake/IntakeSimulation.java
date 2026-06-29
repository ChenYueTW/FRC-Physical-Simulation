package frc.physicssim.intake;

import edu.wpi.first.math.geometry.Translation2d;
import frc.physicssim.SimulatedComponent;
import frc.physicssim.arena.SimulatedArena;
import frc.physicssim.drivetrain.AbstractDriveTrainSimulation;
import frc.physicssim.gamepieces.GamePieceOnField;
import java.util.ArrayList;
import java.util.List;

/**
 * Simulates an intake as a rectangular pickup region rigidly attached to a robot. While the intake
 * is {@linkplain #setRunning(boolean) running} and below capacity, any matching game piece whose
 * center enters the region is removed from the field and added to the robot's internal storage.
 *
 * <p>The region is expressed in the robot's frame and transformed by the live robot pose each
 * sub-tick, so it follows the robot as it drives and turns. Captured pieces are tracked as a count;
 * a shooter can later {@link #obtainGamePieceFromStorage()} to launch one.
 */
public class IntakeSimulation implements SimulatedComponent {
    /** Which edge of the robot the intake sits on. */
    public enum Side {
        FRONT,
        BACK,
        LEFT,
        RIGHT
    }

    private final SimulatedArena arena;
    private final AbstractDriveTrainSimulation robot;
    private final String gamePieceType;
    private final int capacity;

    // Pickup region as an axis-aligned box in the robot frame (meters).
    private final double minX;
    private final double maxX;
    private final double minY;
    private final double maxY;

    private boolean running = false;
    private int storedCount = 0;

    private IntakeSimulation(
            SimulatedArena arena,
            AbstractDriveTrainSimulation robot,
            String gamePieceType,
            int capacity,
            double minX,
            double maxX,
            double minY,
            double maxY) {
        this.arena = arena;
        this.robot = robot;
        this.gamePieceType = gamePieceType;
        this.capacity = capacity;
        this.minX = minX;
        this.maxX = maxX;
        this.minY = minY;
        this.maxY = maxY;
        arena.addComponent(this);
    }

    /**
     * An intake that extends beyond the bumper on one side (the typical "over-the-bumper" roller).
     *
     * @param widthMeters span of the intake along the chosen edge
     * @param extensionMeters how far the intake reaches outward past the bumper
     */
    public static IntakeSimulation overTheBumperIntake(
            SimulatedArena arena,
            AbstractDriveTrainSimulation robot,
            String gamePieceType,
            Side side,
            double widthMeters,
            double extensionMeters,
            int capacity) {
        double halfLen = robot.getBumperLengthMeters() / 2.0;
        double halfWid = robot.getBumperWidthMeters() / 2.0;
        return build(
                arena,
                robot,
                gamePieceType,
                capacity,
                side,
                widthMeters,
                halfLen,
                halfWid,
                halfLen + extensionMeters,
                halfWid + extensionMeters);
    }

    /**
     * An intake contained within the robot's frame (game pieces are grabbed once they reach the
     * bumper line).
     *
     * @param widthMeters span of the intake along the chosen edge
     * @param depthMeters how far the intake reaches inward from the bumper
     */
    public static IntakeSimulation inTheFrameIntake(
            SimulatedArena arena,
            AbstractDriveTrainSimulation robot,
            String gamePieceType,
            Side side,
            double widthMeters,
            double depthMeters,
            int capacity) {
        double halfLen = robot.getBumperLengthMeters() / 2.0;
        double halfWid = robot.getBumperWidthMeters() / 2.0;
        return build(
                arena,
                robot,
                gamePieceType,
                capacity,
                side,
                widthMeters,
                halfLen - depthMeters,
                halfWid - depthMeters,
                halfLen,
                halfWid);
    }

    private static IntakeSimulation build(
            SimulatedArena arena,
            AbstractDriveTrainSimulation robot,
            String gamePieceType,
            int capacity,
            Side side,
            double widthMeters,
            double innerEdgeX,
            double innerEdgeY,
            double outerEdgeX,
            double outerEdgeY) {
        double halfWidth = widthMeters / 2.0;
        switch (side) {
            case FRONT:
                return new IntakeSimulation(
                        arena, robot, gamePieceType, capacity, innerEdgeX, outerEdgeX, -halfWidth, halfWidth);
            case BACK:
                return new IntakeSimulation(
                        arena, robot, gamePieceType, capacity, -outerEdgeX, -innerEdgeX, -halfWidth, halfWidth);
            case LEFT:
                return new IntakeSimulation(
                        arena, robot, gamePieceType, capacity, -halfWidth, halfWidth, innerEdgeY, outerEdgeY);
            case RIGHT:
            default:
                return new IntakeSimulation(
                        arena, robot, gamePieceType, capacity, -halfWidth, halfWidth, -outerEdgeY, -innerEdgeY);
        }
    }

    /** Turns the intake roller on or off. */
    public void setRunning(boolean running) {
        this.running = running;
    }

    public boolean isRunning() {
        return running;
    }

    /** Number of game pieces currently held in the robot. */
    public int getStoredCount() {
        return storedCount;
    }

    public boolean isFull() {
        return storedCount >= capacity;
    }

    /**
     * Removes one stored game piece (e.g. for a shooter to launch).
     *
     * @return {@code true} if a piece was available and removed
     */
    public boolean obtainGamePieceFromStorage() {
        if (storedCount > 0) {
            storedCount--;
            return true;
        }
        return false;
    }

    /** Manually preloads stored game pieces, up to capacity. */
    public void addToStorage(int count) {
        storedCount = Math.min(capacity, storedCount + count);
    }

    @Override
    public void simulationSubTick(int subTickNum, double subTickSeconds) {
        if (!running || isFull()) {
            return;
        }

        List<GamePieceOnField> captured = new ArrayList<>();
        for (GamePieceOnField piece : arena.gamePiecesOnField()) {
            if (!piece.type().equals(gamePieceType)) {
                continue;
            }
            if (isInsideRegion(piece.pose2d().getTranslation())) {
                captured.add(piece);
                if (storedCount + captured.size() >= capacity) {
                    break;
                }
            }
        }

        for (GamePieceOnField piece : captured) {
            if (arena.removeGamePiece(piece)) {
                storedCount++;
            }
        }
    }

    /** Tests whether a field-frame point lies within the intake region (transformed by robot pose). */
    private boolean isInsideRegion(Translation2d fieldPoint) {
        // Express the point in the robot frame.
        Translation2d relative = fieldPoint
                .minus(robot.getActualPose().getTranslation())
                .rotateBy(robot.getHeading().unaryMinus());
        double rx = relative.getX();
        double ry = relative.getY();
        return rx >= minX && rx <= maxX && ry >= minY && ry <= maxY;
    }
}
