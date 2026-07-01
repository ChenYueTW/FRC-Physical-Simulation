package frc.physicssim.arena;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.physicssim.SimConstants;
import frc.physicssim.SimConstants.Rebuilt2026;
import frc.physicssim.gamepieces.rebuilt.RebuiltFuelOnField;
import frc.physicssim.terrain.BumpRegion;
import java.util.Random;
import org.dyn4j.geometry.Geometry;

/**
 * The 2026 <b>REBUILT</b> arena: a 16.54 x 8.07 m field bounded by perimeter walls, with each
 * alliance's 47 in square HUB as a collision obstacle (blue at ~4.03 m from the blue wall, red
 * mirrored), and helpers to (re)generate the starting FUEL.
 *
 * <p>Dimensions and positions come from the official 2026 Game Manual / field drawings. Values still
 * marked {@code TODO} (the exact DEPOT position along the wall, the full multi-BUMP layout, the HUB
 * opening height) are approximations pending the field CAD.
 */
public class Arena2026Rebuilt extends SimulatedArena {
    private static final double WALL_THICKNESS_METERS = 0.1;

    // BUMP crossable by the demo: the blue-side bump on the -Y side of the blue HUB, aligned with the
    // HUB in X. TODO: the real field has bumps flanking each HUB on both sides.
    private static final double BUMP_CENTER_X = Rebuilt2026.HUB_DISTANCE_FROM_WALL_METERS;
    private static final double BUMP_CENTER_Y = 2.5;

    public Arena2026Rebuilt() {
        super(buildFieldMap());
    }

    private static FieldMap buildFieldMap() {
        FieldMap map = new FieldMap();
        map.addPerimeterWalls(Rebuilt2026.FIELD_LENGTH_METERS, Rebuilt2026.FIELD_WIDTH_METERS, WALL_THICKNESS_METERS);
        // Both HUBs are solid square obstacles robots and FUEL bounce off.
        addHub(map, Rebuilt2026.BLUE_HUB_CENTER);
        addHub(map, Rebuilt2026.RED_HUB_CENTER);
        return map;
    }

    private static void addHub(FieldMap map, Translation2d center) {
        map.addObstacle(
                Geometry.createRectangle(Rebuilt2026.HUB_SIZE_METERS, Rebuilt2026.HUB_SIZE_METERS),
                new Pose2d(center, Rotation2d.kZero),
                SimConstants.CARPET_FRICTION_COEFFICIENT);
    }

    /**
     * A representative REBUILT BUMP (official 1.854 x 1.128 x 0.165 m), on the -Y side of the blue
     * HUB. Attach it with {@code drivetrain.setTerrain(Arena2026Rebuilt.rebuiltBump())} so the robot
     * tilts and slows crossing it.
     */
    public static BumpRegion rebuiltBump() {
        return new BumpRegion(
                BUMP_CENTER_X - Rebuilt2026.BUMP_DEPTH_METERS / 2.0,
                BUMP_CENTER_X + Rebuilt2026.BUMP_DEPTH_METERS / 2.0,
                BUMP_CENTER_Y - Rebuilt2026.BUMP_WIDTH_METERS / 2.0,
                BUMP_CENTER_Y + Rebuilt2026.BUMP_WIDTH_METERS / 2.0,
                Rebuilt2026.BUMP_HEIGHT_METERS);
    }

    /**
     * Clears every FUEL and regenerates the starting layout at the <b>official</b> counts: the
     * NEUTRAL ZONE pile plus 24 FUEL in each alliance's DEPOT. (OUTPOST chutes are not simulated.)
     *
     * <p><b>Heads up:</b> this spawns ~{@value Rebuilt2026#NEUTRAL_ZONE_FUEL_COUNT} + 48 dyn4j bodies
     * — realistic, but heavy for real-time play. For a smooth demo pass smaller counts to {@link
     * #regenerateFieldGamePieces(int, int)}.
     */
    public void regenerateFieldGamePieces() {
        regenerateFieldGamePieces(Rebuilt2026.NEUTRAL_ZONE_FUEL_COUNT, Rebuilt2026.DEPOT_FUEL_COUNT);
    }

    /**
     * Clears every FUEL and regenerates the center NEUTRAL ZONE pile and both DEPOTs with the given
     * counts (OUTPOST chutes are not simulated). FUEL is laid out non-overlapping in a jittered grid,
     * since the 2D world cannot stack a real 3D pile.
     *
     * @param neutralZoneCount FUEL to scatter in the center NEUTRAL ZONE
     * @param depotCountPerAlliance FUEL to place in each alliance's DEPOT
     */
    public void regenerateFieldGamePieces(int neutralZoneCount, int depotCountPerAlliance) {
        clearGamePieces();

        // Center NEUTRAL ZONE.
        scatterFuelInBox(
                Rebuilt2026.FIELD_CENTER.getX(),
                Rebuilt2026.FIELD_CENTER_Y,
                Rebuilt2026.NEUTRAL_ZONE_DEPTH_METERS,
                Rebuilt2026.NEUTRAL_ZONE_WIDTH_METERS,
                neutralZoneCount);

        // Blue DEPOT (against the x=0 wall) and red DEPOT (against the far wall, mirrored in Y).
        scatterFuelInBox(
                Rebuilt2026.DEPOT_DEPTH_METERS / 2.0,
                Rebuilt2026.DEPOT_CENTER_Y,
                Rebuilt2026.DEPOT_DEPTH_METERS,
                Rebuilt2026.DEPOT_WIDTH_METERS,
                depotCountPerAlliance);
        scatterFuelInBox(
                Rebuilt2026.FIELD_LENGTH_METERS - Rebuilt2026.DEPOT_DEPTH_METERS / 2.0,
                Rebuilt2026.FIELD_WIDTH_METERS - Rebuilt2026.DEPOT_CENTER_Y,
                Rebuilt2026.DEPOT_DEPTH_METERS,
                Rebuilt2026.DEPOT_WIDTH_METERS,
                depotCountPerAlliance);
    }

    /** Kept for compatibility: lays out the official starting FUEL. */
    public void resetFieldForAuto() {
        regenerateFieldGamePieces();
    }

    /**
     * Places {@code count} FUEL in a non-overlapping jittered grid filling a box (centered at {@code
     * cx,cy}, spanning {@code sizeX} along X and {@code sizeY} along Y).
     */
    private void scatterFuelInBox(double cx, double cy, double sizeX, double sizeY, int count) {
        if (count <= 0) {
            return;
        }
        double minSpacing = Rebuilt2026.FUEL_DIAMETER_METERS * 1.05;
        int cols = Math.max(1, (int) Math.round(Math.sqrt((double) count * sizeY / sizeX)));
        int rows = (int) Math.ceil((double) count / cols);
        double spacingX = sizeX / rows;
        double spacingY = sizeY / cols;
        Random rng = new Random(31L * count + Double.hashCode(cx));

        int placed = 0;
        for (int r = 0; r < rows && placed < count; r++) {
            for (int c = 0; c < cols && placed < count; c++) {
                double x = cx - sizeX / 2.0 + spacingX * (r + 0.5);
                double y = cy - sizeY / 2.0 + spacingY * (c + 0.5);
                // Jitter only within the slack beyond the minimum spacing, so pieces never overlap.
                x += (rng.nextDouble() - 0.5) * Math.max(0.0, spacingX - minSpacing);
                y += (rng.nextDouble() - 0.5) * Math.max(0.0, spacingY - minSpacing);
                addGamePiece(new RebuiltFuelOnField(new Translation2d(x, y)));
                placed++;
            }
        }
    }
}
