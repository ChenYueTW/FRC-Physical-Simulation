package frc.physicssim.arena;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.physicssim.SimConstants;
import frc.physicssim.SimConstants.Rebuilt2026;
import frc.physicssim.gamepieces.rebuilt.RebuiltFuelOnField;
import frc.physicssim.terrain.BumpRegion;
import frc.physicssim.terrain.CompositeTerrain;
import java.util.List;
import java.util.Random;
import org.dyn4j.geometry.Geometry;

/**
 * The 2026 <b>REBUILT</b> arena: a 16.54 x 8.07 m field bounded by perimeter walls, with each
 * alliance's 47 in square HUB and both TRENCH support legs as collision obstacles, and helpers to
 * (re)generate the starting FUEL.
 *
 * <p>Dimensions and positions come from the official field-dimension drawings (FE-2026) and the
 * AprilTag coordinate table, cross-checked against each other. Values still marked {@code TODO} (the
 * exact DEPOT position along the wall, the HUB opening height) are approximations pending further
 * confirmation.
 */
public class Arena2026Rebuilt extends SimulatedArena {
    private static final double WALL_THICKNESS_METERS = 0.1;

    // HUB and TRENCH sit on the same X (distance from the alliance wall); BUMP has its own (see
    // Rebuilt2026.BUMP_DISTANCE_FROM_WALL_METERS).
    private static final double ELEMENT_ROW_X = Rebuilt2026.HUB_DISTANCE_FROM_WALL_METERS;

    public Arena2026Rebuilt() {
        super(buildFieldMap());
        // All four BUMPs apply automatically to every drivetrain/game piece added to this arena.
        setTerrain(rebuiltBumps());
    }

    private static FieldMap buildFieldMap() {
        FieldMap map = new FieldMap();
        map.addPerimeterWalls(Rebuilt2026.FIELD_LENGTH_METERS, Rebuilt2026.FIELD_WIDTH_METERS, WALL_THICKNESS_METERS);
        // Both HUBs are solid square obstacles robots and FUEL bounce off.
        addHub(map, Rebuilt2026.BLUE_HUB_CENTER);
        addHub(map, Rebuilt2026.RED_HUB_CENTER);
        // Both TRENCHes' support legs (the passage underneath is left clear).
        addTrenchLegs(
                map,
                Rebuilt2026.FIELD_LENGTH_METERS - ELEMENT_ROW_X,
                Rebuilt2026.TRENCH_NEAR_FIELD_EDGE_OUTER_Y_METERS,
                +1.0);
        addTrenchLegs(
                map,
                Rebuilt2026.FIELD_LENGTH_METERS - ELEMENT_ROW_X,
                Rebuilt2026.TRENCH_FAR_FIELD_EDGE_OUTER_Y_METERS,
                -1.0);
        addTrenchLegs(map, ELEMENT_ROW_X, Rebuilt2026.TRENCH_NEAR_FIELD_EDGE_OUTER_Y_METERS, +1.0);
        addTrenchLegs(map, ELEMENT_ROW_X, Rebuilt2026.TRENCH_FAR_FIELD_EDGE_OUTER_Y_METERS, -1.0);
        return map;
    }

    private static void addHub(FieldMap map, Translation2d center) {
        map.addObstacle(
                Geometry.createRectangle(Rebuilt2026.HUB_SIZE_METERS, Rebuilt2026.HUB_SIZE_METERS),
                new Pose2d(center, Rotation2d.kZero),
                SimConstants.CARPET_FRICTION_COEFFICIENT);
    }

    /**
     * A TRENCH's two solid support legs, with the {@value Rebuilt2026#TRENCH_PASSAGE_WIDTH_METERS} m
     * passage between them left open so robots can drive underneath.
     *
     * @param outerEdgeY the trench's edge closest to the field's own long edge (not its center — see
     *     {@link Rebuilt2026#TRENCH_NEAR_FIELD_EDGE_OUTER_Y_METERS})
     * @param towardCenterSign {@code +1} if the trench extends toward increasing Y from {@code
     *     outerEdgeY} (i.e. {@code outerEdgeY} is near {@code Y=0}), {@code -1} if it extends toward
     *     decreasing Y (i.e. {@code outerEdgeY} is near the far long edge)
     */
    private static void addTrenchLegs(FieldMap map, double centerX, double outerEdgeY, double towardCenterSign) {
        double innerEdgeY = outerEdgeY + towardCenterSign * Rebuilt2026.TRENCH_WIDTH_METERS;
        double halfLeg = Rebuilt2026.TRENCH_LEG_WIDTH_METERS / 2.0;
        // Each leg's OUTWARD face sits on its respective trench edge; its center is half a leg-width
        // in from that edge.
        double outerLegCenterY = outerEdgeY + towardCenterSign * halfLeg;
        double innerLegCenterY = innerEdgeY - towardCenterSign * halfLeg;
        for (double legCenterY : new double[] {outerLegCenterY, innerLegCenterY}) {
            map.addObstacle(
                    Geometry.createRectangle(Rebuilt2026.TRENCH_DEPTH_METERS, Rebuilt2026.TRENCH_LEG_WIDTH_METERS),
                    new Pose2d(centerX, legCenterY, Rotation2d.kZero),
                    SimConstants.CARPET_FRICTION_COEFFICIENT);
        }
    }

    /**
     * All four REBUILT BUMPs (official 1.854 x 1.128 x 0.165 m each), flanking both HUBs. Attach with
     * {@code drivetrain.setTerrain(Arena2026Rebuilt.rebuiltBumps())} so the robot tilts and slows
     * crossing whichever one it's on.
     */
    public static CompositeTerrain rebuiltBumps() {
        double offset = Rebuilt2026.BUMP_CENTER_Y_OFFSET_METERS;
        double blueBumpX = Rebuilt2026.BUMP_DISTANCE_FROM_WALL_METERS;
        double redBumpX = Rebuilt2026.FIELD_LENGTH_METERS - Rebuilt2026.BUMP_DISTANCE_FROM_WALL_METERS;
        return new CompositeTerrain(List.of(
                bumpAt(blueBumpX, Rebuilt2026.FIELD_CENTER_Y - offset),
                bumpAt(blueBumpX, Rebuilt2026.FIELD_CENTER_Y + offset),
                bumpAt(redBumpX, Rebuilt2026.FIELD_CENTER_Y - offset),
                bumpAt(redBumpX, Rebuilt2026.FIELD_CENTER_Y + offset)));
    }

    private static BumpRegion bumpAt(double centerX, double centerY) {
        return new BumpRegion(
                centerX - Rebuilt2026.BUMP_DEPTH_METERS / 2.0,
                centerX + Rebuilt2026.BUMP_DEPTH_METERS / 2.0,
                centerY - Rebuilt2026.BUMP_WIDTH_METERS / 2.0,
                centerY + Rebuilt2026.BUMP_WIDTH_METERS / 2.0,
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
