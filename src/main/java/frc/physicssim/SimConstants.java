package frc.physicssim;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * Central collection of physical constants for the simulation library.
 *
 * <p>All units are SI (meters, kilograms, seconds, radians) to match WPILib and dyn4j conventions.
 * Season-specific geometry for 2026 <b>REBUILT</b> lives in {@link SimConstants.Rebuilt2026}; values
 * marked {@code TODO} are reasonable approximations pending confirmation against the official Game
 * Manual / AndyMark drawings.
 */
public final class SimConstants {
    private SimConstants() {}

    /** Standard gravitational acceleration (m/s^2). */
    public static final double GRAVITY = 9.80665;

    /** Default robot loop period (s) — matches WPILib's {@code TimedRobot} 50 Hz cycle. */
    public static final double DEFAULT_PERIOD_SECONDS = 0.02;

    /** Default number of physics sub-ticks per robot period (5 -> 250 Hz physics). */
    public static final int DEFAULT_SUB_TICKS = 5;

    /** Approximate coefficient of friction between game pieces / robots and the FRC carpet. */
    public static final double CARPET_FRICTION_COEFFICIENT = 0.8;

    /** 2026 REBUILT season geometry and game-piece specifications. */
    public static final class Rebuilt2026 {
        private Rebuilt2026() {}

        /** FIELD length along the long (X) axis: 651.2 in. */
        public static final double FIELD_LENGTH_METERS = 16.5405;

        /** FIELD width along the short (Y) axis: 317.7 in. */
        public static final double FIELD_WIDTH_METERS = 8.0696;

        /** FUEL diameter: 5.91 in high-density foam ball. */
        public static final double FUEL_DIAMETER_METERS = 0.15011;

        /** FUEL radius (m). */
        public static final double FUEL_RADIUS_METERS = FUEL_DIAMETER_METERS / 2.0;

        /** FUEL mass (kg): official FUEL weighs 0.448–0.5 lb; ~0.215 kg. */
        public static final double FUEL_MASS_KG = 0.215;

        /** FUEL coefficient of restitution (foam -> fairly inelastic) — TODO: tune empirically. */
        public static final double FUEL_RESTITUTION = 0.35;

        /** Geometric center of the FIELD. */
        public static final Translation2d FIELD_CENTER =
                new Translation2d(FIELD_LENGTH_METERS / 2.0, FIELD_WIDTH_METERS / 2.0);

        /** Field Y coordinate of the center line. */
        public static final double FIELD_CENTER_Y = FIELD_WIDTH_METERS / 2.0;

        // ---- HUB: one 47 in square per alliance, centered on the HUB/BUMP/TRENCH axis ----
        /** HUB footprint side length: 47 in. */
        public static final double HUB_SIZE_METERS = 1.1938;

        /**
         * Distance from an alliance wall to the center of that alliance's HUB: 182.11 in. Derived
         * from the official field-dimension drawings (FE-2026, sheet 3 of 11, reference dimension
         * "182.11" from the diamond plate) and cross-checked against the AprilTag coordinate table
         * (averaged HUB tag X-span, both alliances agree to within 0.1 in).
         */
        public static final double HUB_DISTANCE_FROM_WALL_METERS = 4.62559;

        /** Center of the blue-alliance HUB. */
        public static final Translation2d BLUE_HUB_CENTER =
                new Translation2d(HUB_DISTANCE_FROM_WALL_METERS, FIELD_CENTER_Y);

        /** Center of the red-alliance HUB. */
        public static final Translation2d RED_HUB_CENTER =
                new Translation2d(FIELD_LENGTH_METERS - HUB_DISTANCE_FROM_WALL_METERS, FIELD_CENTER_Y);

        /** Approximate height of the HUB opening used as a shooting target — TODO: confirm. */
        public static final double HUB_TARGET_HEIGHT_METERS = 1.1;

        // ---- BUMP: 73 in wide (Y) x 44.4 in deep (X) x 6.513 in tall; 2 per alliance, flanking the
        // HUB in Y on the same X-axis as the HUB/TRENCH row ----
        public static final double BUMP_WIDTH_METERS = 1.8542;
        public static final double BUMP_DEPTH_METERS = 1.1278;
        public static final double BUMP_HEIGHT_METERS = 0.1654;

        /**
         * Offset from the field's Y centerline to each BUMP's center: 90.95 in (field-dimension
         * drawings, sheet 2 of 11).
         */
        public static final double BUMP_CENTER_Y_OFFSET_METERS = 2.31013;

        /**
         * Distance from the alliance wall to each BUMP's center. TODO: not independently confirmed —
         * the drawings give the BUMP's depth (44.4 in) and its Y-offset above, but its X placement
         * wasn't cleanly extractable from the field-dimension drawings. The BUMP ramps sit on the
         * alliance-zone/neutral-zone boundary per the official field tour description, which is closer
         * to the wall than the HUB/TRENCH row; this places it about halfway there as a placeholder that
         * at least doesn't spatially overlap the TRENCH.
         */
        public static final double BUMP_DISTANCE_FROM_WALL_METERS = HUB_DISTANCE_FROM_WALL_METERS / 2.0;

        // ---- TRENCH: 65.65 in wide (Y) x 47.0 in deep (X) x 40.25 in tall archway robots drive
        // under; the clear passage underneath is 50.34 in wide x 22.25 in tall, so only the two edge
        // support legs (the outer ~7.66 in on each side) are solid floor-plan obstacles. 2 per
        // alliance, outboard of the BUMPs (closer to the field's long edges), same X as the HUB. ----
        public static final double TRENCH_WIDTH_METERS = 1.66751;
        public static final double TRENCH_DEPTH_METERS = 1.1938;
        public static final double TRENCH_LEG_WIDTH_METERS = 0.19443;
        public static final double TRENCH_PASSAGE_WIDTH_METERS = 1.27864;

        /**
         * Field Y of the edge of each TRENCH closest to the field's own long edge (not its center):
         * 25.37 in for the trench near Y=0, and 292.31 in for the trench near Y={@value
         * #FIELD_WIDTH_METERS} (AprilTag coordinate table; cross-checked against the "133.47 in from
         * centerline" reference dimension on sheet 3 of 11). Treating these as the trench's center
         * would place a support leg off the field, so they're taken as the near edge instead; the
         * trench's {@value #TRENCH_WIDTH_METERS} m width extends from there toward the field center.
         */
        public static final double TRENCH_NEAR_FIELD_EDGE_OUTER_Y_METERS = 0.64440;

        public static final double TRENCH_FAR_FIELD_EDGE_OUTER_Y_METERS = 7.42667;

        // ---- DEPOT: 42 in wide (along wall, Y) x 27 in deep (into field, X), 24 FUEL each ----
        public static final double DEPOT_WIDTH_METERS = 1.0668;
        public static final double DEPOT_DEPTH_METERS = 0.6858;
        public static final int DEPOT_FUEL_COUNT = 24;

        /**
         * Field Y of each DEPOT's center along its alliance wall — TODO: confirm against the field
         * drawings (the manual gives the DEPOT size but not its exact position along the wall).
         */
        public static final double DEPOT_CENTER_Y = 1.5;

        // ---- NEUTRAL ZONE: FUEL corralled in a 206 in (Y) x 72 in (X) box at field center ----
        public static final double NEUTRAL_ZONE_WIDTH_METERS = 5.2324;
        public static final double NEUTRAL_ZONE_DEPTH_METERS = 1.8288;

        /**
         * FUEL staged in the NEUTRAL ZONE at the start of a match (the manual gives 360–408 depending
         * on how many are preloaded; 408 assumes none preloaded).
         */
        public static final int NEUTRAL_ZONE_FUEL_COUNT = 408;
    }
}
