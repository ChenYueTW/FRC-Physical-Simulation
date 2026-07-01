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

        // ---- HUB: one 47 in square per alliance, 158.6 in from that alliance's wall, centered ----
        /** HUB footprint side length: 47 in. */
        public static final double HUB_SIZE_METERS = 1.1938;

        /** Distance from an alliance wall to the center of that alliance's HUB: 158.6 in. */
        public static final double HUB_DISTANCE_FROM_WALL_METERS = 4.0284;

        /** Center of the blue-alliance HUB. */
        public static final Translation2d BLUE_HUB_CENTER =
                new Translation2d(HUB_DISTANCE_FROM_WALL_METERS, FIELD_CENTER_Y);

        /** Center of the red-alliance HUB. */
        public static final Translation2d RED_HUB_CENTER =
                new Translation2d(FIELD_LENGTH_METERS - HUB_DISTANCE_FROM_WALL_METERS, FIELD_CENTER_Y);

        /** Approximate height of the HUB opening used as a shooting target — TODO: confirm. */
        public static final double HUB_TARGET_HEIGHT_METERS = 1.1;

        // ---- BUMP: 73 in wide (Y) x 44.4 in deep (X) x 6.513 in tall ----
        public static final double BUMP_WIDTH_METERS = 1.8542;
        public static final double BUMP_DEPTH_METERS = 1.1278;
        public static final double BUMP_HEIGHT_METERS = 0.1654;

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
