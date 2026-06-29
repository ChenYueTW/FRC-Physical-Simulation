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

        /** FUEL mass (kg) — TODO: confirm against official AndyMark FUEL specification. */
        public static final double FUEL_MASS_KG = 0.07;

        /** FUEL coefficient of restitution (foam -> fairly inelastic) — TODO: tune empirically. */
        public static final double FUEL_RESTITUTION = 0.35;

        /** Geometric center of the FIELD. */
        public static final Translation2d FIELD_CENTER =
                new Translation2d(FIELD_LENGTH_METERS / 2.0, FIELD_WIDTH_METERS / 2.0);
    }
}
