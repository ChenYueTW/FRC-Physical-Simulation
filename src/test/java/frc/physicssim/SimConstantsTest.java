package frc.physicssim;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

/** Sanity checks for the season constants and that the WPILib/JUnit toolchain is wired up. */
class SimConstantsTest {
    private static final double EPS = 1e-9;

    @Test
    void fuelRadiusIsHalfDiameter() {
        assertEquals(
                SimConstants.Rebuilt2026.FUEL_DIAMETER_METERS / 2.0, SimConstants.Rebuilt2026.FUEL_RADIUS_METERS, EPS);
    }

    @Test
    void fieldCenterIsHalfOfFieldDimensions() {
        assertEquals(
                SimConstants.Rebuilt2026.FIELD_LENGTH_METERS / 2.0, SimConstants.Rebuilt2026.FIELD_CENTER.getX(), EPS);
        assertEquals(
                SimConstants.Rebuilt2026.FIELD_WIDTH_METERS / 2.0, SimConstants.Rebuilt2026.FIELD_CENTER.getY(), EPS);
    }

    @Test
    void fieldIsLongerThanItIsWide() {
        assertTrue(SimConstants.Rebuilt2026.FIELD_LENGTH_METERS > SimConstants.Rebuilt2026.FIELD_WIDTH_METERS);
    }
}
