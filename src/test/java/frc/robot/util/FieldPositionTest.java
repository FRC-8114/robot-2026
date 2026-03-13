package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertAll;
import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.FieldConstants;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

class FieldPositionTest {
    private static final double EPSILON = 1e-3;

    @BeforeAll
    static void initializeHal() {
        HAL.initialize(500, 0);
    }

    @AfterEach
    void resetDriverStation() {
        DriverStationSim.resetData();
        DriverStationSim.notifyNewData();
    }

    @Test
    void allianceRelativeZoneThresholdsAreInclusive() {
        assertAll(
                () -> assertEquals(
                        FieldPosition.Zone.ALLIANCE_ZONE,
                        FieldPosition.getAllianceRelativeZone(FieldConstants.LinesVertical.allianceZone)),
                () -> assertEquals(
                        FieldPosition.Zone.NEUTRAL_ZONE,
                        FieldPosition.getAllianceRelativeZone(FieldConstants.LinesVertical.allianceZone + EPSILON)),
                () -> assertEquals(
                        FieldPosition.Zone.NEUTRAL_ZONE,
                        FieldPosition.getAllianceRelativeZone(FieldConstants.LinesVertical.oppAllianceZone - EPSILON)),
                () -> assertEquals(
                        FieldPosition.Zone.OPPONENT_ZONE,
                        FieldPosition.getAllianceRelativeZone(FieldConstants.LinesVertical.oppAllianceZone)));
    }

    @Test
    void blueAllianceFieldCoordinatesUseBluePerspective() {
        setAlliance(AllianceStationID.Blue1);

        double allianceX = FieldConstants.LinesVertical.allianceZone - EPSILON;
        double neutralX = (FieldConstants.LinesVertical.neutralZoneNear + FieldConstants.LinesVertical.neutralZoneFar) / 2.0;
        double opponentX = FieldConstants.LinesVertical.oppAllianceZone + EPSILON;

        assertAll(
                () -> assertEquals(
                        FieldPosition.Zone.ALLIANCE_ZONE,
                        FieldPosition.getZone(new Translation2d(allianceX, 0.0))),
                () -> assertEquals(
                        FieldPosition.Zone.NEUTRAL_ZONE,
                        FieldPosition.getZone(new Translation3d(neutralX, 0.0, 0.0))),
                () -> assertEquals(
                        FieldPosition.Zone.OPPONENT_ZONE,
                        FieldPosition.getZone(new Pose2d(opponentX, 0.0, Rotation2d.kZero))));
    }

    @Test
    void redAllianceFieldCoordinatesFlipToAllianceRelativePerspective() {
        setAlliance(AllianceStationID.Red1);

        double allianceX = FieldConstants.fieldLength - (FieldConstants.LinesVertical.allianceZone - EPSILON);
        double neutralX = FieldConstants.fieldLength
                - ((FieldConstants.LinesVertical.neutralZoneNear + FieldConstants.LinesVertical.neutralZoneFar) / 2.0);
        double opponentX = FieldConstants.fieldLength - (FieldConstants.LinesVertical.oppAllianceZone + EPSILON);

        assertAll(
                () -> assertEquals(
                        FieldPosition.Zone.ALLIANCE_ZONE,
                        FieldPosition.getZone(new Translation2d(allianceX, 0.0))),
                () -> assertEquals(
                        FieldPosition.Zone.NEUTRAL_ZONE,
                        FieldPosition.getZone(new Translation3d(neutralX, 0.0, 0.0))),
                () -> assertEquals(
                        FieldPosition.Zone.OPPONENT_ZONE,
                        FieldPosition.getZone(new Pose2d(opponentX, 0.0, Rotation2d.kZero))));
    }

    private static void setAlliance(AllianceStationID allianceStationID) {
        DriverStationSim.setAllianceStationId(allianceStationID);
        DriverStationSim.notifyNewData();
    }
}
