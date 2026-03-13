package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertAll;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class HubShiftUtilTest {
    private static final double EPSILON = 1e-9;

    @BeforeAll
    static void initializeHal() {
        HAL.initialize(500, 0);
    }

    @BeforeEach
    void setUp() {
        DriverStationSim.resetData();
        DriverStationSim.notifyNewData();
        SimHooks.pauseTiming();
        SimHooks.restartTiming();
        HubShiftUtil.initialize();
    }

    @AfterEach
    void tearDown() {
        DriverStationSim.resetData();
        DriverStationSim.notifyNewData();
        SimHooks.restartTiming();
    }

    @Test
    void emptyOrInvalidGameSpecificMessageDefaultsToCurrentAlliance() {
        setAlliance(AllianceStationID.Blue1);

        DriverStationSim.setGameSpecificMessage("");
        DriverStationSim.notifyNewData();
        assertEquals(Alliance.Blue, HubShiftUtil.getFirstActiveAlliance());

        DriverStationSim.setGameSpecificMessage("?");
        DriverStationSim.notifyNewData();
        assertEquals(Alliance.Blue, HubShiftUtil.getFirstActiveAlliance());
    }

    @Test
    void validGameSpecificMessageUsesOppositeAllianceAsFirstActive() {
        setAlliance(AllianceStationID.Red1);

        DriverStationSim.setGameSpecificMessage("B");
        DriverStationSim.notifyNewData();
        assertEquals(Alliance.Red, HubShiftUtil.getFirstActiveAlliance());

        DriverStationSim.setGameSpecificMessage("R");
        DriverStationSim.notifyNewData();
        assertEquals(Alliance.Blue, HubShiftUtil.getFirstActiveAlliance());
    }

    @Test
    void officialShiftInfoResyncsToFieldMatchTimeWhenFmsIsAttached() {
        setAlliance(AllianceStationID.Blue1);
        DriverStationSim.setGameSpecificMessage("R");
        DriverStationSim.setEnabled(true);
        DriverStationSim.setAutonomous(false);
        DriverStationSim.setFmsAttached(true);
        DriverStationSim.notifyNewData();

        SimHooks.stepTiming(40.0);

        // 140 - 115 = 25 seconds into teleop, which should still be SHIFT1 and active.
        DriverStationSim.setMatchTime(115.0);
        DriverStationSim.notifyNewData();

        HubShiftUtil.ShiftInfo shiftInfo = HubShiftUtil.getOfficialShiftInfo();

        assertAll(
                () -> assertEquals(HubShiftUtil.ShiftEnum.SHIFT1, shiftInfo.currentShift()),
                () -> assertTrue(shiftInfo.active()),
                () -> assertEquals(25.0, shiftInfo.elapsedTime(), EPSILON),
                () -> assertEquals(10.0, shiftInfo.remainingTime(), EPSILON));
    }

    @Test
    void allianceFlipWarningTurnsOnFiveSecondsBeforeNextFlip() {
        setAlliance(AllianceStationID.Blue1);
        DriverStationSim.setGameSpecificMessage("R");
        DriverStationSim.setEnabled(true);
        DriverStationSim.setAutonomous(false);
        DriverStationSim.notifyNewData();

        SimHooks.stepTiming(29.0);
        assertEquals(false, HubShiftUtil.isAllianceFlipImminent(5.0));

        SimHooks.stepTiming(1.0);
        assertTrue(HubShiftUtil.isAllianceFlipImminent(5.0));
    }

    private static void setAlliance(AllianceStationID allianceStationID) {
        DriverStationSim.setAllianceStationId(allianceStationID);
        DriverStationSim.notifyNewData();
    }
}
