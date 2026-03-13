package frc.robot.supersystems;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static org.junit.jupiter.api.Assertions.assertAll;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.FieldConstants;
import frc.robot.subsystems.turret.Turret;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

class ShotSolutionCalculatorTest {
    private static final double EPSILON = 1e-9;

    private static final ShotSolutionCalculator.TargetLayout SYNTHETIC_TARGET_LAYOUT =
            new ShotSolutionCalculator.TargetLayout(
                    new Translation3d(4.0, 6.0, 2.0),
                    new Translation3d(8.0, 6.5, 1.5),
                    new Translation3d(8.0, 2.5, 1.5));

    private static final ShotSolutionCalculator.TargetLayout CROSS_FIELD_TARGET_LAYOUT =
            new ShotSolutionCalculator.TargetLayout(
                    new Translation3d(4.0, 6.0, 2.0),
                    new Translation3d(12.0, 6.5, 1.5),
                    new Translation3d(2.0, 4.7, 1.5));

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
    void solveUsesHubTargetInsideAllianceZone() {
        setAlliance(AllianceStationID.Blue1);

        Translation3d turretPosition = new Translation3d(
                FieldConstants.LinesVertical.allianceZone - 0.5,
                FieldConstants.fieldWidth / 2.0 - 1.2,
                1.0);
        Rotation2d heading = Rotation2d.kZero;
        Translation2d fieldVelocity = new Translation2d();
        ShotSolutionCalculator.ShotParameters shotParameters =
                new ShotSolutionCalculator.ShotParameters(2100.0, 0.6);
        AtomicInteger lookupCalls = new AtomicInteger();
        AtomicReference<Double> lookupDistance = new AtomicReference<>();

        ShotSolutionCalculator.ShotSolution solution = ShotSolutionCalculator.solve(
                new ShotSolutionCalculator.ShotInput(turretPosition, heading, fieldVelocity),
                distanceMeters -> {
                    lookupCalls.incrementAndGet();
                    lookupDistance.set(distanceMeters);
                    return shotParameters;
                });

        Translation3d expectedTarget = FieldConstants.Hub.innerCenterPoint;
        double expectedDistance = turretPosition.toTranslation2d().getDistance(expectedTarget.toTranslation2d());
        double expectedTimeOfFlight = calculateTimeOfFlight(expectedDistance, shotParameters);
        double expectedYawRadians = calculateExpectedYawRadians(
                turretPosition.toTranslation2d(),
                expectedTarget.toTranslation2d(),
                fieldVelocity,
                heading,
                expectedTimeOfFlight);

        assertAll(
                () -> assertEquals(1, lookupCalls.get()),
                () -> assertFalse(solution.passingShotActive()),
                () -> assertTranslationEquals(expectedTarget, solution.target()),
                () -> assertEquals(expectedDistance, lookupDistance.get(), EPSILON),
                () -> assertEquals(expectedDistance, solution.distanceMeters(), EPSILON),
                () -> assertEquals(shotParameters.rpm(), solution.rpm().in(RPM), EPSILON),
                () -> assertEquals(shotParameters.pitchRadians(), solution.pitch().in(Radians), EPSILON),
                () -> assertEquals(expectedTimeOfFlight, solution.timeOfFlightSeconds(), EPSILON),
                () -> assertEquals(expectedYawRadians, solution.yaw().in(Radians), EPSILON));
    }

    @Test
    void solveUsesNearestPassingTargetOutsideAllianceZone() {
        setAlliance(AllianceStationID.Blue1);

        Translation3d turretPosition = new Translation3d(
                FieldConstants.LinesVertical.neutralZoneNear + 0.2,
                0.5,
                1.0);
        Rotation2d heading = Rotation2d.kZero;
        Translation2d fieldVelocity = new Translation2d();
        ShotSolutionCalculator.ShotParameters shotParameters =
                new ShotSolutionCalculator.ShotParameters(1850.0, 0.55);

        ShotSolutionCalculator.ShotSolution solution = ShotSolutionCalculator.solve(
                new ShotSolutionCalculator.ShotInput(turretPosition, heading, fieldVelocity),
                distanceMeters -> shotParameters,
                SYNTHETIC_TARGET_LAYOUT);

        Translation3d expectedTarget = SYNTHETIC_TARGET_LAYOUT.rightPassingTarget();
        double expectedDistance = turretPosition.toTranslation2d().getDistance(expectedTarget.toTranslation2d());
        double expectedTimeOfFlight = calculateTimeOfFlight(expectedDistance, shotParameters);
        double expectedYawRadians = calculateExpectedYawRadians(
                turretPosition.toTranslation2d(),
                expectedTarget.toTranslation2d(),
                fieldVelocity,
                heading,
                expectedTimeOfFlight);

        assertAll(
                () -> assertTrue(solution.passingShotActive()),
                () -> assertTranslationEquals(expectedTarget, solution.target()),
                () -> assertEquals(expectedDistance, solution.distanceMeters(), EPSILON),
                () -> assertEquals(expectedTimeOfFlight, solution.timeOfFlightSeconds(), EPSILON),
                () -> assertEquals(expectedYawRadians, solution.yaw().in(Radians), EPSILON));
    }

    @Test
    void solveUsesNearestAllianceBumpByDistanceForCrossFieldShot() {
        setAlliance(AllianceStationID.Blue1);

        Translation3d turretPosition = new Translation3d(
                FieldConstants.LinesVertical.oppAllianceZone + 0.2,
                4.8,
                1.0);
        Rotation2d heading = Rotation2d.kZero;
        Translation2d fieldVelocity = new Translation2d();
        ShotSolutionCalculator.ShotParameters shotParameters =
                new ShotSolutionCalculator.ShotParameters(1950.0, 0.52);

        ShotSolutionCalculator.ShotSolution solution = ShotSolutionCalculator.solve(
                new ShotSolutionCalculator.ShotInput(turretPosition, heading, fieldVelocity),
                distanceMeters -> shotParameters,
                CROSS_FIELD_TARGET_LAYOUT);

        Translation3d expectedTarget = CROSS_FIELD_TARGET_LAYOUT.leftPassingTarget();
        double expectedDistance = turretPosition.toTranslation2d().getDistance(expectedTarget.toTranslation2d());
        double expectedTimeOfFlight = calculateTimeOfFlight(expectedDistance, shotParameters);
        double expectedYawRadians = calculateExpectedYawRadians(
                turretPosition.toTranslation2d(),
                expectedTarget.toTranslation2d(),
                fieldVelocity,
                heading,
                expectedTimeOfFlight);

        assertAll(
                () -> assertTrue(solution.passingShotActive()),
                () -> assertTranslationEquals(expectedTarget, solution.target()),
                () -> assertEquals(expectedDistance, solution.distanceMeters(), EPSILON),
                () -> assertEquals(expectedYawRadians, solution.yaw().in(Radians), EPSILON));
    }

    @Test
    void solveFlipsAllianceRelativePassingTargetsForRedAlliance() {
        setAlliance(AllianceStationID.Red1);

        Translation3d allianceRelativeTurretPosition = new Translation3d(
                FieldConstants.LinesVertical.neutralZoneNear + 0.2,
                8.0,
                1.0);
        Translation3d turretPosition = new Translation3d(
                FieldConstants.fieldLength - allianceRelativeTurretPosition.getX(),
                allianceRelativeTurretPosition.getY(),
                allianceRelativeTurretPosition.getZ());
        Rotation2d heading = Rotation2d.fromRadians(Math.PI);
        Translation2d fieldVelocity = new Translation2d();
        ShotSolutionCalculator.ShotParameters shotParameters =
                new ShotSolutionCalculator.ShotParameters(1900.0, 0.5);

        ShotSolutionCalculator.ShotSolution solution = ShotSolutionCalculator.solve(
                new ShotSolutionCalculator.ShotInput(turretPosition, heading, fieldVelocity),
                distanceMeters -> shotParameters,
                SYNTHETIC_TARGET_LAYOUT);

        Translation3d expectedTarget = new Translation3d(
                FieldConstants.fieldLength - SYNTHETIC_TARGET_LAYOUT.leftPassingTarget().getX(),
                SYNTHETIC_TARGET_LAYOUT.leftPassingTarget().getY(),
                SYNTHETIC_TARGET_LAYOUT.leftPassingTarget().getZ());
        double expectedDistance = turretPosition.toTranslation2d().getDistance(expectedTarget.toTranslation2d());
        double expectedTimeOfFlight = calculateTimeOfFlight(expectedDistance, shotParameters);
        double expectedYawRadians = calculateExpectedYawRadians(
                turretPosition.toTranslation2d(),
                expectedTarget.toTranslation2d(),
                fieldVelocity,
                heading,
                expectedTimeOfFlight);

        assertAll(
                () -> assertTrue(solution.passingShotActive()),
                () -> assertTranslationEquals(expectedTarget, solution.target()),
                () -> assertEquals(expectedDistance, solution.distanceMeters(), EPSILON),
                () -> assertEquals(expectedYawRadians, solution.yaw().in(Radians), EPSILON));
    }

    @Test
    void solveAppliesLeadUsingTheUnledDistanceSnapshot() {
        setAlliance(AllianceStationID.Blue1);

        Translation3d turretPosition = new Translation3d(
                FieldConstants.LinesVertical.allianceZone - 0.2,
                4.5,
                1.0);
        Rotation2d heading = Rotation2d.kZero;
        Translation2d fieldVelocity = new Translation2d(0.0, 1.5);
        ShotSolutionCalculator.ShotParameters shotParameters =
                new ShotSolutionCalculator.ShotParameters(1200.0, 0.5);
        AtomicReference<Double> lookupDistance = new AtomicReference<>();

        ShotSolutionCalculator.ShotSolution solution = ShotSolutionCalculator.solve(
                new ShotSolutionCalculator.ShotInput(turretPosition, heading, fieldVelocity),
                distanceMeters -> {
                    lookupDistance.set(distanceMeters);
                    return shotParameters;
                },
                SYNTHETIC_TARGET_LAYOUT);

        Translation3d expectedTarget = SYNTHETIC_TARGET_LAYOUT.hubTarget();
        double expectedDistance = turretPosition.toTranslation2d().getDistance(expectedTarget.toTranslation2d());
        double expectedTimeOfFlight = calculateTimeOfFlight(expectedDistance, shotParameters);
        Translation2d expectedFieldRelativeTargetVector = expectedTarget.toTranslation2d()
                .minus(fieldVelocity.times(expectedTimeOfFlight))
                .minus(turretPosition.toTranslation2d());
        double expectedYawRadians = calculateExpectedYawRadians(
                turretPosition.toTranslation2d(),
                expectedTarget.toTranslation2d(),
                fieldVelocity,
                heading,
                expectedTimeOfFlight);

        assertAll(
                () -> assertEquals(expectedDistance, lookupDistance.get(), EPSILON),
                () -> assertEquals(expectedDistance, solution.distanceMeters(), EPSILON),
                () -> assertTranslationEquals(expectedTarget, solution.target()),
                () -> assertTranslationEquals(expectedFieldRelativeTargetVector, solution.fieldRelativeTargetVector()),
                () -> assertEquals(expectedTimeOfFlight, solution.timeOfFlightSeconds(), EPSILON),
                () -> assertEquals(expectedYawRadians, solution.yaw().in(Radians), EPSILON));
    }

    private static double calculateTimeOfFlight(
            double distanceMeters,
            ShotSolutionCalculator.ShotParameters shotParameters) {
        double exitVelocityMetersPerSecond =
                (shotParameters.rpm() * 2.0 * Math.PI * ShooterSupersystem.Constants.FLYWHEEL_RADIUS_METERS / 60.0)
                        * ShooterSupersystem.Constants.SPIN_TRANSFER_EFFICIENCY;
        return distanceMeters / (exitVelocityMetersPerSecond * Math.cos(shotParameters.pitchRadians()));
    }

    private static double calculateExpectedYawRadians(
            Translation2d turretPosition,
            Translation2d targetPosition,
            Translation2d fieldVelocity,
            Rotation2d heading,
            double timeOfFlight) {
        Translation2d fieldRelativeTargetVector = targetPosition
                .minus(fieldVelocity.times(timeOfFlight))
                .minus(turretPosition);
        if (fieldRelativeTargetVector.getNorm() < EPSILON) {
            return 0.0;
        }

        Translation2d robotRelativeTargetVector = fieldRelativeTargetVector.rotateBy(heading.unaryMinus());
        return Turret.clampAngle(
                Turret.normalizeAngle(Radians.of(robotRelativeTargetVector.getAngle().getRadians()))).in(Radians);
    }

    private static void assertTranslationEquals(Translation3d expected, Translation3d actual) {
        assertAll(
                () -> assertEquals(expected.getX(), actual.getX(), EPSILON),
                () -> assertEquals(expected.getY(), actual.getY(), EPSILON),
                () -> assertEquals(expected.getZ(), actual.getZ(), EPSILON));
    }

    private static void assertTranslationEquals(Translation2d expected, Translation2d actual) {
        assertAll(
                () -> assertEquals(expected.getX(), actual.getX(), EPSILON),
                () -> assertEquals(expected.getY(), actual.getY(), EPSILON));
    }

    private static void setAlliance(AllianceStationID allianceStationID) {
        DriverStationSim.setAllianceStationId(allianceStationID);
        DriverStationSim.notifyNewData();
    }
}
