package frc.robot.supersystems;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.FieldConstants;
import frc.robot.subsystems.turret.Turret;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.FieldPosition;

final class ShotSolutionCalculator {
    @FunctionalInterface
    interface ShotParametersLookup {
        ShotParameters get(double distanceMeters);
    }

    record ShotParameters(double rpm, double pitchRadians) {
    }

    record ShotInput(Translation3d turretPosition, Rotation2d robotHeading, Translation2d fieldVelocity) {
    }

    record ShotSolution(
            Translation3d turretPosition,
            Translation3d target,
            Translation2d fieldRelativeTargetVector,
            Translation2d robotRelativeTargetVector,
            Angle yaw,
            Angle pitch,
            AngularVelocity rpm,
            double distanceMeters,
            double exitVelocityMetersPerSecond,
            double timeOfFlightSeconds,
            boolean passingShotActive) {
    }

    record TargetLayout(Translation3d hubTarget, Translation3d leftPassingTarget, Translation3d rightPassingTarget) {
    }

    record TargetSelection(Translation3d target, boolean passingShotActive) {
    }

    private static final TargetLayout FIELD_TARGET_LAYOUT = new TargetLayout(
            FieldConstants.Hub.innerCenterPoint,
            getPassingTargetForBump(
                    FieldConstants.LeftBump.nearLeftCorner,
                    FieldConstants.LeftBump.nearRightCorner,
                    FieldConstants.LeftBump.height),
            getPassingTargetForBump(
                    FieldConstants.RightBump.nearLeftCorner,
                    FieldConstants.RightBump.nearRightCorner,
                    FieldConstants.RightBump.height));

    private ShotSolutionCalculator() {
    }

    static ShotSolution solve(ShotInput shotInput, ShotParametersLookup shotParametersLookup) {
        return solve(shotInput, shotParametersLookup, FIELD_TARGET_LAYOUT);
    }

    static ShotSolution solve(
            ShotInput shotInput,
            ShotParametersLookup shotParametersLookup,
            TargetLayout targetLayout) {
        TargetSelection targetSelection = getTargetSelection(shotInput.turretPosition(), targetLayout);
        Translation3d target = targetSelection.target();
        double distanceMeters =
                target.toTranslation2d().getDistance(shotInput.turretPosition().toTranslation2d());
        ShotParameters shotParameters = shotParametersLookup.get(distanceMeters);
        double exitVelocityMetersPerSecond = getExitVelocityMetersPerSecond(shotParameters);
        double timeOfFlightSeconds =
                distanceMeters / (exitVelocityMetersPerSecond * Math.cos(shotParameters.pitchRadians()));

        Translation2d turretPosition2d = shotInput.turretPosition().toTranslation2d();
        Translation2d targetPosition2d = target.toTranslation2d();
        Translation2d fieldRelativeTargetVector = getFieldRelativeLeadTargetVector(
                turretPosition2d,
                targetPosition2d,
                shotInput.fieldVelocity(),
                timeOfFlightSeconds);
        Translation2d robotRelativeTargetVector =
                getRobotRelativeTargetVector(fieldRelativeTargetVector, shotInput.robotHeading());
        Angle yaw = Turret.clampAngle(getRobotRelativeYaw(fieldRelativeTargetVector, shotInput.robotHeading()));

        return new ShotSolution(
                shotInput.turretPosition(),
                target,
                fieldRelativeTargetVector,
                robotRelativeTargetVector,
                yaw,
                Radians.of(shotParameters.pitchRadians()),
                RPM.of(shotParameters.rpm()),
                distanceMeters,
                exitVelocityMetersPerSecond,
                timeOfFlightSeconds,
                targetSelection.passingShotActive());
    }

    static Translation2d getFieldRelativeLeadTargetVector(
            Translation2d turretPosition,
            Translation2d targetPosition,
            Translation2d fieldVelocity,
            double timeOfFlight) {
        Translation2d offsetTarget = targetPosition.minus(fieldVelocity.times(timeOfFlight));
        return offsetTarget.minus(turretPosition);
    }

    static Translation2d getRobotRelativeTargetVector(
            Translation2d fieldRelativeTargetVector,
            Rotation2d robotHeading) {
        return fieldRelativeTargetVector.rotateBy(robotHeading.unaryMinus());
    }

    static Angle getRobotRelativeYaw(Translation2d fieldRelativeTargetVector, Rotation2d robotHeading) {
        if (fieldRelativeTargetVector.getNorm() < 1e-9) {
            return Radians.zero();
        }

        Translation2d robotRelativeTargetVector = getRobotRelativeTargetVector(fieldRelativeTargetVector, robotHeading);
        return Turret.normalizeAngle(Radians.of(robotRelativeTargetVector.getAngle().getRadians()));
    }

    private static double getExitVelocityMetersPerSecond(ShotParameters shotParameters) {
        double rpm = shotParameters.rpm();
        return (rpm * 2.0 * Math.PI * ShooterSupersystem.Constants.FLYWHEEL_RADIUS_METERS / 60.0)
                * ShooterSupersystem.Constants.SPIN_TRANSFER_EFFICIENCY;
    }

    static TargetSelection getTargetSelection(Translation3d turretPosition) {
        return getTargetSelection(turretPosition, FIELD_TARGET_LAYOUT);
    }

    private static TargetSelection getTargetSelection(Translation3d turretPosition, TargetLayout targetLayout) {
        Translation3d allianceRelativeTurretPosition = AllianceFlipUtil.apply(turretPosition);
        boolean passingShotActive =
                FieldPosition.getAllianceRelativeZone(allianceRelativeTurretPosition) != FieldPosition.Zone.ALLIANCE_ZONE;

        Translation3d allianceRelativeTarget = passingShotActive
                ? getNearestPassingTarget(allianceRelativeTurretPosition, targetLayout)
                : targetLayout.hubTarget();
        return new TargetSelection(AllianceFlipUtil.apply(allianceRelativeTarget), passingShotActive);
    }

    private static Translation3d getNearestPassingTarget(
            Translation3d allianceRelativeTurretPosition,
            TargetLayout targetLayout) {
        Translation2d turretPosition = allianceRelativeTurretPosition.toTranslation2d();
        double leftDistance = turretPosition.getDistance(targetLayout.leftPassingTarget().toTranslation2d());
        double rightDistance = turretPosition.getDistance(targetLayout.rightPassingTarget().toTranslation2d());
        return leftDistance <= rightDistance ? targetLayout.leftPassingTarget() : targetLayout.rightPassingTarget();
    }

    private static Translation3d getPassingTargetForBump(
            Translation2d nearLeftCorner,
            Translation2d nearRightCorner,
            double bumpHeightMeters) {
        Translation2d nearEdgeCenter = nearLeftCorner.interpolate(nearRightCorner, 0.5);
        return new Translation3d(
                nearEdgeCenter.getX(),
                nearEdgeCenter.getY(),
                bumpHeightMeters + ShooterSupersystem.Constants.passingShotClearance.in(edu.wpi.first.units.Units.Meter));
    }
}
