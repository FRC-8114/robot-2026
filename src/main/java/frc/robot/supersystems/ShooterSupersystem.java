package frc.robot.supersystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.ejml.simple.SimpleMatrix;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import edu.wpi.first.math.InterpolatingMatrixTreeMap;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooterpitch.ShooterPitch;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turretloader.TurretLoader;
import frc.robot.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intakepivot.IntakePivot;

public class ShooterSupersystem extends SubsystemBase {
    private final Turret turretPivot;
    private final ShooterPitch turretPitch;
    private final Shooter shooter;
    private final Indexer indexer;
    private final TurretLoader turretLoader;
    private final Drive drive;

    private final IntakePivot intakePivot;

    private final InterpolatingMatrixTreeMap<Double, N2, N1> distanceToPitchAndRPM = new InterpolatingMatrixTreeMap<Double, N2, N1>();

    public static class Constants {
        public static final Distance turretXOffset = Inches.of(-6.5); // negative X is back
        public static final Distance turretYOffset = Inches.of(6.875); // positive Y is left
        public static final Distance turretZOffset = Inches.of(20.5); // positive Z is up

        public static final Transform3d turretOffset = new Transform3d(turretXOffset, turretYOffset, turretZOffset,
                new Rotation3d());

        public static final double FLYWHEEL_RADIUS_METERS = 0.050;
    }

    private static final LoggedNetworkBoolean staticTurretMode = new LoggedNetworkBoolean("Tuning/TurretStaticMode",
            false);

    private void putMeasurement(Distance dist, double pitchDegrees, double rpm) {
        double pitchRad = Math.toRadians(pitchDegrees);
        distanceToPitchAndRPM.put(dist.in(Meter), new Matrix<N2, N1>(new SimpleMatrix(new double[] { pitchRad, rpm })));
    }

    public ShooterSupersystem(Turret turretPivot, ShooterPitch turretPitch, Shooter shooter, Indexer indexer, TurretLoader turretLoader,
            Drive drive, IntakePivot intakePivot) {

        putMeasurement(Feet.of(7), 20, 1300);
        putMeasurement(Feet.of(8), 24.0, 1450);
        putMeasurement(Feet.of(9), 28.0, 1485);
        putMeasurement(Feet.of(10), 28.0, 1555);
        putMeasurement(Feet.of(11), 28.0, 1630);
        putMeasurement(Feet.of(13), 28.0, 1820);
        putMeasurement(Feet.of(16), 28.0, 2060);

        putMeasurement(Feet.of(25), 33, 2225);
        putMeasurement(Feet.of(40), 33, 2800);

        // todo: distance to pitch and rpm

        this.turretPivot = turretPivot;
        this.turretPitch = turretPitch;
        this.shooter = shooter;
        this.indexer = indexer;
        this.turretLoader = turretLoader;
        this.drive = drive;
    
        this.intakePivot = intakePivot;

        setDefaultCommand(defaultHoming());
    }

    private Pair<Double, Double> getRPMAndPitch(double distance) {
        var mat = distanceToPitchAndRPM.get(distance).getData();

        return Pair.of(mat[1], mat[0]);
    }

    private double estimateTimeOfFlight(double horizontalDist) {
        var rpmAndPitch = getRPMAndPitch(horizontalDist);
        double rpm = rpmAndPitch.getFirst();
        double pitchRad = rpmAndPitch.getSecond();
        double exitVelocity = (rpm * 2.0 * Math.PI * Constants.FLYWHEEL_RADIUS_METERS / 60.0) * 0.75;

        Logger.recordOutput("Shooter/exitVelocity", exitVelocity);

        return horizontalDist / (exitVelocity * Math.cos(pitchRad));
    }

    private Translation3d getTurretPosition() {
        return new Pose3d(drive.getPose()).transformBy(Constants.turretOffset).getTranslation();
    }

    private Translation3d getTarget() {
        return FieldConstants.Hub.innerCenterPoint;
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

    static Angle getLeadYaw(
            Translation2d turretPosition,
            Translation2d targetPosition,
            Rotation2d robotHeading,
            Translation2d fieldVelocity,
            double timeOfFlight) {
        Translation2d fieldRelativeTargetVector =
                getFieldRelativeLeadTargetVector(turretPosition, targetPosition, fieldVelocity, timeOfFlight);
        return getRobotRelativeYaw(fieldRelativeTargetVector, robotHeading);
    }

    private Angle getLeadYaw() {
        ChassisSpeeds robotSpeeds = drive.getChassisSpeeds();
        Rotation2d heading = drive.getPose().getRotation();
        ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(robotSpeeds, heading);

        Translation3d target = getTarget();
        Translation3d turretPosition = getTurretPosition();
        double distance = target.toTranslation2d().getDistance(turretPosition.toTranslation2d());

        double tof = estimateTimeOfFlight(distance);
        Translation2d targetPosition = target.toTranslation2d();
        Translation2d turretPosition2d = turretPosition.toTranslation2d();
        Translation2d fieldVelocity = new Translation2d(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond);
        Translation2d fieldRelativeTargetVector =
                getFieldRelativeLeadTargetVector(turretPosition2d, targetPosition, fieldVelocity, tof);
        Translation2d robotRelativeTargetVector = getRobotRelativeTargetVector(fieldRelativeTargetVector, heading);
        Angle turretAngle = getRobotRelativeYaw(fieldRelativeTargetVector, heading);

        Logger.recordOutput("Shooter/RobotHeadingRad", heading.getRadians());
        Logger.recordOutput("Shooter/TurretPosition", turretPosition2d);
        Logger.recordOutput("Shooter/TargetPosition", targetPosition);
        Logger.recordOutput("Shooter/FieldVelocity", fieldVelocity);
        Logger.recordOutput("Shooter/TimeOfFlight", tof);
        Logger.recordOutput("Shooter/FieldRelativeTargetVector", fieldRelativeTargetVector);
        Logger.recordOutput("Shooter/RobotRelativeTargetVector", robotRelativeTargetVector);
        Logger.recordOutput("Shooter/LeadYawRawRad", turretAngle.in(Radians));
        return Turret.clampAngle(turretAngle);
    }

    public boolean isReadyToFire(Angle turretAngle, Angle pitchAngle) {
        return turretPivot.isAtAngle(turretAngle)
                && turretPitch.isAtAngle(pitchAngle)
                && shooter.atSpeed.getAsBoolean()
                && turretLoader.atSpeed.getAsBoolean();
    }

    public Command shootWhenReady(Supplier<Angle> turretAngle, Supplier<Angle> pitchAngle, Supplier<AngularVelocity> rpm, BooleanSupplier fireTrigger) {
        Trigger shouldFeed = new Trigger(() -> fireTrigger.getAsBoolean()
                && isReadyToFire(turretAngle.get(), pitchAngle.get()));

        return Commands.parallel(
                turretPivot.followAngle(turretAngle),
                turretPitch.followAngle(pitchAngle),
                shooter.runFlywheels(rpm),
                turretLoader.feed(),
                Commands.sequence(
                    Commands.waitUntil(shouldFeed),
                    indexer.feed()
                ));
    }

    private double getDistanceToTarget() {
        return getTarget().toTranslation2d().getDistance(getTurretPosition().toTranslation2d());
    }

    private Angle getPitchAngle() {
        double distance = getDistanceToTarget();
        return Radians.of(getRPMAndPitch(distance).getSecond());
    }

    private AngularVelocity getShooterRPM() {
        double distance = getDistanceToTarget();
        return RPM.of(getRPMAndPitch(distance).getFirst());
    }

    public Command shootAtTarget(BooleanSupplier fireTrigger) {
        Supplier<Angle> yaw = this::getLeadYaw;
        Supplier<Angle> pitch = this::getPitchAngle;
        Supplier<AngularVelocity> rpm = this::getShooterRPM;

        return shootWhenReady(yaw, pitch, rpm, fireTrigger);
    }

    public Command defaultHoming() {
        Supplier<Angle> yaw = this::getLeadYaw;
        Supplier<Angle> pitch = this::getPitchAngle;

        Trigger turretCanMove = new Trigger(() -> !staticTurretMode.get()) .and(intakePivot.isDeployed);

        return Commands.parallel(
            turretPivot.followAngle(yaw),
                // .onlyIf(turretCanMove),
            turretPitch.followAngle(pitch),
            run(() -> {})
        );
    }
}
