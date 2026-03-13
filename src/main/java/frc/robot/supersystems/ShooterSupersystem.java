package frc.robot.supersystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;

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
import frc.robot.util.AllianceFlipUtil;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooterpitch.ShooterPitch;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turretloader.TurretLoader;
import frc.robot.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.supersystems.ShotSolutionCalculator.TargetSelection;

public class ShooterSupersystem extends SubsystemBase {
    private final Turret turretPivot;
    private final ShooterPitch shooterPitch;
    private final Shooter shooter;
    private final Indexer indexer;
    private final TurretLoader turretLoader;
    private final Drive drive;

    private final InterpolatingMatrixTreeMap<Double, N2, N1> distanceToPitchAndRPM = new InterpolatingMatrixTreeMap<Double, N2, N1>();

    public static class Constants {
        public static final Distance turretXOffset = Inches.of(-6.5); // negative X is back
        public static final Distance turretYOffset = Inches.of(6.875); // positive Y is left
        public static final Distance turretZOffset = Inches.of(20.5); // positive Z is up
        public static final Distance passingShotClearance = Inches.of(18.0);

        public static final Transform3d turretOffset = new Transform3d(turretXOffset, turretYOffset, turretZOffset,
                new Rotation3d());

        public static final double FLYWHEEL_RADIUS_METERS = 0.050;
        public static final double SPIN_TRANSFER_EFFICIENCY = 0.75;
    }

    private static final LoggedNetworkBoolean staticTurretMode = new LoggedNetworkBoolean("Tuning/TurretStaticMode", false);

    private void putMeasurement(Distance dist, double pitchDegrees, double rpm) {
        double pitchRad = Math.toRadians(pitchDegrees);
        distanceToPitchAndRPM.put(dist.in(Meter), new Matrix<N2, N1>(new SimpleMatrix(new double[] { pitchRad, rpm })));
    }

    public ShooterSupersystem(
            Turret turretPivot,
            ShooterPitch shooterPitch,
            Shooter shooter,
            Indexer indexer,
            TurretLoader turretLoader,
            Drive drive) {
        putMeasurement(Feet.of(7), 20, 1300);
        putMeasurement(Feet.of(8), 24.0, 1450);
        putMeasurement(Feet.of(9), 28.0, 1485);
        putMeasurement(Feet.of(10), 28.0, 1555);
        putMeasurement(Feet.of(11), 28.0, 1630);
        putMeasurement(Feet.of(13), 28.0, 1820);
        putMeasurement(Feet.of(16), 28.0, 2060);

        putMeasurement(Feet.of(25), 33, 2225);
        putMeasurement(Feet.of(40), 33, 2800);

        this.turretPivot = turretPivot;
        this.shooterPitch = shooterPitch;
        this.shooter = shooter;
        this.indexer = indexer;
        this.turretLoader = turretLoader;
        this.drive = drive;

        setDefaultCommand(defaultHoming());
        new Trigger(staticTurretMode)
            .whileTrue(turretPivot.setAngle(Degrees.of(180)));
    }

    private Pair<Double, Double> getRPMAndPitch(double distance) {
        var mat = distanceToPitchAndRPM.get(distance).getData();

        return Pair.of(mat[1], mat[0]);
    }

    private double estimateTimeOfFlight(double horizontalDist) {
        var rpmAndPitch = getRPMAndPitch(horizontalDist);
        double rpm = rpmAndPitch.getFirst();
        double pitchRad = rpmAndPitch.getSecond();
        double exitVelocity =
                (rpm * 2.0 * Math.PI * Constants.FLYWHEEL_RADIUS_METERS / 60.0) * Constants.SPIN_TRANSFER_EFFICIENCY;

        Logger.recordOutput("Shooter/exitVelocity", exitVelocity);

        return horizontalDist / (exitVelocity * Math.cos(pitchRad));
    }

    private Translation3d getTurretPosition() {
        return new Pose3d(drive.getPose()).transformBy(Constants.turretOffset).getTranslation();
    }

    private Translation3d getTarget(Translation3d turretPosition) {
        TargetSelection targetSelection = ShotSolutionCalculator.getTargetSelection(turretPosition);

        Logger.recordOutput("Shooter/PassingShotActive", targetSelection.passingShotActive());
        Logger.recordOutput(
                "Shooter/AllianceRelativeTargetPosition",
                AllianceFlipUtil.apply(targetSelection.target()).toTranslation2d());
        return targetSelection.target();
    }

    private Translation3d getTarget() {
        return getTarget(getTurretPosition());
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
        Translation2d fieldRelativeTargetVector = getFieldRelativeLeadTargetVector(turretPosition, targetPosition,
                fieldVelocity, timeOfFlight);
        return getRobotRelativeYaw(fieldRelativeTargetVector, robotHeading);
    }

    private Angle getLeadYaw() {
        ChassisSpeeds robotSpeeds = drive.getChassisSpeeds();
        Rotation2d heading = drive.getPose().getRotation();
        ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(robotSpeeds, heading);

        Translation3d turretPosition = getTurretPosition();
        Translation3d target = getTarget(turretPosition);
        double distance = target.toTranslation2d().getDistance(turretPosition.toTranslation2d());

        double tof = estimateTimeOfFlight(distance);
        Translation2d targetPosition = target.toTranslation2d();
        Translation2d turretPosition2d = turretPosition.toTranslation2d();
        Translation2d fieldVelocity = new Translation2d(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond);
        Translation2d fieldRelativeTargetVector = getFieldRelativeLeadTargetVector(turretPosition2d, targetPosition,
                fieldVelocity, tof);
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

    public Command shootWhenReady(
            Supplier<Angle> turretAngle,
            Supplier<Angle> pitchAngle,
            Supplier<AngularVelocity> rpm) {
        return Commands.parallel(
            run(() -> {}),
            turretPivot.followAngle(turretAngle),
            shooterPitch.followAngle(pitchAngle),
            shooter.runFlywheels(rpm),
            turretLoader.feed(),
            Commands.print("SOO WE'RE RUNNING IN HERE"),
            Commands.sequence(
                Commands.waitTime(Seconds.of(0.2)), // wait for turretLane to speed up
                indexer.feed()
            )
        );
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

    public Command shootAtTarget() {
        return shootWhenReady(
            this::getLeadYaw,
            this::getPitchAngle,
            this::getShooterRPM
        );
    }

    public Command defaultHoming() {
        Supplier<Angle> yaw = this::getLeadYaw;
        // Supplier<Angle> pitch = this::getPitchAngle;

        return Commands.parallel(
            turretPivot.followAngle(yaw)
                .unless(staticTurretMode),
            shooterPitch.setAngle(ShooterPitch.Constants.MIN_ANGLE), // must fit under trench
            run(() -> {})
        );
    }
}
