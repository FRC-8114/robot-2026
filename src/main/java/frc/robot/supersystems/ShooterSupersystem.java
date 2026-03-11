package frc.robot.supersystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.ejml.simple.SimpleMatrix;
import org.littletonrobotics.junction.Logger;

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
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooterpitch.ShooterPitch;
import frc.robot.subsystems.turret.Turret;
import frc.robot.FieldConstants;
import frc.robot.subsystems.drive.Drive;

public class ShooterSupersystem extends SubsystemBase {
    private final Turret turretPivot;
    private final ShooterPitch turretPitch;
    private final Shooter shooter;
    private final Indexer indexer;
    private final Drive drive;

    private final InterpolatingMatrixTreeMap<Double, N2, N1> distanceToPitchAndRPM = new InterpolatingMatrixTreeMap<Double, N2, N1>();

    public static class Constants {
        public static final Distance turretXOffset = Inches.of(-6.5); // negative X is back
        public static final Distance turretYOffset = Inches.of(6.875); // positive Y is left
        public static final Distance turretZOffset = Inches.of(20.5); // positive Z is up

        public static final Transform3d turretOffset = new Transform3d(turretXOffset, turretYOffset, turretZOffset, new Rotation3d());

        public static final double FLYWHEEL_RADIUS_METERS = 0.050;
    }

    private void putMeasurement(double meters, double pitchDegrees, double rpm) {
        double pitchRad = Math.toRadians(pitchDegrees);
        distanceToPitchAndRPM.put(meters, new Matrix<N2, N1>(new SimpleMatrix(new double[] { pitchRad, rpm })));
    }

    public ShooterSupersystem(Turret turretPivot, ShooterPitch turretPitch, Shooter shooter, Indexer indexer,
            Drive drive) {

        putMeasurement(1, 11, 1200);
        putMeasurement(2, 15, 1200);

        // todo: distance to pitch and rpm

        this.turretPivot = turretPivot;
        this.turretPitch = turretPitch;
        this.shooter = shooter;
        this.indexer = indexer;
        this.drive = drive;

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
        if (drive.getPose().getX() > FieldConstants.LinesVertical.allianceZone) {
            return new Translation3d(); // TODO: pass
        }

        return FieldConstants.Hub.innerCenterPoint;
    }

    private Angle getLeadYaw() {
        ChassisSpeeds robotSpeeds = drive.getChassisSpeeds();
        double heading = drive.getPose().getRotation().getRadians();
        ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(robotSpeeds, Rotation2d.fromRadians(heading));

        Translation3d target = getTarget();
        Translation3d turretPosition = getTurretPosition();
        double distance = target.toTranslation2d().getDistance(turretPosition.toTranslation2d());

        double tof = estimateTimeOfFlight(distance);
        Translation2d velocityOffset = new Translation2d(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond)
                .times(tof);
        Translation2d offsetTarget = target.toTranslation2d().minus(velocityOffset);
            
        Logger.recordOutput("Shooter/offsetTarget", offsetTarget);

        Translation2d offsetFromTurret = offsetTarget.minus(turretPosition.toTranslation2d());

        double fieldAngle = Math.atan2(offsetFromTurret.getY(), offsetFromTurret.getX());
        return Turret.clampAngle(Radians.of(fieldAngle - heading));
    }

    public boolean isReadyToFire(Angle turretAngle, Angle pitchAngle) {
        return turretPivot.isAtAngle(turretAngle)
                && turretPitch.isAtAngle(pitchAngle)
                && shooter.isAtSpeed()
                && indexer.isTurretLaneAtSpeed();
    }

    public Command shootWhenReady(Supplier<Angle> turretAngle, Supplier<Angle> pitchAngle,
            Supplier<AngularVelocity> rpm, BooleanSupplier fireTrigger) {
        BooleanSupplier shouldFeed = () -> fireTrigger.getAsBoolean()
                && isReadyToFire(turretAngle.get(), pitchAngle.get());

        return Commands.parallel(
                turretPivot.followAngle(turretAngle),
                turretPitch.followAngle(pitchAngle),
                shooter.runFlywheels(rpm),
                indexer.prepareAndFeedWhen(shouldFeed));
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

        return Commands.parallel(turretPivot.followAngle(yaw),
                turretPitch.followAngle(pitch));
    }
}
