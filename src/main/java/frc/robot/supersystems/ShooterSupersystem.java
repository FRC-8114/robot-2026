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
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooterpitch.ShooterPitch;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turretloader.TurretLoader;
import frc.robot.supersystems.ShotSolutionCalculator.ShotParameters;
import frc.robot.supersystems.ShotSolutionCalculator.TargetSelection;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intakepivot.IntakePivot;
import frc.robot.util.AllianceFlipUtil;

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
        public static final Distance passingShotClearance = Inches.of(18.0);

        public static final Transform3d turretOffset = new Transform3d(turretXOffset, turretYOffset, turretZOffset,
                new Rotation3d());

        public static final double FLYWHEEL_RADIUS_METERS = 0.050;
        public static final double SPIN_TRANSFER_EFFICIENCY = 0.75;
    }

    private static final LoggedNetworkBoolean staticTurretMode = new LoggedNetworkBoolean("Tuning/TurretStaticMode",
            false);

    private void putMeasurement(Distance dist, double pitchDegrees, double rpm) {
        double pitchRad = Degrees.of(pitchDegrees).in(Radians);
        distanceToPitchAndRPM.put(dist.in(Meter), new Matrix<N2, N1>(new SimpleMatrix(new double[] { pitchRad, rpm })));
    }

    public ShooterSupersystem(Turret turretPivot, ShooterPitch turretPitch, Shooter shooter, Indexer indexer,
            TurretLoader turretLoader,
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

    private ShotParameters getShotParameters(double distance) {
        var mat = distanceToPitchAndRPM.get(distance).getData();

        return new ShotParameters(mat[1], mat[0]);
    }

    private Translation3d getTurretPosition() {
        return new Pose3d(drive.getPose()).transformBy(Constants.turretOffset).getTranslation();
    }

    private TargetSelection getTargetSelection(Translation3d turretPosition) {
        TargetSelection targetSelection = ShotSolutionCalculator.getTargetSelection(turretPosition);

        Logger.recordOutput("Shooter/PassingShotActive", targetSelection.passingShotActive());
        Logger.recordOutput(
                "Shooter/AllianceRelativeTargetPosition",
                AllianceFlipUtil.apply(targetSelection.target()).toTranslation2d());
        return targetSelection;
    }

    private Translation3d getTarget(Translation3d turretPosition) {
        return getTargetSelection(turretPosition).target();
    }

    private Translation3d getTarget() {
        return getTarget(getTurretPosition());
    }

    private double estimateTimeOfFlight(double horizontalDist) {
        ShotParameters shotParameters = getShotParameters(horizontalDist);
        double exitVelocity = (shotParameters.rpm() * 2.0 * Math.PI * Constants.FLYWHEEL_RADIUS_METERS / 60.0)
                * Constants.SPIN_TRANSFER_EFFICIENCY;

        return horizontalDist / (exitVelocity * Math.cos(shotParameters.pitchRadians()));
    }

    private double getDistanceToTarget() {
        double distance = getTarget().toTranslation2d().getDistance(getTurretPosition().toTranslation2d());
        Logger.recordOutput("Shooter/DistanceToTargetMeters", distance);
        return distance;
    }

    private Angle getLeadYaw() {
        ChassisSpeeds robotSpeeds = drive.getChassisSpeeds();
        Rotation2d heading = drive.getPose().getRotation();
        ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(robotSpeeds, heading);
        Translation2d fieldVelocity = new Translation2d(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond);

        Translation3d turretPosition = getTurretPosition();
        Translation3d target = getTarget(turretPosition);
        double distance = target.toTranslation2d().getDistance(turretPosition.toTranslation2d());

        double timeOfFlight = estimateTimeOfFlight(distance);
        Translation2d fieldRelativeTargetVector = ShotSolutionCalculator.getFieldRelativeLeadTargetVector(
                turretPosition.toTranslation2d(),
                target.toTranslation2d(),
                fieldVelocity,
                timeOfFlight);
        Angle turretAngle = ShotSolutionCalculator.getRobotRelativeYaw(fieldRelativeTargetVector, heading);
        return Turret.clampAngle(turretAngle);
    }

    private Angle getPitchAngle() {
        double distance = getDistanceToTarget();
        return Radians.of(getShotParameters(distance).pitchRadians());
    }

    private AngularVelocity getShooterRPM() {
        double distance = getDistanceToTarget();
        return RPM.of(getShotParameters(distance).rpm());
    }

    public boolean isReadyToFire(Angle turretAngle, Angle pitchAngle) {
        return turretPivot.isAtAngle(turretAngle)
                && turretPitch.isAtAngle(pitchAngle)
                && shooter.atSpeed.getAsBoolean()
                && turretLoader.atSpeed.getAsBoolean();
    }

    public Command shootWhenReady(
            Supplier<Angle> turretAngle,
            Supplier<Angle> pitchAngle,
            Supplier<AngularVelocity> rpm,
            BooleanSupplier fireTrigger) {
        BooleanSupplier shouldFeed = () -> fireTrigger.getAsBoolean()
                && isReadyToFire(turretAngle.get(), pitchAngle.get());

        return Commands.parallel(
                run(() -> {
                    Logger.recordOutput("Shooter/ShouldFeed", shouldFeed.getAsBoolean());
                }),
                turretPivot.followAngle(turretAngle),
                turretPitch.followAngle(pitchAngle),
                shooter.runFlywheels(rpm),
                turretLoader.feed(),
                Commands.sequence(
                        Commands.waitUntil(shouldFeed),
                        indexer.feed()));
    }

    public Command shootAtTarget(BooleanSupplier fireTrigger) {
        return shootWhenReady(
                this::getLeadYaw,
                this::getPitchAngle,
                this::getShooterRPM,
                fireTrigger);
    }

    public Command defaultHoming() {
        BooleanSupplier turretCanMove = () -> !staticTurretMode.get() && intakePivot.isDeployed.getAsBoolean();
        Supplier<Angle> yaw = () -> turretCanMove.getAsBoolean()
                ? getLeadYaw()
                : Radians.of(turretPivot.getTurretPositionRads());
        Supplier<Angle> pitch = this::getPitchAngle;

        return Commands.parallel(
                run(() -> {
                }),
                turretPivot.followAngle(yaw),
                turretPitch.followAngle(pitch));
    }
}
