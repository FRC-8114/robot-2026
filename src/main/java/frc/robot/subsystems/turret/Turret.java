package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Turret extends SubsystemBase {
    public static class Constants {
        private static final Angle ANGLE_TOLERANCE = Degrees.of(10);

        public static final Angle MIN_ANGLE = Degrees.of(40);
        public static final Angle MAX_ANGLE = Radians.of(4.85);
    }

    private final TurretIO pivotMotor;
    private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
    private final SysIdRoutine sysId;

    public Turret(TurretIO pivotMotor) {
        this.pivotMotor = pivotMotor;

        sysId = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null, null, null,
                        (state) -> Logger.recordOutput("Turret/SysIdState", state.toString())),
                new SysIdRoutine.Mechanism(
                        (voltage) -> pivotMotor.setVoltage(voltage.in(Volts)), null, this));

    }

    public static Angle normalizeAngle(Angle angle) {
        double fullRotationRadians = 2.0 * Math.PI;
        double normalizedAngle = angle.in(Radians) % fullRotationRadians;

        if (normalizedAngle < 0.0) {
            normalizedAngle += fullRotationRadians;
        }

        return Radians.of(normalizedAngle);
    }

    public static Angle clampAngle(Angle angle) {
        double targetRadians = angle.in(Radians);
        double minAngleRadians = Constants.MIN_ANGLE.in(Radians);
        double maxAngleRadians = Constants.MAX_ANGLE.in(Radians);
        double fullRotationRadians = 2.0 * Math.PI;

        long minimumTurnsToEnterRange = (long) Math.ceil((minAngleRadians - targetRadians) / fullRotationRadians);
        long maximumTurnsToEnterRange = (long) Math.floor((maxAngleRadians - targetRadians) / fullRotationRadians);

        if (minimumTurnsToEnterRange <= maximumTurnsToEnterRange) {
            long turns = minimumTurnsToEnterRange <= 0 && 0 <= maximumTurnsToEnterRange
                    ? 0
                    : (Math.abs(minimumTurnsToEnterRange) <= Math.abs(maximumTurnsToEnterRange)
                            ? minimumTurnsToEnterRange
                            : maximumTurnsToEnterRange);
            return Radians.of(targetRadians + (turns * fullRotationRadians));
        }

        double distanceToMinAngle = Math.abs(MathUtil.inputModulus(
                targetRadians - minAngleRadians,
                -Math.PI,
                Math.PI));
        double distanceToMaxAngle = Math.abs(MathUtil.inputModulus(
                targetRadians - maxAngleRadians,
                -Math.PI,
                Math.PI));

        return Radians.of(distanceToMinAngle <= distanceToMaxAngle ? minAngleRadians : maxAngleRadians);
    }

    public double getTurretPositionRads() {
        return normalizeAngle(Radians.of(inputs.turretMotorPosition)).in(Radians);
    }

    @Override
    public void periodic() {
        pivotMotor.updateInputs(inputs);
        Logger.processInputs("Turret", inputs);
        Logger.recordOutput("Turret/NormalizedPositionDeg", Math.toDegrees(getTurretPositionRads()));
    }

    public boolean isAtAngle(Angle target) {
        double error = MathUtil.inputModulus(
                clampAngle(target).in(Radians) - getTurretPositionRads(),
                -Math.PI,
                Math.PI);
        return Math.abs(error) <= Constants.ANGLE_TOLERANCE.in(Radians);
    }

    public Command setAngle(Angle angle) {
        return run(() -> pivotMotor.setTarget(angle));
    }

    public Command followAngle(Supplier<Angle> angle) {
        return run(() -> pivotMotor.setTarget(angle.get()));
    }

    private boolean isOutOfBounds() {
        return inputs.turretMotorPosition < Constants.MIN_ANGLE.in(Radians)
                || inputs.turretMotorPosition > Constants.MAX_ANGLE.in(Radians);
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return run(() -> pivotMotor.setVoltage(0.0))
                .withTimeout(1.0)
                .andThen(sysId.quasistatic(direction)
                        .until(this::isOutOfBounds)
                        .finallyDo(() -> pivotMotor.setVoltage(0.0)));
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return run(() -> pivotMotor.setVoltage(0.0))
                .withTimeout(1.0)
                .andThen(sysId.dynamic(direction)
                        .until(this::isOutOfBounds)
                        .finallyDo(() -> pivotMotor.setVoltage(0.0)));
    }
}
