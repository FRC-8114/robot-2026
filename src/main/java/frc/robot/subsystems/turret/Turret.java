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
        private static final Angle ANGLE_TOLERANCE = Degrees.of(1);

        public static final Angle MIN_ANGLE = Radians.of(0);
        public static final Angle MAX_ANGLE = Radians.of(1);
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

    public static Angle clampAngle(Angle angle) {
        return Radians.of(MathUtil.clamp(
                MathUtil.angleModulus(angle.in(Radians)),
                Constants.MIN_ANGLE.in(Radians),
                Constants.MAX_ANGLE.in(Radians)));
    }

    public double getTurretPositionRads() {
        return inputs.turretMotorPosition;
    }

    @Override
    public void periodic() {
        pivotMotor.updateInputs(inputs);
        Logger.processInputs("Turret", inputs);
        Logger.recordOutput("Turret/NormalizedPositionDeg", Math.toDegrees(getTurretPositionRads()));
    }

    public boolean isAtAngle(Angle target) {
        double error = MathUtil.angleModulus(clampAngle(target).in(Radians) - getTurretPositionRads());
        return Math.abs(error) <= Constants.ANGLE_TOLERANCE.in(Radians);
    }

    public Command setAngle(Angle angle) {
        return run(() -> pivotMotor.setTarget(clampAngle(angle)));
    }

    public Command followAngle(Supplier<Angle> angle) {
        return run(() -> pivotMotor.setTarget(clampAngle(angle.get())));
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
