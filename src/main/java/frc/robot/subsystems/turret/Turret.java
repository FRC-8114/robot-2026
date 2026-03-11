package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Turret extends SubsystemBase {
    public static class Constants {
        private static final Angle ANGLE_TOLERANCE = Degrees.of(1);
        public static final Angle MIN_ANGLE = Radians.of(4.988214 - (2.0 * Math.PI));
        public static final Angle MAX_ANGLE = Radians.of(0.75185);
    }

    private final TurretIO pivotMotor;
    private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
    private final SysIdRoutine sysId;
    private double normalizedPositionDeg;

    private final LoggedNetworkNumber turretPosition = new LoggedNetworkNumber("Tuning/TurretPosition", 0);

    public Turret(TurretIO pivotMotor) {
        this.pivotMotor = pivotMotor;

        sysId = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null, null, null,
                        (state) -> Logger.recordOutput("Turret/SysIdState", state.toString())),
                new SysIdRoutine.Mechanism(
                        (voltage) -> pivotMotor.setVoltage(voltage.in(Volts)), null, this));

    }

    public static double wrapAngleRadians(double angleRadians) {
        return MathUtil.inputModulus(angleRadians, -Math.PI, Math.PI);
    }

    public static Angle normalizeAngle(Angle angle) {
        return Radians.of(wrapAngleRadians(angle.in(Radians)));
    }

    public static Angle clampAngle(Angle angle) {
        double wrappedRadians = wrapAngleRadians(angle.in(Radians));
        
        return Radians.of(MathUtil.clamp(
                wrappedRadians,
                Constants.MIN_ANGLE.in(Radians),
                Constants.MAX_ANGLE.in(Radians)));
    }

    public double getTurretPositionRads() {
        return wrapAngleRadians(inputs.turretMotorPosition);
    }

    @Override
    public void periodic() {
        pivotMotor.updateInputs(inputs);
        Logger.processInputs("Turret", inputs);
        normalizedPositionDeg = Math.toDegrees(wrapAngleRadians(inputs.turretMotorPosition));
        Logger.recordOutput("Turret/NormalizedPositionDeg", normalizedPositionDeg);
    }

    public boolean isAtAngle(Angle target) {
        double error = MathUtil.inputModulus(clampAngle(target).in(Degrees) - normalizedPositionDeg, -180, 180);
        return Math.abs(error) <= Constants.ANGLE_TOLERANCE.in(Degrees);
    }

    public Command setAngle(Angle angle) {
        return run(() -> pivotMotor.setTarget(clampAngle(angle)));
    }

    public Command followAngle(Supplier<Angle> angle) {
        return run(() -> pivotMotor.setTarget(clampAngle(angle.get())));
    }

    private boolean isOutOfBounds() {
        double signedPosition = wrapAngleRadians(inputs.turretMotorPosition);
        return signedPosition < Constants.MIN_ANGLE.in(Radians)
                || signedPosition > Constants.MAX_ANGLE.in(Radians);
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
