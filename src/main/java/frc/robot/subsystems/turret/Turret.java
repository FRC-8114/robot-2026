package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Degrees;
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
        public static final Angle MIN_ANGLE = Degrees.of(-90);
        public static final Angle MAX_ANGLE = Degrees.of(180);
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

        setDefaultCommand(followAngle(() -> Degrees.of(MathUtil.clamp(turretPosition.get(),
                Constants.MIN_ANGLE.in(Degrees), Constants.MAX_ANGLE.in(Degrees)))));
    }

    public double getTurretPositionRads() {
        return Math.toRadians(normalizedPositionDeg);
    }

    @Override
    public void periodic() {
        pivotMotor.updateInputs(inputs);
        Logger.processInputs("Turret", inputs);
        normalizedPositionDeg = MathUtil.inputModulus(Math.toDegrees(inputs.turretMotorPosition), -180, 180);
        Logger.recordOutput("Turret/NormalizedPositionDeg", normalizedPositionDeg);
    }

    public boolean isAtAngle(Angle target) {
        double error = MathUtil.inputModulus(target.in(Degrees) - normalizedPositionDeg, -180, 180);
        return Math.abs(error) <= Constants.ANGLE_TOLERANCE.in(Degrees);
    }

    public Command setAngle(Angle angle) {
        return run(() -> pivotMotor.setTarget(angle));
    }

    public Command followAngle(Supplier<Angle> angle) {
        return run(() -> pivotMotor.setTarget(angle.get()));
    }

    private boolean isOutOfBounds() {
        return normalizedPositionDeg < Constants.MIN_ANGLE.in(Degrees)
                || normalizedPositionDeg > Constants.MAX_ANGLE.in(Degrees);
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
