package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Turret extends SubsystemBase {
    private static class Constants {
        private static final Angle ANGLE_TOLERANCE = Degrees.of(1);
        private static final Angle MIN_ANGLE = Degrees.of(-180);
        private static final Angle MAX_ANGLE = Degrees.of(180);
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

    public double getTurretPositionRads() {
        return MathUtil.angleModulus(inputs.turretMotorPosition);
    }

    @Override
    public void periodic() {
        pivotMotor.updateInputs(inputs);
        Logger.processInputs("Turret", inputs);
        Logger.recordOutput("Turret/NormalizedPositionDeg",
                Math.toDegrees(MathUtil.angleModulus(inputs.turretMotorPosition)));
    }

    public boolean isAtAngle(Angle target) {
        double normalizedTarget = MathUtil.inputModulus(target.in(Degrees), -180, 180);
        double normalizedCurrent = MathUtil.inputModulus(
                Math.toDegrees(inputs.turretMotorPosition), -180, 180);
        double error = MathUtil.inputModulus(normalizedTarget - normalizedCurrent, -180, 180);
        return Math.abs(error) <= Constants.ANGLE_TOLERANCE.in(Degrees);
    }

    private Angle computeMotorTarget(Angle target) {
        double targetDeg = MathUtil.inputModulus(target.in(Degrees), -180, 180);
        targetDeg = MathUtil.clamp(targetDeg, Constants.MIN_ANGLE.in(Degrees), Constants.MAX_ANGLE.in(Degrees));

        double currentDeg = Math.toDegrees(inputs.turretMotorPosition);
        double normalizedCurrentDeg = MathUtil.inputModulus(currentDeg, -180, 180);
        double delta = MathUtil.inputModulus(targetDeg - normalizedCurrentDeg, -180, 180);

        return Degrees.of(currentDeg + delta);
    }

    public Command setAngle(Angle angle) {
        return run(() -> pivotMotor.setTarget(computeMotorTarget(angle)));
    }

    public Command followAngle(Supplier<Angle> angle) {
        return run(() -> pivotMotor.setTarget(computeMotorTarget(angle.get())));
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return run(() -> pivotMotor.setVoltage(0.0))
                .withTimeout(1.0)
                .andThen(sysId.quasistatic(direction));
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return run(() -> pivotMotor.setVoltage(0.0))
                .withTimeout(1.0)
                .andThen(sysId.dynamic(direction));
    }
}
