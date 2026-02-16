package frc.robot.subsystems.shooterpitch;

import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class ShooterPitch extends SubsystemBase {
    private final ShooterPitchIO pitchMotor;
    private final ShooterPitchInputsAutoLogged inputs = new ShooterPitchInputsAutoLogged();
    private final SysIdRoutine sysId;

    public ShooterPitch(ShooterPitchIO pitchMotor) {
        this.pitchMotor = pitchMotor;

        sysId = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null, null, null,
                        (state) -> Logger.recordOutput("ShooterPitch/SysIdState", state.toString())),
                new SysIdRoutine.Mechanism(
                        (voltage) -> pitchMotor.setVoltage(voltage.in(Volts)), null, this));
    }

    public double getPitchPositionRads() {
        return inputs.pitchPosition;
    }

    @Override
    public void periodic() {
        pitchMotor.updateInputs(inputs);
    }

    public Command setAngle(Angle pitchAngle) {
        return run(() -> pitchMotor.setTarget(pitchAngle));
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return run(() -> pitchMotor.setVoltage(0.0))
                .withTimeout(1.0)
                .andThen(sysId.quasistatic(direction));
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return run(() -> pitchMotor.setVoltage(0.0))
                .withTimeout(1.0)
                .andThen(sysId.dynamic(direction));
    }
}
