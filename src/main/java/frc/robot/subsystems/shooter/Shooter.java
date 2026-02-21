package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Shooter extends SubsystemBase {
    private static class Constants {
        static final double flywheelTargetRPM = 3000;
        static final double flywheelToleranceRPM = 50;
    }

    private final ShooterIO io;
    private final ShooterInputsAutoLogged inputs = new ShooterInputsAutoLogged();
    private final SysIdRoutine sysId;

    public Shooter(ShooterIO io) {
        this.io = io;

        sysId = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null, null, null,
                        (state) -> Logger.recordOutput("Shooter/SysIdState", state.toString())),
                new SysIdRoutine.Mechanism(
                        (voltage) -> io.setVoltage(voltage.in(Volts)),
                        null, this));
    }

    public double getAverageFlywheelRPMs() {
        return (inputs.leftFlywheelRPMs + inputs.rightFlywheelRPMs) / 2.0;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
    }

    public boolean isAtSpeed() {
        return Math.abs(inputs.leftFlywheelRPMs - Constants.flywheelTargetRPM) < Constants.flywheelToleranceRPM
                && Math.abs(inputs.rightFlywheelRPMs - Constants.flywheelTargetRPM) < Constants.flywheelToleranceRPM;
    }

    public Command runFlywheels() {
        return runEnd(
                () -> io.setFlywheelVelocity(RPM.of(Constants.flywheelTargetRPM)),
                () -> io.stopFlywheels());
    }

    public Command stopFlywheels() {
        return runOnce(() -> io.stopFlywheels());
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return run(() -> io.stopFlywheels())
                .withTimeout(1.0)
                .andThen(sysId.quasistatic(direction));
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return run(() -> io.stopFlywheels())
                .withTimeout(1.0)
                .andThen(sysId.dynamic(direction));
    }
}
