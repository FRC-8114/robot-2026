package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Shooter extends SubsystemBase {
    private static class Constants {
        static final double flywheelToleranceRPM = 50;
    }

    private final ShooterIO io;
    private final ShooterInputsAutoLogged inputs = new ShooterInputsAutoLogged();
    private final SysIdRoutine sysId;

    private AngularVelocity targetVelocity;

    private LoggedNetworkNumber tuneVelocity;
 
    public Shooter(ShooterIO io) {
        this.io = io;

        sysId = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null, null, null,
                        (state) -> Logger.recordOutput("Shooter/SysIdState", state.toString())),
                new SysIdRoutine.Mechanism(
                        (voltage) -> io.setVoltage(voltage.in(Volts)),
                        null, this));
        
        tuneVelocity = new LoggedNetworkNumber("Tuning/TuneShooterVelocityRPM", 2000);
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
        return RPM.of(inputs.leftFlywheelRPMs).minus(targetVelocity).magnitude() < Constants.flywheelToleranceRPM
                && RPM.of(inputs.rightFlywheelRPMs).minus(targetVelocity).magnitude() < Constants.flywheelToleranceRPM;
    }

    public Command runFlywheelsTunableVelocity() {
        return run(() -> io.setFlywheelVelocity(RPM.of(tuneVelocity.get())));
    }

    public Command runFlywheels(AngularVelocity target) {
        targetVelocity = target;

        return runEnd(
                () -> io.setFlywheelVelocity(targetVelocity),
                () -> io.stopFlywheels());
    }

    public Command runFlywheels(Supplier<AngularVelocity> target) {
        return runEnd(
                () -> {
                    targetVelocity = target.get();
                    io.setFlywheelVelocity(targetVelocity);
                },
                () -> io.stopFlywheels());
    }

    public Command runFlywheelsVolts(Voltage volts) {
        return runEnd(() -> io.setVoltage(volts.in(Volts)), () -> io.stopFlywheels());
    }

    public Command stopFlywheels() {
        return runOnce(() -> io.stopFlywheels());
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysId.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysId.dynamic(direction);
    }
}
