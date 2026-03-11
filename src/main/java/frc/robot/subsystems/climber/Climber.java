package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Climber extends SubsystemBase {
    private static class Constants {
        // drum diameter: 0.787in
        public static final double DEPLOY_ROTATIONS = 3.023;
        public static final double CLIMB_ROTATIONS = 1.511;
    }

    private final ClimberIO climber;
    private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
    private final SysIdRoutine sysId;

    public Climber(ClimberIO io) {
        this.climber = io;

        sysId = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null, null, null,
                        (state) -> Logger.recordOutput("Climber/SysIdState", state.toString())),
                new SysIdRoutine.Mechanism(
                        (voltage) -> climber.runVolts(voltage), null, this));
    }

    private Command doRotationsDownCommand(double rotations) {
        double startRotations = inputs.rotations;
        return run(() -> climber.runVolts(Volts.of(-3)))
            .until(() -> startRotations - inputs.rotations == rotations);
    }
    private Command doRotationsUpCommand(double rotations) {
        double startRotations = inputs.rotations;
        return run(() -> climber.runVolts(Volts.of(3)))
            .until(() -> startRotations - inputs.rotations == rotations);
    }

    public Command deploy() {
        return doRotationsUpCommand(Constants.DEPLOY_ROTATIONS);
    }

    public Command climb() {
        return doRotationsDownCommand(Constants.CLIMB_ROTATIONS);
    }

    public Command unclimb() {
        return doRotationsUpCommand(-Constants.CLIMB_ROTATIONS);
    }

    public Command stow() {
        return doRotationsDownCommand(-Constants.CLIMB_ROTATIONS);
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return switch (direction) {
            case kForward ->
                sysId.quasistatic(direction).until(() -> inputs.rotations == Constants.CLIMB_ROTATIONS);
            case kReverse ->
                sysId.quasistatic(direction).until(() -> inputs.rotations == 0);
        };
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return switch (direction) {
            case kForward ->
                sysId.dynamic(direction).until(() -> inputs.rotations == Constants.CLIMB_ROTATIONS);
            case kReverse ->
                sysId.dynamic(direction).until(() -> inputs.rotations == 0);
        };
    }

    @Override
    public void periodic() {
        climber.updateInputs(inputs);
    }
}
