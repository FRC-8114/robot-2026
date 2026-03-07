package frc.robot.subsystems.climber;

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
                        (state) -> {
                        }),
                new SysIdRoutine.Mechanism(
                        (voltage) -> climber.setVoltage(voltage), null, this));
    }

    private Command doRotationsCommand(double rotations) {
        double startRotations = inputs.rotations;
        return run(() -> climber.doRotations(rotations))
            .until(() -> startRotations - inputs.rotations == rotations);
    }

    public Command deploy() {
        return doRotationsCommand(Constants.DEPLOY_ROTATIONS);
    }

    public Command climb() {
        return doRotationsCommand(Constants.CLIMB_ROTATIONS);
    }

    public Command unclimb() {
        return doRotationsCommand(-Constants.CLIMB_ROTATIONS);
    }

    public Command stow() {
        return doRotationsCommand(-Constants.CLIMB_ROTATIONS);
    }

    @Override
    public void periodic() {
        climber.updateInputs(inputs);
    }
}
