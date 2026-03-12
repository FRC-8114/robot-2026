package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    private enum ClimbState {
        DEPLOY,
        CLIMB,
        UNCLIMB,
        STOW
    }

    private static class Constants {
        // drum diameter: 0.787in
        public static final double DEPLOY_ROTATIONS = 3.023;
        public static final double CLIMB_ROTATIONS = 1.511;

        static final double rotationTolerance = 0.025;
    }

    private ClimbState state;

    private final ClimberIO climber;
    private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

    public Climber(ClimberIO io) {
        this.climber = io;

        this.state = ClimbState.STOW;
    }

    private Command doRotationsDownCommand(double rotations) {
        double startRotations = inputs.rotations;
        return run(() -> climber.runVolts(Volts.of(-3)))
            .until(() -> MathUtil.isNear(rotations, startRotations - inputs.rotations, Constants.rotationTolerance));
    }
    private Command doRotationsUpCommand(double rotations) {
        double startRotations = inputs.rotations;
        return run(() -> climber.runVolts(Volts.of(3)))
            .until(() -> MathUtil.isNear(rotations, inputs.rotations - startRotations, Constants.rotationTolerance));
    }

    public Command deploy() {
        state = ClimbState.DEPLOY;
        return doRotationsUpCommand(Constants.DEPLOY_ROTATIONS);
    }

    public Command climb() {
        state = ClimbState.CLIMB;
        return doRotationsDownCommand(Constants.CLIMB_ROTATIONS);
    }

    public Command unclimb() {
        state = ClimbState.UNCLIMB;
        return doRotationsUpCommand(Constants.CLIMB_ROTATIONS);
    }

    public Command stow() {
        state = ClimbState.STOW;
        return doRotationsDownCommand(Constants.DEPLOY_ROTATIONS);
    }

    public Command doNext() {
        return switch (state) {
            case DEPLOY -> climb();
            case CLIMB -> unclimb();
            case UNCLIMB -> stow();
            case STOW -> deploy();
            default -> Commands.none();
        };
    }

    @Override
    public void periodic() {
        climber.updateInputs(inputs);
    }
}
