package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Climber extends SubsystemBase {
    private static class Constants {
        public static final double DEPLOY_HEIGHT = 0;
        public static final double CLIMB_HEIGHT = 0;
        public static final double RETRACT_HEIGHT = 0;
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

    private Command goToHeightCommand(double height) {
        return run(() -> climber.setTargetHeight(height));
    }

    public boolean isDeployed() {
        return inputs.heightMeters > (Constants.DEPLOY_HEIGHT);
    }

    public boolean isClimbed() {
        return inputs.heightMeters > (Constants.CLIMB_HEIGHT);
    }

    public boolean isRetracted() {
        return inputs.heightMeters < (Constants.RETRACT_HEIGHT + 0.1);
    }

    public Command deploy() {
        return goToHeightCommand(Constants.DEPLOY_HEIGHT);
    }

    public Command climb() {
        return goToHeightCommand(Constants.CLIMB_HEIGHT);
    }

    public Command retract() {
        return goToHeightCommand(Constants.RETRACT_HEIGHT);
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return run(() -> climber.setVoltage(Volts.of(0.0)))
                .withTimeout(1.0)
                .andThen(sysId.quasistatic(direction).until(() -> isDeployed() || isRetracted()))
                .finallyDo(() -> climber.setVoltage(Volts.of(0.0)));
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return run(() -> climber.setVoltage(Volts.of(0.0)))
                .withTimeout(1.0)
                .andThen(sysId.dynamic(direction).until(() -> isDeployed() || isRetracted()))
                .finallyDo(() -> climber.setVoltage(Volts.of(0.0)));
    }

    @Override
    public void periodic() {
        climber.updateInputs(inputs);
    }
}
