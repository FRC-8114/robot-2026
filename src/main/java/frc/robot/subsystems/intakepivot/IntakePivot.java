package frc.robot.subsystems.intakepivot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class IntakePivot extends SubsystemBase {
    public static final Angle stowAngle = Rotations.of(0); // TODO: measure stow angle
    private static final Angle deployAngle = Rotations.of(-0.0036019423);
    private static final Angle angleTolerance = Degrees.of(3);

    private IntakePivotIO io;
    private final IntakePivotInputsAutoLogged inputs = new IntakePivotInputsAutoLogged();

    private SysIdRoutine sysId;

    public IntakePivot(IntakePivotIO io) {
        this.io = io;

        sysId = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null, null, null,
                        (state) -> Logger.recordOutput("IntakePivot/SysIdState", state.toString())),
                new SysIdRoutine.Mechanism(
                        (voltage) -> io.runVolts(voltage), null, this));
    }

    private Trigger isDeployed = new Trigger(() -> Radians.of(inputs.positionRads).isNear(deployAngle, angleTolerance));
    private Trigger isStowed = new Trigger(() -> Radians.of(inputs.positionRads).isNear(stowAngle, angleTolerance));

    public Command periodicPulse() {
        return Commands.repeatingSequence(
            run(() -> io.runVolts(Volts.of(2))),
            Commands.waitTime(Seconds.of(0.25)),
            deploy(),
            Commands.waitTime(Seconds.of(1))
        )
            .finallyDo(this::deploy);  
    }

    public Command deploy() {
        return run(() -> io.setTarget(deployAngle))
            // .until(isDeployed)
            .withName("DeployIntake");
    }

    public Command stow() {
        return run(() -> io.setTarget(stowAngle))
            .until(isStowed);
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysId.quasistatic(direction)
            .until(isDeployed.or(isStowed));
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysId.dynamic(direction)
            .until(isDeployed.or(isStowed));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("IntakePivot", inputs);
    }
}
