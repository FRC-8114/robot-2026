package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.intake.IntakeIO.IntakeInputs;

public class Intake extends SubsystemBase {
    private static class Constants {
        static final Angle stowedAngle = Degrees.of(0);
        static final Angle deployedAngle = Degrees.of(180); // TODO: tune

        static final double deploySysIdMinRads = Math.toRadians(5); // TODO: tune
        static final double deploySysIdMaxRads = Math.toRadians(170); // TODO: tune

        static final AngularVelocity intakeVelocity = RPM.of(3000);
        static final AngularVelocity ejectVelocity = RPM.of(-1500);
    }

    private final IntakeIO io;
    private final IntakeInputs inputs = new IntakeInputs();
    private final SysIdRoutine deploySysId;
    private final SysIdRoutine rollerSysId;

    @AutoLogOutput
    private final LoggedMechanism2d mechanism = new LoggedMechanism2d(3, 3);
    private final LoggedMechanismLigament2d armLigament;
    private final LoggedMechanismLigament2d rollerLigament;

    public Intake(IntakeIO io) {
        this.io = io;

        deploySysId = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null, null, null,
                        (state) -> Logger.recordOutput("Intake/DeploySysIdState", state.toString())),
                new SysIdRoutine.Mechanism(
                        (voltage) -> io.setDeployVoltage(voltage.in(Volts)), null, this));

        rollerSysId = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null, null, null,
                        (state) -> Logger.recordOutput("Intake/RollerSysIdState", state.toString())),
                new SysIdRoutine.Mechanism(
                        (voltage) -> io.setRollerVoltage(voltage.in(Volts)), null, this));

        LoggedMechanismRoot2d root = mechanism.getRoot("intake", 1.5, 1.0);
        armLigament = root.append(
                new LoggedMechanismLigament2d("arm", 0.2919984, 0, 6, new Color8Bit(Color.kOrange)));
        rollerLigament = armLigament.append(
                new LoggedMechanismLigament2d("roller", 0.69596, 90, 4, new Color8Bit(Color.kGreen)));
    }

    /** Deploy the intake and run rollers inward. */
    public Command intake() {
        return Commands.parallel(
                deploy(),
                runRollers(Constants.intakeVelocity))
                .withName("Intake");
    }

    /** Stow the intake and stop rollers. */
    public Command stow() {
        return runOnce(() -> {
            io.setDeployTarget(Constants.stowedAngle);
            io.stopRollers();
        }).withName("Stow");
    }

    /** Deploy the intake and run rollers outward to eject. */
    public Command eject() {
        return Commands.parallel(
                deploy(),
                runRollers(Constants.ejectVelocity))
                .withName("Eject");
    }

    /** Flip the intake to deployed position. */
    public Command deploy() {
        return runOnce(() -> io.setDeployTarget(Constants.deployedAngle)).withName("Deploy");
    }

    /** Run rollers at the given velocity until interrupted. */
    public Command runRollers(AngularVelocity velocity) {
        return runEnd(
                () -> io.setRollerVelocity(velocity),
                () -> io.stopRollers())
                .withName("RunRollers");
    }

    private boolean deployWithinSysIdLimits() {
        return inputs.deployPositionRads > Constants.deploySysIdMinRads
                && inputs.deployPositionRads < Constants.deploySysIdMaxRads;
    }

    public Command deploySysIdQuasistatic(SysIdRoutine.Direction direction) {
        return run(() -> io.setDeployVoltage(0.0))
                .withTimeout(1.0)
                .andThen(deploySysId.quasistatic(direction)
                        .until(() -> !deployWithinSysIdLimits()))
                .finallyDo(() -> io.setDeployVoltage(0.0));
    }

    public Command deploySysIdDynamic(SysIdRoutine.Direction direction) {
        return run(() -> io.setDeployVoltage(0.0))
                .withTimeout(1.0)
                .andThen(deploySysId.dynamic(direction)
                        .until(() -> !deployWithinSysIdLimits()))
                .finallyDo(() -> io.setDeployVoltage(0.0));
    }

    public Command rollerSysIdQuasistatic(SysIdRoutine.Direction direction) {
        return run(() -> io.setRollerVoltage(0.0))
                .withTimeout(1.0)
                .andThen(rollerSysId.quasistatic(direction));
    }

    public Command rollerSysIdDynamic(SysIdRoutine.Direction direction) {
        return run(() -> io.setRollerVoltage(0.0))
                .withTimeout(1.0)
                .andThen(rollerSysId.dynamic(direction));
    }

    public double getRollerRPMs() {
        return inputs.rollerRPMs;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);

        armLigament.setAngle(Math.toDegrees(inputs.deployPositionRads));
        rollerLigament.setColor(inputs.rollerRPMs > 0
                ? new Color8Bit(Color.kGreen)
                : inputs.rollerRPMs < 0
                        ? new Color8Bit(Color.kRed)
                        : new Color8Bit(Color.kGray));
    }
}
