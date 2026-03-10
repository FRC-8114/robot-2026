package frc.robot.subsystems.intakerollers;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class IntakeRollers extends SubsystemBase {
    private static final AngularVelocity intakeVelocity = RPM.of(2400);

    private final IntakeRollersIO io;
    private final IntakeRollersInputsAutoLogged inputs = new IntakeRollersInputsAutoLogged();

    private SysIdRoutine sysId;

    public IntakeRollers(IntakeRollersIO io) {
        this.io = io;

        sysId = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null, null, null,
                        (state) -> Logger.recordOutput("IntakeRollers/SysIdState", state.toString())),
                new SysIdRoutine.Mechanism(
                        (voltage) -> io.runVolts(voltage), null, this));
    }

    public Command intake() {
        return runEnd(
            () -> io.setVelocity(intakeVelocity),
            () -> io.setVelocity(RPM.of(0))
        )
            .withName("Intake Rollers");
    }

    public AngularVelocity getVelocity() {
        return RPM.of(inputs.velocityRPM);
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysId.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysId.dynamic(direction)
            .withTimeout(Seconds.of(5));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("IntakeRollers", inputs);
    }
}
