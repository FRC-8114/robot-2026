package frc.robot.subsystems.turretloader;

import static edu.wpi.first.units.Units.RPM;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class TurretLoader extends SubsystemBase {
    private static final AngularVelocity velocityTolerance = RPM.of(100);
    private static final AngularVelocity turretLoaderVelocity = RPM.of(2000);

    private final TurretLoaderIO io;
    private final TurretLoaderInputsAutoLogged inputs = new TurretLoaderInputsAutoLogged();

    private SysIdRoutine sysId;

    private final LoggedNetworkNumber tuneTurretLoaderVelocity = new LoggedNetworkNumber("Tuning/TurretLoaderVelocityRPM");

    public TurretLoader(TurretLoaderIO io) {
        this.io = io;

        sysId = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null, null, null,
                        (state) -> Logger.recordOutput("TurretLane/SysIdState", state.toString())),
                new SysIdRoutine.Mechanism(
                        (voltage) -> io.runVolts(voltage), null, this));
    }

    public final Trigger atSpeed = new Trigger(() -> RPM.of(inputs.velocityRPM).isNear(turretLoaderVelocity, velocityTolerance));

    public Command feed() {
        return runEnd(
            () -> io.setVelocity(turretLoaderVelocity),
            () -> io.stopMotor()
        );
    }

    public Command feedTunable() {
        return runEnd(
            () -> io.setVelocity(RPM.of(tuneTurretLoaderVelocity.get())),
            () -> io.stopMotor()
        );
    }

    public AngularVelocity getVelocity() {
        return RPM.of(inputs.velocityRPM);
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysId.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysId.dynamic(direction);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("TurretLoader", inputs);
    }
}
