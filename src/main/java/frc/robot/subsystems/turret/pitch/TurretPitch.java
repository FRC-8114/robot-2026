package frc.robot.subsystems.turret.pitch;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TurretPitch extends SubsystemBase {
    private final TurretPitchIO pitchMotor;
    private final TurretPitchInputsAutoLogged inputs = new TurretPitchInputsAutoLogged();

    public TurretPitch(TurretPitchIO pitchMotor) {
        this.pitchMotor = pitchMotor;
    }

    @Override
    public void periodic() {
        pitchMotor.updateInputs(inputs);
    }

    public Command setAngle(Angle pitchAngle) {
        return run(() -> pitchMotor.setTarget(pitchAngle));
    }
}
