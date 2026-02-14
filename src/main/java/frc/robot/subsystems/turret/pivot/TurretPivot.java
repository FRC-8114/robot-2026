package frc.robot.subsystems.turret.pivot;

import java.util.function.Supplier;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TurretPivot extends SubsystemBase {
    private final TurretPivotIO pivotMotor;
    private final TurretPivotIOInputsAutoLogged inputs = new TurretPivotIOInputsAutoLogged();

    public TurretPivot(TurretPivotIO pivotMotor) {
        this.pivotMotor = pivotMotor;
    }

    @Override
    public void periodic() {
        pivotMotor.updateInputs(inputs);
    }

    public Command setAngle(Angle angle) {
        return run(() -> pivotMotor.setTarget(angle));
    }

    public Command followAngle(Supplier<Angle> angle) {
        return run(() -> pivotMotor.setTarget(angle.get()));
    }
}
