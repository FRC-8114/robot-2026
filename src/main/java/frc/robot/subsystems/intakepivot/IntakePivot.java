package frc.robot.subsystems.intakepivot;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.subsystems.intakepivot.IntakePivotConstants.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

public class IntakePivot extends SubsystemBase {
    private final TalonFX pivotMotor = new TalonFX(pivotMotorID, new CANBus("canivore"));

    private PositionVoltage pos = new PositionVoltage(0);

    IntakePivot() {
        pivotMotor.getConfigurator().apply(pivotMotorCfg);
    }

    // Ex. setAngle(IntakePivotConstants.INIT_ANGLE)
    public Command setAngle(Angle angle) {
        return run(() -> pivotMotor.setControl(pos.withPosition(angle)));
    }
}
