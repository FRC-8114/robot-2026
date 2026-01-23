package frc.robot.subsystems.turretpivot;

import java.util.function.Supplier;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;

import static frc.robot.subsystems.turretpivot.TurretPivotConstants.*;

public class TurretPivotIOTalonFX implements TurretPivotIO {
    private final TalonFX pivotMotor = new TalonFX(pivotMotorID, new CANBus("canivore"));
    private final CANcoder pivotEncoder = new CANcoder(pivotEncoderID);

    private final MotionMagicVoltage control = new MotionMagicVoltage(0);

    TurretPivotIOTalonFX() {
        pivotMotor.getConfigurator().apply(pivotMotorCfg);
        pivotEncoder.getConfigurator().apply(pivotEncoderCfg);
    }

    public Angle getPosition() {
        return pivotMotor.getPosition().getValue();
    }

    public void setPosition(Angle angle) {
        pivotMotor.setControl(control.withPosition(angle));
    }
    public void followPosition(Supplier<Angle> angle) {
        pivotMotor.setControl(control.withPosition(angle.get()));
    }
}
