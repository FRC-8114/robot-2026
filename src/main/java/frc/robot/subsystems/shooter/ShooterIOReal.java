package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.units.measure.AngularVelocity;

public class ShooterIOReal implements ShooterIO {
    private static class Constants {
        static final int leftFlywheelMotorId = 35;
        static final int rightFlywheelMotorId = 36;

        static final Slot0Configs flywheelSlot0 = new Slot0Configs()
                .withKS(0.001)
                .withKV(0.032)
                .withKA(0.01)
                .withKP(9.78)
                .withKI(0.0)
                .withKD(0.0);

        static final CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(80)
                .withSupplyCurrentLimitEnable(true)
                .withStatorCurrentLimit(120)
                .withStatorCurrentLimitEnable(true);

        static final TalonFXConfiguration flywheelMotorConfig = new TalonFXConfiguration()
                .withSlot0(flywheelSlot0)
                .withCurrentLimits(currentLimits);
    }

    private final TalonFX leftFlywheel = new TalonFX(Constants.leftFlywheelMotorId);
    private final TalonFX rightFlywheel = new TalonFX(Constants.rightFlywheelMotorId);

    private final VelocityTorqueCurrentFOC velocityControl = new VelocityTorqueCurrentFOC(0).withSlot(0);
    private final VoltageOut voltageControl = new VoltageOut(0);

    public ShooterIOReal() {
        leftFlywheel.getConfigurator().apply(Constants.flywheelMotorConfig);
        rightFlywheel.getConfigurator().apply(Constants.flywheelMotorConfig);

        rightFlywheel.setControl(new Follower(Constants.leftFlywheelMotorId, MotorAlignmentValue.Opposed));
    }

    @Override
    public void setFlywheelVelocity(AngularVelocity velocity) {
        leftFlywheel.setControl(velocityControl.withVelocity(velocity));
    }

    @Override
    public void setVoltage(double volts) {
        leftFlywheel.setControl(voltageControl.withOutput(volts));
    }

    @Override
    public void stopFlywheels() {
        leftFlywheel.stopMotor();
        rightFlywheel.stopMotor();
    }

    @Override
    public void updateInputs(ShooterInputs inputs) {
        inputs.leftFlywheelRPMs = leftFlywheel.getVelocity().getValue().in(RPM);
        inputs.leftCurrentAmps = leftFlywheel.getTorqueCurrent().getValueAsDouble();

        inputs.rightFlywheelRPMs = rightFlywheel.getVelocity().getValue().in(RPM);
        inputs.rightCurrentAmps = rightFlywheel.getTorqueCurrent().getValueAsDouble();
    }
}
