package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public class IntakeIOReal implements IntakeIO {
    private static class Constants {
        static final int deployMotorId = 51;
        static final int rollerMotorId = 52;

        static final double deployGearRatio = 25.0;

        static final Slot0Configs deployPIDs = new Slot0Configs()
                .withKS(0.001)
                .withKV(0.032)
                .withKA(0.01)
                .withKP(9.78)
                .withKI(0.0)
                .withKD(0.0);

        static final MotionMagicConfigs deployMotionMagic = new MotionMagicConfigs()
                .withMotionMagicAcceleration(2)
                .withMotionMagicCruiseVelocity(4);

        static final TalonFXConfiguration deployMotorCfg = new TalonFXConfiguration()
                .withSlot0(deployPIDs)
                .withMotionMagic(deployMotionMagic)
                .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(deployGearRatio));

        static final Slot0Configs rollerPIDs = new Slot0Configs()
                .withKS(0.001)
                .withKV(0.032)
                .withKA(0.01)
                .withKP(9.78)
                .withKI(0.0)
                .withKD(0.0);

        static final TalonFXConfiguration rollerMotorCfg = new TalonFXConfiguration()
                .withSlot0(rollerPIDs);
    }

    private final TalonFX deployMotor = new TalonFX(Constants.deployMotorId);
    private final TalonFX rollerMotor = new TalonFX(Constants.rollerMotorId);

    private final MotionMagicVoltage deployControl = new MotionMagicVoltage(0);
    private final VelocityTorqueCurrentFOC rollerControl = new VelocityTorqueCurrentFOC(0).withSlot(0);
    private final VoltageOut deployVoltageControl = new VoltageOut(0);
    private final VoltageOut rollerVoltageControl = new VoltageOut(0);

    public IntakeIOReal() {
        deployMotor.getConfigurator().apply(Constants.deployMotorCfg);
        rollerMotor.getConfigurator().apply(Constants.rollerMotorCfg);
    }

    @Override
    public void setDeployTarget(Angle angle) {
        deployMotor.setControl(deployControl.withPosition(angle));
    }

    @Override
    public void setDeployVoltage(double volts) {
        deployMotor.setControl(deployVoltageControl.withOutput(volts));
    }

    @Override
    public void setRollerVelocity(AngularVelocity velocity) {
        rollerMotor.setControl(rollerControl.withVelocity(velocity));
    }

    @Override
    public void setRollerVoltage(double volts) {
        rollerMotor.setControl(rollerVoltageControl.withOutput(volts));
    }

    @Override
    public void stopRollers() {
        rollerMotor.stopMotor();
    }

    @Override
    public void updateInputs(IntakeInputs inputs) {
        inputs.deployPositionRads = deployMotor.getPosition().getValue().in(Radians);
        inputs.deployVelocityRadsPerSec = deployMotor.getVelocity().getValue().in(RadiansPerSecond);
        inputs.deployAppliedVoltage = deployMotor.getMotorVoltage().getValueAsDouble();
        inputs.deployCurrentAmps = deployMotor.getTorqueCurrent().getValueAsDouble();

        inputs.rollerRPMs = rollerMotor.getVelocity().getValue().in(RotationsPerSecond) * 60.0;
        inputs.rollerAppliedVoltage = rollerMotor.getMotorVoltage().getValueAsDouble();
        inputs.rollerCurrentAmps = rollerMotor.getTorqueCurrent().getValueAsDouble();
    }
}
