package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.RobotConstants;

public class IntakeIOReal implements IntakeIO {
    private static class Constants {
        static final int deployMotorId = 51;

        static final double deployGearRatio = 11.8125;

        static final Slot0Configs deployPIDs = new Slot0Configs()
                .withKS(0.001)
                .withKV(0.032)
                .withKA(0.01)
                .withKP(9.78)
                .withKI(0.0)
                .withKD(0.0);
        static final MotionMagicConfigs deployMotionMagic = new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(10)
                .withMotionMagicAcceleration(45)
                .withMotionMagicJerk(0);

        static final TalonFXConfiguration deployMotorCfg = new TalonFXConfiguration()
                .withSlot0(deployPIDs)
                .withMotionMagic(deployMotionMagic)
                .withFeedback(new FeedbackConfigs()
                        .withSensorToMechanismRatio(deployGearRatio))
                .withMotorOutput(new MotorOutputConfigs()
                        .withNeutralMode(NeutralModeValue.Brake)
                        .withInverted(InvertedValue.CounterClockwise_Positive));

        static final int rollerMotorId = 52;

        static final double rollerGearRatio = 1.0;

        static final MotionMagicConfigs rollerMMConfigs = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(2400)
            .withMotionMagicExpo_kV(0.12)
            .withMotionMagicExpo_kA(0.1);
        static final Slot0Configs rollerPIDs = new Slot0Configs()
                .withKS(1.7081)
                .withKV(0.099828)
                .withKA(0.017114)
                .withKP(0.15938)
                .withKD(0.0);
        static final TalonFXConfiguration rollerMotorCfg = new TalonFXConfiguration()
                .withSlot0(rollerPIDs)
                .withCurrentLimits(new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(80))
                .withMotionMagic(rollerMMConfigs)
                .withFeedback(new FeedbackConfigs()
                        .withSensorToMechanismRatio(rollerGearRatio));
    }

    private final TalonFX deployMotor = new TalonFX(Constants.deployMotorId, RobotConstants.canBus);
    private final TalonFX rollerMotor = new TalonFX(Constants.rollerMotorId, RobotConstants.canBus);

    private final MotionMagicVoltage deployControl = new MotionMagicVoltage(0);
    private final VoltageOut deployVoltageControl = new VoltageOut(0);
    
    private final MotionMagicVelocityVoltage rollerVelocity = new MotionMagicVelocityVoltage(0).withEnableFOC(true);
    private final VoltageOut rollerVoltage = new VoltageOut(0);

    public IntakeIOReal() {
        deployMotor.getConfigurator().apply(Constants.deployMotorCfg);
        rollerMotor.getConfigurator().apply(Constants.rollerMotorCfg);

        deployMotor.setPosition(0);
    }

    public void setDeployTarget(Angle angle) {
        deployMotor.setControl(deployControl.withPosition(angle));
    }

    public void setDeployVoltage(double volts) {
        deployMotor.setControl(deployVoltageControl.withOutput(volts));
    }

    public void setRollerVelocity(AngularVelocity velocity) {
        rollerMotor.setControl(rollerVelocity.withVelocity(velocity));
    }

    public void setRollerVoltage(double volts) {
        rollerMotor.setControl(rollerVoltage.withOutput(volts));
    }

    public void stopRollers() {
        rollerMotor.stopMotor();
    }

    public void updateInputs(IntakeInputs inputs) {
        inputs.deployPositionRads = deployMotor.getPosition().getValue().in(Radians);
        inputs.deployVelocityRadsPerSec = deployMotor.getVelocity().getValue().in(RadiansPerSecond);
        inputs.deployAppliedVoltage = deployMotor.getMotorVoltage().getValueAsDouble();
        inputs.deployCurrentAmps = deployMotor.getTorqueCurrent().getValueAsDouble();

        inputs.rollerVelocityRPM = rollerMotor.getVelocity().getValue().in(RPM);
        inputs.rollerAppliedVoltage = rollerMotor.getMotorVoltage().getValue().in(Volts);
        inputs.rollerCurrentAmps = rollerMotor.getTorqueCurrent().getValueAsDouble();
        inputs.rollerPosition = rollerMotor.getPosition().getValueAsDouble();
    }
}
