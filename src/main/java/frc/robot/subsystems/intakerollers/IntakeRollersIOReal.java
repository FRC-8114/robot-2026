package frc.robot.subsystems.intakerollers;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.RobotConstants;

public class IntakeRollersIOReal implements IntakeRollersIO {
    private static final int motorID = 52;

    private static final double gearRatio = 1.0;

    static final Slot0Configs pidConfig = new Slot0Configs()
        .withKS(1.7081)
        .withKV(0.099828)
        .withKA(0.017114)
        .withKP(0.15938)
        .withKD(0.0);

    static final MotionMagicConfigs mmConfig = new MotionMagicConfigs()
        .withMotionMagicCruiseVelocity(2400)
        .withMotionMagicExpo_kV(0.12)
        .withMotionMagicExpo_kA(0.1);

    static final TalonFXConfiguration rollerMotorCfg = new TalonFXConfiguration()
        .withSlot0(pidConfig)
        .withMotionMagic(mmConfig)
        .withFeedback(new FeedbackConfigs()
            .withSensorToMechanismRatio(gearRatio));
    
    private final TalonFX rollerMotor = new TalonFX(motorID, RobotConstants.canBus);

    private static final MotionMagicVelocityVoltage control = new MotionMagicVelocityVoltage(0);
    private static final VoltageOut controlVoltage = new VoltageOut(0);

    public void setVelocity(AngularVelocity velocity) {
        rollerMotor.setControl(control.withVelocity(velocity));
    }

    public void runVolts(Voltage volts) {
        rollerMotor.setControl(controlVoltage.withOutput(volts));
    }

    @Override
    public void updateInputs(IntakeRollersInputs inputs) {
        inputs.velocityRPM = rollerMotor.getVelocity().getValue().in(RPM);
        inputs.appliedVoltageVolts = rollerMotor.getMotorVoltage().getValue().in(Volts);
        inputs.rollerPositionRads = rollerMotor.getPosition().getValue().in(Radians);
    }
}
