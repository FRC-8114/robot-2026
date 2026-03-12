package frc.robot.subsystems.turretloader;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.RobotConstants;

public class TurretLoaderIOReal implements TurretLoaderIO {
    private static final int motorID = 41;

    static final Slot0Configs pidConfig = new Slot0Configs()
            .withKS(0.001)
            .withKV(0.032)
            .withKA(0.01)
            .withKP(9.78)
            .withKI(0.0)
            .withKD(0.0);

    static final TalonFXConfiguration motorConfig = new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive))
            .withFeedback(new FeedbackConfigs()
                .withSensorToMechanismRatio(1.875))
            .withSlot0(pidConfig);

    private final TalonFX laneMotor = new TalonFX(motorID, RobotConstants.canBus);

    private final VelocityVoltage control = new VelocityVoltage(0);
    private final VoltageOut controlVoltage = new VoltageOut(0);

    public void runVolts(Voltage volts) {
        laneMotor.setControl(controlVoltage.withOutput(volts));
    }

    public void setVelocity(AngularVelocity velocity) {
        laneMotor.setControl(control.withVelocity(velocity));
    }

    public void stopMotor() {
        laneMotor.stopMotor();
    }

    public void updateInputs(TurretLoaderInputs inputs) {
        inputs.appliedVoltageVolts = laneMotor.getMotorVoltage().getValue().in(Volts);
        inputs.motorPositionRads = laneMotor.getPosition().getValue().in(Radians);
        inputs.velocityRPM = laneMotor.getVelocity().getValue().in(RPM);
    }
}
