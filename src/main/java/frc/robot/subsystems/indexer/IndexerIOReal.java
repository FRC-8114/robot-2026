package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.RobotConstants;

public class IndexerIOReal implements IndexerIO {
    private static class Constants {
        static final int hopperLanesMotorId = 41;
        static final int turretLaneMotorId = 42;

        static final Slot0Configs turretLaneMotorSlot0 = new Slot0Configs()
                .withKS(1.0481)
                .withKV(0.13694)
                .withKA(0.010647)
                .withKP(0.55729)
                .withKI(0.0)
                .withKD(0.0);

        static final TalonFXConfiguration turretLaneMotorConfig = new TalonFXConfiguration()
                .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive))
                .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(1.875))
                .withSlot0(turretLaneMotorSlot0);

        static final Slot0Configs hopperLaneMotorSlot0 = new Slot0Configs()
                .withKS(0.56491)
                .withKV(0.097911)
                .withKA(0.0092569)
                .withKP(0.58604)
                .withKI(0.0)
                .withKD(0.0);

        static final TalonFXConfiguration hopperLaneMotorConfig = new TalonFXConfiguration()
            .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(14.4))
                .withSlot0(hopperLaneMotorSlot0);

    };

    private final TalonFX turretLaneMotor = new TalonFX(Constants.turretLaneMotorId, RobotConstants.canBus);
    private final TalonFX hopperLanesMotor = new TalonFX(Constants.hopperLanesMotorId, RobotConstants.canBus);

    private final VelocityVoltage turretLaneControl = new VelocityVoltage(0).withSlot(0);
    private final VelocityVoltage hopperLaneControl = new VelocityVoltage(0).withSlot(0);

    private final VoltageOut turretLaneControlVoltage = new VoltageOut(0);
    private final VoltageOut hopperLaneControlVoltage = new VoltageOut(0);

    public IndexerIOReal() {
        turretLaneMotor.getConfigurator().apply(Constants.turretLaneMotorConfig);
        hopperLanesMotor.getConfigurator().apply(Constants.hopperLaneMotorConfig);
    }

    public void setTurretLaneVelocity(AngularVelocity velocity) {
        turretLaneMotor.setControl(turretLaneControl.withVelocity(velocity));
    }

    public void setHopperLaneVelocity(AngularVelocity velocity) {
        hopperLanesMotor.setControl(hopperLaneControl.withVelocity(velocity));
    }

    public void stopHopperLane() {
        hopperLanesMotor.setControl(new VoltageOut(0));
    }

    public void stopTurretLane() {
        turretLaneMotor.setControl(new VoltageOut(0));
    }

    public void setHopperReverse(boolean runReverse) {
        if (runReverse) {
            hopperLanesMotor.set(-0.2);
        } else {
            hopperLanesMotor.stopMotor();
        }
    }

    public void updateInputs(IndexerInputs inputs) {
        inputs.hopperLanesVelocity = hopperLanesMotor.getVelocity().getValueAsDouble();
        inputs.hopperLanesCurrentAmps = hopperLanesMotor.getTorqueCurrent().getValueAsDouble();
        inputs.hopperLanesAppliedVoltage = hopperLanesMotor.getMotorVoltage().getValueAsDouble();
        inputs.hopperLanePosition = hopperLanesMotor.getPosition().getValueAsDouble();

        inputs.turretLaneVelocity = turretLaneMotor.getVelocity().getValueAsDouble();
        inputs.turretLaneCurrentAmps = turretLaneMotor.getSupplyCurrent().getValueAsDouble();
        inputs.turretLaneAppliedVoltage = turretLaneMotor.getMotorVoltage().getValue().in(Volts);
        inputs.turretLanePosition = turretLaneMotor.getPosition().getValueAsDouble();
    }

    public void setHopperLaneVoltage(Voltage volts) {
        hopperLanesMotor.setControl(hopperLaneControlVoltage.withOutput(volts));
    }

    public void setTurretLaneVoltage(Voltage volts) {
        turretLaneMotor.setControl(turretLaneControlVoltage.withOutput(volts));
    }
}
