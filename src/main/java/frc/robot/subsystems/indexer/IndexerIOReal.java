package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.units.measure.AngularVelocity;

public class IndexerIOReal implements IndexerIO {
    private static class Constants {
        static final int hopperLanesMotorId = 41;
        static final int turretLaneMotorId = 42;

        static final Slot0Configs turretLaneMotorSlot0 = new Slot0Configs()
                .withKS(0.001)
                .withKV(0.032)
                .withKA(0.01)
                .withKP(9.78)
                .withKI(0.0)
                .withKD(0.0);

        static final TalonFXConfiguration turretLaneMotorConfig = new TalonFXConfiguration()
                .withSlot0(turretLaneMotorSlot0);

        static final Slot0Configs hopperLaneMotorSlot0 = new Slot0Configs()
                .withKS(0.02)
                .withKV(0.032)
                .withKA(0.01)
                .withKP(9.78)
                .withKI(0.0)
                .withKD(0.0);

        static final TalonFXConfiguration hopperLaneMotorConfig = new TalonFXConfiguration()
                .withSlot0(hopperLaneMotorSlot0)
                .withMotorOutput(new MotorOutputConfigs()
                        .withInverted(InvertedValue.Clockwise_Positive));

    };

    private final TalonFX turretLaneMotor = new TalonFX(Constants.turretLaneMotorId);
    private final TalonFX hopperLanesMotor = new TalonFX(Constants.hopperLanesMotorId);

    private final VelocityTorqueCurrentFOC turretLaneControl = new VelocityTorqueCurrentFOC(0).withSlot(0);
    private final VelocityTorqueCurrentFOC hopperLaneControl = new VelocityTorqueCurrentFOC(0).withSlot(0);

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

    public void setHopperReverse(boolean runReverse) {
        if (runReverse) {
            hopperLanesMotor.set(-0.2);
        } else {
            hopperLanesMotor.stopMotor();
        }
    }

    public void updateInputs(IndexerInputs inputs) {
        inputs.hopperLanesRPMs = hopperLanesMotor.getVelocity().getValue().in(RPM);
        inputs.hopperLanesCurrentAmps = hopperLanesMotor.getTorqueCurrent().getValueAsDouble();

        inputs.turretLaneRPMs = turretLaneMotor.getVelocity().getValue().in(RPM);
        inputs.turretLaneCurrentAmps = turretLaneMotor.getTorqueCurrent().getValueAsDouble();
    }
}
