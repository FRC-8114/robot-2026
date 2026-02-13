package frc.robot.subsystems.turret.pitch;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.units.measure.Angle;

public class TurretPitchIOReal implements TurretPitchIO {
    private static class Constants {
        public static final int turretPitchMotorId = 0;
        public static final int turretPitchEncoderId = 0;

        public static final double turretPitchRatio = 1.2;
        private static final double pitchEncoderOffset = 0.0;

        private static final MagnetSensorConfigs pitchEncoderMagnetConfigs = new MagnetSensorConfigs()
                .withMagnetOffset(pitchEncoderOffset)
                .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive);

        public static final CANcoderConfiguration pitchEncoderCfg = new CANcoderConfiguration()
                .withMagnetSensor(pitchEncoderMagnetConfigs);

        private static final Slot0Configs pitchMotorPIDs = new Slot0Configs()
                .withKS(0.001)
                .withKV(0.032)
                .withKA(0.01)
                .withKP(9.78)
                .withKI(0.0)
                .withKD(0.0);

        private static final MotionMagicConfigs pitchMotionMagicConfigs = new MotionMagicConfigs()
                .withMotionMagicAcceleration(1)
                .withMotionMagicCruiseVelocity(2);

        private static final FeedbackConfigs pitchFeedbackConfigs = new FeedbackConfigs()
                .withFeedbackRemoteSensorID(turretPitchEncoderId)
                .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
                .withSensorToMechanismRatio(turretPitchRatio);

        public static final TalonFXConfiguration pitchMotorCfg = new TalonFXConfiguration()
                .withSlot0(pitchMotorPIDs)
                .withMotionMagic(pitchMotionMagicConfigs)
                .withFeedback(pitchFeedbackConfigs);
    }

    private final CANcoder turretPitchEncoder = new CANcoder(Constants.turretPitchEncoderId);
    private final TalonFX turretPitchMotor = new TalonFX(Constants.turretPitchMotorId);
    private final MotionMagicVoltage control = new MotionMagicVoltage(0);

    public TurretPitchIOReal() {
        turretPitchEncoder.getConfigurator().apply(Constants.pitchEncoderCfg);
        turretPitchMotor.getConfigurator().apply(Constants.pitchMotorCfg);
    }

    @Override
    public void setTarget(Angle angle) {
        turretPitchMotor.setControl(control.withPosition(angle));
    }

    @Override
    public void updateInputs(TurretPitchInputs inputs) {
        inputs.pitchPosition = turretPitchMotor.getPosition().getValue().in(Degrees);
        inputs.velocityRadsPerSec = turretPitchMotor.getVelocity().getValue().in(RadiansPerSecond);
        inputs.appliedVoltage = turretPitchMotor.getMotorVoltage().getValueAsDouble();
    }
}
