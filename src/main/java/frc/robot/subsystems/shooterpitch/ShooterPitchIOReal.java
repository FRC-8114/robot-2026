package frc.robot.subsystems.shooterpitch;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;

public class ShooterPitchIOReal implements ShooterPitchIO {
    private static class Constants {
        public static final int turretPitchMotorId = 31;

        public static final double turretPitchRatio = 1.2;

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
                .withSensorToMechanismRatio(turretPitchRatio);

        public static final TalonFXConfiguration pitchMotorCfg = new TalonFXConfiguration()
                .withSlot0(pitchMotorPIDs)
                .withMotionMagic(pitchMotionMagicConfigs)
                .withFeedback(pitchFeedbackConfigs);
    }

    private final TalonFX turretPitchMotor = new TalonFX(Constants.turretPitchMotorId);
    private final MotionMagicVoltage control = new MotionMagicVoltage(0);
    private final VoltageOut voltageControl = new VoltageOut(0);

    public ShooterPitchIOReal() {
        turretPitchMotor.getConfigurator().apply(Constants.pitchMotorCfg);
    }

    public void setTarget(Angle angle) {
        turretPitchMotor.setControl(control.withPosition(angle));
    }

    public void setVoltage(double volts) {
        turretPitchMotor.setControl(voltageControl.withOutput(volts));
    }

    public void updateInputs(ShooterPitchInputs inputs) {
        inputs.pitchPosition = turretPitchMotor.getPosition().getValue().in(Degrees);
        inputs.velocityRadsPerSec = turretPitchMotor.getVelocity().getValue().in(RadiansPerSecond);
        inputs.appliedVoltage = turretPitchMotor.getMotorVoltage().getValueAsDouble();
    }
}
