package frc.robot.subsystems.turret.pivot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.Optional;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;

public class TurretPivotIOReal implements TurretPivotIO {
    private static class Constants {
        public static final int pivotMotorID = 10;

        // The turret encoder CANCoder which has the 19T gear
        public static final int turretEncoder1ID = 11;
        // the turret encoder which has the 21T gear
        public static final int turretEncoder2ID = 12;

        public static final double MAX_ROTATION = 200.0;

        private static final double encoder1Offset = 0.0;
        private static final double encoder2Offset = 0.0;

        public static final double ERROR_THRESHOLD = Math.toRadians(2);

        private static final MagnetSensorConfigs magnetConfig = new MagnetSensorConfigs()
                .withMagnetOffset(encoder1Offset)
                .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
                .withAbsoluteSensorDiscontinuityPoint(1);

        private static final MagnetSensorConfigs encoder2MagnetConfigs = new MagnetSensorConfigs()
                .withMagnetOffset(encoder2Offset)
                .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
                .withAbsoluteSensorDiscontinuityPoint(1);

        public static final CANcoderConfiguration encoder1Cfg = new CANcoderConfiguration()
                .withMagnetSensor(magnetConfig);

        public static final CANcoderConfiguration encoder2Cfg = new CANcoderConfiguration()
                .withMagnetSensor(encoder2MagnetConfigs);

        private static final Slot0Configs pivotMotorPIDs = new Slot0Configs()
                .withKS(0.001)
                .withKV(0.032)
                .withKA(0.01)
                .withKP(9.78)
                .withKI(0.0)
                .withKD(0.0);

        private static final MotionMagicConfigs pivotMotionMagicConfigs = new MotionMagicConfigs()
                .withMotionMagicAcceleration(1) // 1 rotation of the turret per second squared
                .withMotionMagicCruiseVelocity(3); // 3 rotations of the turret per second

        public static final TalonFXConfiguration pivotMotorCfg = new TalonFXConfiguration()
                .withSlot0(pivotMotorPIDs)
                .withMotionMagic(pivotMotionMagicConfigs)
                .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(20));// 20 rotations of the motor for 1
                                                                                    // rotation of the turret
    }

    private final TalonFX pivotMotor = new TalonFX(Constants.pivotMotorID);
    private final CANcoder turretEncoder1 = new CANcoder(Constants.turretEncoder1ID);
    private final CANcoder turretEncoder2 = new CANcoder(Constants.turretEncoder2ID);
    private final MotionMagicVoltage control = new MotionMagicVoltage(0);

    public TurretPivotIOReal() {
        turretEncoder1.getConfigurator().apply(Constants.encoder1Cfg);
        turretEncoder2.getConfigurator().apply(Constants.encoder2Cfg);
        pivotMotor.getConfigurator().apply(Constants.pivotMotorCfg);

        Optional<Angle> initialAngle = getTurretAngle();

        if (initialAngle.isEmpty()) {
            System.err.println("Turret encoders are messed up");
        }

        reseedPosition(initialAngle.orElse(Angle.ofRelativeUnits(0, Degrees)));
    }

    // chinese remainder theorem is simple, actually
    private Optional<Angle> getTurretAngle() {
        double teeth1 = turretEncoder1.getAbsolutePosition().getValueAsDouble() * 19.0;
        double teeth2 = turretEncoder2.getAbsolutePosition().getValueAsDouble() * 21.0;
        double turretGearTeeth = (teeth1 * 21.0 * 10.0 + teeth2 * 19.0 * 10.0) % 399.0;

        if (turretGearTeeth > Constants.MAX_ROTATION) {
            return Optional.empty();
        }

        return Optional.of(Angle.ofRelativeUnits(turretGearTeeth * (360.0 / 200.0), Degrees));
    }

    private void reseedPosition(Angle angle) {
        pivotMotor.setPosition(angle);
    }

    @Override
    public void setTarget(Angle angle) {
        pivotMotor.setControl(control.withPosition(angle));
    }

    @Override
    public void updateInputs(TurretPivotIOInputs inputs) {
        double position = pivotMotor.getPosition().getValue().in(Radians);
        Optional<Angle> positionCrt = getTurretAngle();

        inputs.hasValidCRT = positionCrt.isPresent();
        inputs.turretMotorPosition = position;
        inputs.turretPositionCRT = positionCrt.orElse(Angle.ofRelativeUnits(0, Radians)).in(Radians);
        
        if (positionCrt.isPresent() && Math
                .abs(MathUtil.angleModulus(position - positionCrt.get().in(Radians))) > Constants.ERROR_THRESHOLD) {
            // valid CRT but motor disagrees
            inputs.motorPositionErrorCounter += 1;
        } else {
            inputs.motorPositionErrorCounter = 0;
        }

        inputs.velocityRadsPerSec = pivotMotor.getVelocity().getValue().in(RadiansPerSecond);
        inputs.appliedVoltage = pivotMotor.getMotorVoltage().getValueAsDouble();

        // 4 degrees is the threshold for when to reseed
        if (Math.abs(inputs.motorPositionErrorCounter) > 4) {
            reseedPosition(positionCrt.get());
            inputs.motorPositionErrorCounter = 0;
        }
    }

}
