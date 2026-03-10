package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;


import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import frc.robot.RobotConstants;

public class TurretIOReal implements TurretIO {
    private static class Constants {
        public static final int pivotMotorID = 32;

        // The turret encoder CANCoder which has the 19T gear
        public static final int encoder19TID = 33;
        // the turret encoder which has the 21T gear
        public static final int encoder21TID = 34;

        private static final double encoder19TOffset = -0.06396484375;
        private static final double encoder21TOffset = -0.311279296875;

        public static final double ERROR_THRESHOLD = Math.toRadians(2);

        private static final MagnetSensorConfigs magnetConfig = new MagnetSensorConfigs()
                .withMagnetOffset(encoder19TOffset)
                .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
                .withAbsoluteSensorDiscontinuityPoint(1);

        private static final MagnetSensorConfigs encoder2MagnetConfigs = new MagnetSensorConfigs()
                .withMagnetOffset(encoder21TOffset)
                .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
                .withAbsoluteSensorDiscontinuityPoint(1);

        public static final CANcoderConfiguration encoder1Cfg = new CANcoderConfiguration()
                .withMagnetSensor(magnetConfig);

        public static final CANcoderConfiguration encoder2Cfg = new CANcoderConfiguration()
                .withMagnetSensor(encoder2MagnetConfigs);

        private static final Slot0Configs pivotMotorPIDs = new Slot0Configs()
                .withKS(1.1)
                .withKV(0.1034)
                .withKA(0)
                .withKP(9.78)
                .withKI(0.0)
                .withKD(0.0);

        private static final MotionMagicConfigs pivotMotionMagicConfigs = new MotionMagicConfigs()
                .withMotionMagicAcceleration(0.5)
                .withMotionMagicCruiseVelocity(1);

        private static final SoftwareLimitSwitchConfigs softwareLimits = new SoftwareLimitSwitchConfigs()
                .withForwardSoftLimitThreshold(Turret.Constants.MAX_ANGLE)
                .withForwardSoftLimitEnable(true)
                .withReverseSoftLimitThreshold(Turret.Constants.MIN_ANGLE)
                .withReverseSoftLimitEnable(true);

        public static final TalonFXConfiguration pivotMotorCfg = new TalonFXConfiguration()
                .withSlot0(pivotMotorPIDs)
                .withClosedLoopGeneral(new ClosedLoopGeneralConfigs().withContinuousWrap(false))
                .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive))
                .withMotionMagic(pivotMotionMagicConfigs)
                .withSoftwareLimitSwitch(softwareLimits)
                .withFeedback(new FeedbackConfigs()
                    .withSensorToMechanismRatio(10)); // 10 rotations of the motor for 1 rotation of the turret
    }

    private final TalonFX pivotMotor = new TalonFX(Constants.pivotMotorID, RobotConstants.canBus);
    private final CANcoder turret19TEncoder = new CANcoder(Constants.encoder19TID, RobotConstants.canBus);
    private final CANcoder turret21TEncoder = new CANcoder(Constants.encoder21TID, RobotConstants.canBus);
    private final MotionMagicVoltage control = new MotionMagicVoltage(0);
    private final VoltageOut voltageControl = new VoltageOut(0);

    public TurretIOReal() {
        turret19TEncoder.getConfigurator().apply(Constants.encoder1Cfg);
        turret21TEncoder.getConfigurator().apply(Constants.encoder2Cfg);
        pivotMotor.getConfigurator().apply(Constants.pivotMotorCfg);

        Angle initialAngle = getTurretAngle();

        reseedPosition(initialAngle);
    }

    // chinese remainder theorem is simple, actually
    private Angle getTurretAngle() {
        long teeth1 = Math.round(turret19TEncoder.getAbsolutePosition().getValueAsDouble() * 19.0) % 19l;
        long teeth2 = Math.round(turret21TEncoder.getAbsolutePosition().getValueAsDouble() * 21.0) % 21l;
        long coarse = (teeth1 * 21l * 10l + teeth2 * 19l * 10l) % 399l;

        double fraction = teeth2 % 1;
        double turretGearTeeth = coarse + fraction;

        return Radians.of(MathUtil.inputModulus(turretGearTeeth * ((2 * Math.PI) / 200.0), 0, Math.PI * 2));
    }

    private void reseedPosition(Angle angle) {
        pivotMotor.setPosition(angle);
    }

    public void setTarget(Angle angle) {
        pivotMotor.setControl(control.withPosition(angle));
    }

    public void setVoltage(double volts) {
        pivotMotor.setControl(voltageControl.withOutput(volts));
    }

    public void updateInputs(TurretIOInputs inputs) {
        double position = pivotMotor.getPosition().getValue().in(Radians);
        Angle positionCrt = getTurretAngle();

        inputs.turretMotorPosition = position;
        inputs.turretPositionCRT = positionCrt.in(Radians);

        if (Math.abs(MathUtil.angleModulus(position - positionCrt.in(Radians)))
            > Constants.ERROR_THRESHOLD) {
            // valid CRT but motor disagrees
            inputs.motorPositionErrorCounter += 1;
        } else {
            inputs.motorPositionErrorCounter = 0;
        }

        inputs.velocityRadsPerSec = pivotMotor.getVelocity().getValue().in(RadiansPerSecond);
        inputs.appliedVoltage = pivotMotor.getMotorVoltage().getValueAsDouble();

        // err
        if (Math.abs(inputs.motorPositionErrorCounter) > 5) {
            reseedPosition(positionCrt);
            inputs.motorPositionErrorCounter = 0;
        }
    }

}
