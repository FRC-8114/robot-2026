package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import org.littletonrobotics.junction.Logger;

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
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.units.measure.Angle;
import frc.robot.RobotConstants;

public class TurretIOReal implements TurretIO {
    private static class Constants {
        public static final int pivotMotorID = 32;

        // The turret encoder CANCoder which has the 19T gear
        public static final int encoder19TID = 33;
        // the turret encoder which has the 21T gear
        public static final int encoder21TID = 34;

        private static final double encoder19TOffset = -0.0888671875;
        private static final double encoder21TOffset = -0.33447265625;

        public static final double RESEED_ERROR_THRESHOLD = Math.toRadians(4);
        public static final double STATIONARY_VELOCITY_THRESHOLD = Math.toRadians(5);
        public static final int RESEED_SAMPLE_COUNT = 10;
        public static final int CRT_MEDIAN_TAPS = 5;

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
    private final MedianFilter crtMedianFilter = new MedianFilter(Constants.CRT_MEDIAN_TAPS);

    private int reseedCounter = 0;

    public TurretIOReal() {
        turret19TEncoder.getConfigurator().apply(Constants.encoder1Cfg);
        turret21TEncoder.getConfigurator().apply(Constants.encoder2Cfg);
        pivotMotor.getConfigurator().apply(Constants.pivotMotorCfg);

        Angle initialAngle = getSeedAngle();

        reseedPosition(initialAngle);
    }

    // chinese remainder theorem is simple, actually
    private Angle getAbsoluteCrtAngle() {
        double rawTeeth1 = turret19TEncoder.getAbsolutePosition().getValueAsDouble() * 19.0;
        double rawTeeth2 = turret21TEncoder.getAbsolutePosition().getValueAsDouble() * 21.0;
        long teeth1 = Math.round(rawTeeth1) % 19l;
        long teeth2 = Math.round(rawTeeth2) % 21l;
        long coarse = (teeth1 * 21l * 10l + teeth2 * 19l * 10l) % 399l;

        double fraction = rawTeeth2 - Math.round(rawTeeth2);
        double turretGearTeeth = coarse + fraction;

        double angle0To2Pi = MathUtil.inputModulus(turretGearTeeth * ((2 * Math.PI) / 200.0), 0, Math.PI * 2);
        return Radians.of(angle0To2Pi);
    }

    private Angle seedAngleFromCRT(Angle crtAngle) {
        return Radians.of(MathUtil.angleModulus(crtAngle.in(Radians) + Math.PI));
    }

    private Angle getSeedAngle() {
        return seedAngleFromCRT(getAbsoluteCrtAngle());
    }

    private void reseedPosition(Angle angle) {
        pivotMotor.setPosition(angle);
    }

    private static boolean isWithinLimits(double angleRadians) {
        return angleRadians >= Turret.Constants.MIN_ANGLE.in(Radians)
                && angleRadians <= Turret.Constants.MAX_ANGLE.in(Radians);
    }

    public void setTarget(Angle angle) {
        Logger.recordOutput("Turret/Goal", Turret.clampAngle(angle));
        pivotMotor.setControl(control.withPosition(Turret.clampAngle(angle)));
    }

    public void setVoltage(double volts) {
        pivotMotor.setControl(voltageControl.withOutput(volts));
    }

    public void updateInputs(TurretIOInputs inputs) {
        double position = pivotMotor.getPosition().getValue().in(Radians);
        double velocity = pivotMotor.getVelocity().getValue().in(RadiansPerSecond);
        double crtPosition = getSeedAngle().in(Radians);
        double filteredCrtPosition = crtMedianFilter.calculate(crtPosition);
        boolean hasValidCrt = Double.isFinite(filteredCrtPosition);
        boolean crtInRange = hasValidCrt && isWithinLimits(filteredCrtPosition);
        double positionError = hasValidCrt ? MathUtil.angleModulus(filteredCrtPosition - position) : 0.0;

        inputs.turretMotorPosition = position;
        inputs.turretPositionCRT = filteredCrtPosition;
        inputs.hasValidCRT = hasValidCrt;
        inputs.velocityRadsPerSec = velocity;
        inputs.appliedVoltage = pivotMotor.getMotorVoltage().getValueAsDouble();

        boolean shouldReseed = crtInRange
                && Math.abs(velocity) <= Constants.STATIONARY_VELOCITY_THRESHOLD
                && Math.abs(positionError) >= Constants.RESEED_ERROR_THRESHOLD;

        reseedCounter = shouldReseed ? reseedCounter + 1 : 0;
        inputs.motorPositionErrorCounter = reseedCounter;

        if (reseedCounter >= Constants.RESEED_SAMPLE_COUNT) {
            reseedPosition(Radians.of(filteredCrtPosition));
            reseedCounter = 0;
            inputs.motorPositionErrorCounter = 0;
        }
    }

}
