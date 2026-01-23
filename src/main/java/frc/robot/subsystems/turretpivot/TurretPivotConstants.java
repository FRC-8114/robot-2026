package frc.robot.subsystems.turretpivot;

import static edu.wpi.first.units.Units.Inches;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.measure.Distance;

public final class TurretPivotConstants {
    private static final int idPrefix = 30;

    private static final Distance xOffset = Inches.of(0); // negative is backwards, positive is forwards
    private static final Distance yOffset = Inches.of(0); // positive is left, negative is right
    public static final Transform2d offset = new Transform2d(xOffset, yOffset, new Rotation2d(0));

    public static final int pivotEncoderID = idPrefix + 1;
    private static final double pivotEncoderOffset = 0;
    public static final MagnetSensorConfigs pivotEncoderCfg = new MagnetSensorConfigs()
        .withMagnetOffset(pivotEncoderOffset)
        .withSensorDirection(SensorDirectionValue.Clockwise_Positive);

    public static final int pivotMotorID = idPrefix;
    private static final double gearRatio = 0.0;

    private static final Slot0Configs pivotMotorPIDs = new Slot0Configs()
        .withGravityType(GravityTypeValue.Elevator_Static)
        .withKP(0)
        .withKI(0)
        .withKD(0);
    private static final FeedbackConfigs fusedEncoderCfg = new FeedbackConfigs()
        .withFeedbackRemoteSensorID(pivotEncoderID)
        .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
        .withRotorToSensorRatio(gearRatio);
    public static final TalonFXConfiguration pivotMotorCfg = new TalonFXConfiguration()
        .withSlot0(pivotMotorPIDs)
        .withFeedback(fusedEncoderCfg);

}
