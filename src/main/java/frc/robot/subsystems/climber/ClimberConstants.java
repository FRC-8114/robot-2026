package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;

public class ClimberConstants {
    // Presets
    public static final double DEPLOY_HEIGHT = 0;
    public static final double RETRACT_HEIGHT = 0;

    public static final int climbMotorID = 0;

    public static final double drumDiameter = 0;

    private static final double gearRatio = 0;
    private static final Slot0Configs climbMotorPIDs = new Slot0Configs()
        .withGravityType(GravityTypeValue.Elevator_Static)
        .withKP(0)
        .withKI(0)
        .withKD(0);
    private static final FeedbackConfigs fusedEncoderCfg = new FeedbackConfigs()
        .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
        .withRotorToSensorRatio(gearRatio);
    public static final TalonFXConfiguration climbMotorCfg = new TalonFXConfiguration()
        .withSlot0(climbMotorPIDs)
        .withFeedback(fusedEncoderCfg);
}
