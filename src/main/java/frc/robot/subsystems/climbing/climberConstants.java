package frc.robot.subsystems.climbing;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;

public class climberConstants {
    public static final double deployHeight = 0;
    public static final int climb_motorID = 0;
    public static final double drumDiameter = 0;
    private static final double gearRatio = 0;
    private static final Slot0Configs climb_motorPIDs = new Slot0Configs()
        .withGravityType(GravityTypeValue.Elevator_Static)
        .withKP(0)
        .withKI(0)
        .withKD(0);
    private static final FeedbackConfigs fusedEncoderCfg = new FeedbackConfigs()
        .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
        .withRotorToSensorRatio(gearRatio);
    public static final TalonFXConfiguration climb_motorCfg = new TalonFXConfiguration()
        .withSlot0(climb_motorPIDs)
        .withFeedback(fusedEncoderCfg);
}
