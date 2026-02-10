package frc.robot.subsystems.intakepivot;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.units.measure.Angle;

public class IntakePivotConstants {
    public static final int pivotMotorID = 0;

    public static final Angle INIT_ANGLE = Degrees.of(0);
    public static final Angle DEPLOY_ANGLE = Degrees.of(0);

    private static final Slot0Configs pivotMotorPIDs = new Slot0Configs()
        .withGravityType(GravityTypeValue.Arm_Cosine)
        .withKP(0)
        .withKI(0)
        .withKD(0)
        .withKS(0)
        .withKG(0);

    public static final TalonFXConfiguration pivotMotorCfg = new TalonFXConfiguration()
        .withSlot0(pivotMotorPIDs);
}
