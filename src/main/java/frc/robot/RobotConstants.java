package frc.robot;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.generated.TunerConstants;

public class RobotConstants {
    public enum RobotMode {
        REAL,
        SIMULATION,
        REPLAY
    };

    public static final RobotMode robotMode = RobotBase.isReal() ? RobotMode.REAL : RobotMode.SIMULATION;

    public static final CANBus canBus = TunerConstants.kCANBus;
}
