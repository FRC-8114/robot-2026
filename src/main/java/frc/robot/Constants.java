package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

public class Constants {
    public enum RobotMode {
        REAL,
        SIMULATION,
        REPLAY
    };

    public static final RobotMode ROBOT_MODE = RobotBase.isReal() ? RobotMode.REAL : RobotMode.SIMULATION;
}
