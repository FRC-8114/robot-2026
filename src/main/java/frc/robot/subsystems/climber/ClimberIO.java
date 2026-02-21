package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Voltage;

public interface ClimberIO {
    @AutoLog
    public static class ClimberIOInputs {
        public double heightMeters = 0.0;
        public double velocityRPM = 0.0;
        public double currentAmps = 0.0;
    }

    void setVoltage(Voltage volts);

    void setTargetHeight(double heightMeters);

    void updateInputs(ClimberIOInputs inputs);
}
