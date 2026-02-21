package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;

public interface TurretIO {
    @AutoLog
    public static class TurretIOInputs {
        public boolean hasValidCRT = false;
        public double turretMotorPosition = 0;
        public double turretPositionCRT = 0;
        public double motorPositionErrorCounter = 0;
        public double velocityRadsPerSec = 0;
        public double appliedVoltage = 0;
    }

    void updateInputs(TurretIOInputs inputs);

    void setTarget(Angle angle);

    void setVoltage(double volts);
}
