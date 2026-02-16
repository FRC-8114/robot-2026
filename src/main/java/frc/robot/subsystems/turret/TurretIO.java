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

    abstract void updateInputs(TurretIOInputs inputs);

    abstract void setTarget(Angle angle);

    abstract void setVoltage(double volts);
}
