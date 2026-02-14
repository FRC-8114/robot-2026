package frc.robot.subsystems.turret.pivot;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;

public interface TurretPivotIO {
    @AutoLog
    public static class TurretPivotIOInputs {
        public boolean hasValidCRT = false;
        public double turretMotorPosition = 0;
        public double turretPositionCRT = 0;
        public double motorPositionErrorCounter = 0;
        public double velocityRadsPerSec = 0;
        public double appliedVoltage = 0;
    }

    abstract void updateInputs(TurretPivotIOInputs inputs);

    abstract void setTarget(Angle angle);
}
