package frc.robot.subsystems.turret.pitch;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;

public interface TurretPitchIO {
    @AutoLog
    public static class TurretPitchInputs {
        public double pitchPosition = 0;
        public double velocityRadsPerSec = 0;
        public double appliedVoltage = 0;
    }

    abstract void setTarget(Angle angle);

    abstract void updateInputs(TurretPitchInputs inputs);
}
