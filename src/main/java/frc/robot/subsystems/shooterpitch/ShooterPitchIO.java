package frc.robot.subsystems.shooterpitch;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;

public interface ShooterPitchIO {
    @AutoLog
    public static class ShooterPitchInputs {
        public double pitchPosition = 0;
        public double velocityRadsPerSec = 0;
        public double appliedVoltage = 0;
    }

    abstract void setTarget(Angle angle);

    abstract void setVoltage(double volts);

    abstract void updateInputs(ShooterPitchInputs inputs);
}
