package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public interface IntakeIO {
    @AutoLog
    public static class IntakeInputs {
        public double deployPositionRads = 0;
        public double deployVelocityRadsPerSec = 0;
        public double deployAppliedVoltage = 0;
        public double deployCurrentAmps = 0;

        public double rollerRPMs = 0;
        public double rollerAppliedVoltage = 0;
        public double rollerCurrentAmps = 0;
    }

    void setDeployTarget(Angle angle);

    void setDeployVoltage(double volts);

    void setRollerVelocity(AngularVelocity velocity);

    void setRollerVoltage(double volts);

    void stopRollers();

    void updateInputs(IntakeInputs inputs);
}
