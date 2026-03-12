package frc.robot.subsystems.turretloader;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

public interface TurretLoaderIO {
    @AutoLog
    public static class TurretLoaderInputs {
        public double motorPositionRads = 0;
        public double velocityRPM = 0;
        public double appliedVoltageVolts = 0;
    }

    void runVolts(Voltage volts);
    void setVelocity(AngularVelocity velocity);

    void stopMotor();

    void updateInputs(TurretLoaderInputs inputs);
}
