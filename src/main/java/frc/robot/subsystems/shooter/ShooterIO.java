package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.AngularVelocity;

public interface ShooterIO {
    @AutoLog
    public static class ShooterInputs {
        public double leftFlywheelRPMs = 0;
        public double rightFlywheelRPMs = 0;
        public double leftCurrentAmps = 0;
        public double rightCurrentAmps = 0;
    }

    void setFlywheelVelocity(AngularVelocity velocity);

    void setVoltage(double volts);

    void stopFlywheels();

    void updateInputs(ShooterInputs inputs);
}
