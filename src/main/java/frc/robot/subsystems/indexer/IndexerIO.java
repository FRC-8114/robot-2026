package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

public interface IndexerIO {
    @AutoLog
    public static class IndexerInputs {
        public double turretLaneVelocity = 0;
        public double turretLanePosition = 0;
        public double turretLaneCurrentAmps = 0;
        public double turretLaneAppliedVoltage = 0;
    
        public double hopperLanesVelocity = 0;
        public double hopperLanePosition = 0;
        public double hopperLanesCurrentAmps = 0;
        public double hopperLanesAppliedVoltage = 0;
    }

    void setHopperLaneVoltage(Voltage volts);
    void setTurretLaneVoltage(Voltage volts);

    void setTurretLaneVelocity(AngularVelocity velocity);
    void setHopperLaneVelocity(AngularVelocity velocity);

    void stopTurretLane();
    void stopHopperLane();
 
    void setHopperReverse(boolean runReverse);

    void updateInputs(IndexerInputs inputs);
}
