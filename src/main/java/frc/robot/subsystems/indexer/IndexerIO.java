package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.AngularVelocity;

public interface IndexerIO {
    @AutoLog
    public static class IndexerInputs {
        public double turretLaneRPMs = 0;
        public double turretLaneCurrentAmps = 0;
        public double hopperLanesRPMs = 0;
        public double hopperLanesCurrentAmps = 0;
    }

    void setTurretLaneVelocity(AngularVelocity velocity);

    void setHopperLaneVelocity(AngularVelocity velocity);

    void setHopperReverse(boolean runReverse);

    void updateInputs(IndexerInputs inputs);
}
