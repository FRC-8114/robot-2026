package frc.robot.subsystems.indexer;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

public class IndexerIOSim implements IndexerIO {
    public void runVolts(Voltage volts) {}

    public void setVelocity(AngularVelocity velocity) {}

    public void stopMotor() {}

    public void updateInputs(IndexerInputs inputs) {}
}
