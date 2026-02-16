package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.indexer.IndexerIO.IndexerInputs;

public class Indexer extends SubsystemBase {
    private static class Constants {
        static final double reverseFrequency = 3.0;
        static final double reverseBurstTime = 0.2;

        static final AngularVelocity hopperLaneVelocity = RPM.of(1000);
        static final AngularVelocity turretLaneVelocity = RPM.of(1000);

    };

    private final IndexerIO io;
    private final IndexerInputs inputs = new IndexerInputs();

    public Indexer(IndexerIO io) {
        this.io = io;

        setDefaultCommand(periodicReverse());
    }

    public Command waitUntilHopperLaneVelocity(AngularVelocity velocity) {
        double inRPM = velocity.in(RPM);

        return Commands.waitUntil(() -> Math.abs(inputs.hopperLanesRPMs - inRPM) <= 1); // within 1 RPM
    }

    public Command waitUntilTurretLaneVelocity(AngularVelocity velocity) {
        double inRPM = velocity.in(RPM);

        return Commands.waitUntil(() -> Math.abs(inputs.turretLaneRPMs - inRPM) <= 1); // within 1 RPM
    }

    public Command runHopperLanes() {
        return runEnd(() -> io.setHopperLaneVelocity(Constants.hopperLaneVelocity),
                () -> io.setHopperLaneVelocity(RPM.zero()));
    }

    public Command runTurretLanes() {
        return runEnd(() -> io.setTurretLaneVelocity(Constants.turretLaneVelocity),
                () -> io.setTurretLaneVelocity(RPM.zero()));
    }

    public Command reverse() {
        return runEnd(() -> io.setHopperReverse(true), () -> io.setHopperReverse(false));
    }

    public Command periodicReverse() {
        return Commands.repeatingSequence(Commands.waitSeconds(Constants.reverseFrequency),
                reverse().withTimeout(Constants.reverseBurstTime));
    }

    public double getHopperLanesRPMs() {
        return inputs.hopperLanesRPMs;
    }

    public double getTurretLaneRPMs() {
        return inputs.turretLaneRPMs;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }
}
