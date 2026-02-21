package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.RPM;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
    private static class Constants {
        static final double reverseFrequency = 3.0;
        static final double reverseBurstTime = 0.2;

        static final AngularVelocity hopperLaneVelocity = RPM.of(250);
        static final AngularVelocity turretLaneVelocity = RPM.of(1500);

        static final AngularVelocity turretLaneVelocityTolerance = RPM.of(100);
    };

    private final IndexerIO io;
    private final IndexerInputsAutoLogged inputs = new IndexerInputsAutoLogged();

    public Indexer(IndexerIO io) {
        this.io = io;

        setDefaultCommand(periodicReverse());
    }

    public boolean isTurretLaneAtSpeed() {
        return RPM.of(inputs.turretLaneRPMs).isNear(Constants.turretLaneVelocity,
                Constants.turretLaneVelocityTolerance);
    }

    public Command runHopperLanes() {
        return runEnd(() -> io.setHopperLaneVelocity(Constants.hopperLaneVelocity),
                () -> io.setHopperLaneVelocity(RPM.zero()));
    }

    public Command runTurretLanes() {
        return runEnd(() -> io.setTurretLaneVelocity(Constants.turretLaneVelocity),
                () -> io.setTurretLaneVelocity(RPM.zero()));
    }

    public Command feed() {
        return runEnd(
                () -> {
                    io.setHopperLaneVelocity(Constants.hopperLaneVelocity);
                    io.setTurretLaneVelocity(Constants.turretLaneVelocity);
                },
                () -> {
                    io.setHopperLaneVelocity(RPM.zero());
                    io.setTurretLaneVelocity(RPM.zero());
                });
    }

    public Command prepareAndFeedWhen(BooleanSupplier shouldFeed) {
        return runEnd(
                () -> {
                    io.setTurretLaneVelocity(Constants.turretLaneVelocity);
                    if (shouldFeed.getAsBoolean()) {
                        io.setHopperLaneVelocity(Constants.hopperLaneVelocity);
                    } else {
                        io.setHopperLaneVelocity(RPM.zero());
                    }
                },
                () -> {
                    io.setTurretLaneVelocity(RPM.zero());
                    io.setHopperLaneVelocity(RPM.zero());
                });
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
        Logger.processInputs("Indexer", inputs);
    }
}
