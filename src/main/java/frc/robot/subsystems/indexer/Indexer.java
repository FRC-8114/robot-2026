package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Indexer extends SubsystemBase {
    private static class Constants {
        static final double reverseFrequency = 3.0;
        static final double reverseBurstTime = 0.2;

        static final AngularVelocity turretLaneVelocityTolerance = RPM.of(100);
    };

    private final IndexerIO io;
    private final IndexerInputsAutoLogged inputs = new IndexerInputsAutoLogged();

    private final LoggedNetworkNumber hopperLaneVelocity = new LoggedNetworkNumber("Tuning/TuneHopperLaneVelocity",
            700);
    private final LoggedNetworkNumber turretLaneVelocity = new LoggedNetworkNumber("Tuning/TuneTurretLaneVelocity",
            1500);

    private final LoggedNetworkNumber hopperLaneVoltage = new LoggedNetworkNumber("Tuning/TuneHopperLaneVoltage", 9.82);
    private final LoggedNetworkNumber turretLaneVoltage = new LoggedNetworkNumber("Tuning/TuneTurretLaneVoltage", 5.52);

    private final SysIdRoutine sysIdHopper;
    private final SysIdRoutine sysIdTurret;

    public Indexer(IndexerIO io) {

        this.io = io;

        sysIdHopper = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null, null, null,
                        (state) -> Logger.recordOutput("HopperLane/SysIdState", state.toString())),
                new SysIdRoutine.Mechanism(
                        (voltage) -> io.setHopperLaneVoltage(voltage), null, this));

        sysIdTurret = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null, null, null,
                        (state) -> Logger.recordOutput("TurretLane/SysIdState", state.toString())),
                new SysIdRoutine.Mechanism(
                        (voltage) -> io.setTurretLaneVoltage(voltage), null, this));

        // setDefaultCommand(periodicReverse());
    }

    public boolean isTurretLaneAtSpeed() {
        return RotationsPerSecond.of(inputs.turretLaneVelocity).isNear(RPM.of(turretLaneVelocity.get()),
                Constants.turretLaneVelocityTolerance);
    }

    public Command runHopperLanes() {
        return runEnd(() -> io.setHopperLaneVelocity(RPM.of(hopperLaneVelocity.get())),
                () -> io.stopHopperLane());
    }

    public Command runTurretLanes() {
        return runEnd(() -> io.setTurretLaneVelocity(RPM.of(turretLaneVelocity.get())),
                () -> io.stopHopperLane());
    }

    public Command feed() {
        return runEnd(
                () -> {
                    io.setTurretLaneVelocity(RPM.of(turretLaneVelocity.get()));
                    io.setHopperLaneVelocity(RPM.of(hopperLaneVelocity.get()));
                },
                () -> {
                    io.stopHopperLane();
                    io.stopTurretLane();
                });
    }

    public Command feedVoltage() {
        return runEnd(() -> {
            io.setTurretLaneVoltage(Volts.of(turretLaneVoltage.get()));
            io.setHopperLaneVoltage(Volts.of(hopperLaneVoltage.get()));
        }, () -> {
            io.stopTurretLane();
            io.stopHopperLane();
        });
    }

    public Command prepareAndFeedWhen(BooleanSupplier shouldFeed) {
        return runEnd(
                () -> {
                    io.setTurretLaneVelocity(RPM.of(turretLaneVelocity.get()));
                    if (shouldFeed.getAsBoolean()) {
                        io.setHopperLaneVelocity(RPM.of(hopperLaneVelocity.get()));
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
        return inputs.hopperLanesVelocity / 60;
    }

    public double getTurretLaneRPMs() {
        return inputs.turretLaneVelocity / 60;
    }

    public Command sysIdQuasistaticTurret(SysIdRoutine.Direction direction) {
        return sysIdTurret.quasistatic(direction);
    }

    public Command sysIdDynamicTurret(SysIdRoutine.Direction direction) {
        return sysIdTurret.dynamic(direction);
    }

    public Command sysIdQuasistaticHopper(SysIdRoutine.Direction direction) {
        return sysIdHopper.quasistatic(direction);
    }

    public Command sysIdDynamicHopper(SysIdRoutine.Direction direction) {
        return sysIdHopper.dynamic(direction);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Indexer", inputs);
    }
}
