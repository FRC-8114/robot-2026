package frc.robot.simulation;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;

import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooterpitch.ShooterPitch;
import frc.robot.subsystems.turret.Turret;
import frc.robot.util.FuelSim;

public class GamePieceTracker {
    private static final double INTAKE_DWELL = 0.3;
    private static final double HOPPER_DWELL = 0.4;
    private static final double TURRET_LANE_DWELL = 0.2;
    private static final double RPM_THRESHOLD = 200.0;
    private static final double LAUNCH_HEIGHT_METERS = 0.5;
    private static final double RPM_TO_MPS_FACTOR = 0.075 * 2.0 * Math.PI / 60.0;

    private final FuelSim fuelSim;
    private final Indexer indexer;
    private final Shooter shooter;
    private final ShooterPitch shooterPitch;
    private final Turret turret;
    private final Drive drive;

    private int intakeCount = 0;
    private int hopperCount = 0;
    private int turretLaneCount = 0;
    private int readyToFireCount = 0;
    private int shotCount = 0;

    private final Timer intakeTimer = new Timer();
    private final Timer hopperTimer = new Timer();
    private final Timer turretLaneTimer = new Timer();

    public GamePieceTracker(FuelSim fuelSim, Indexer indexer, Shooter shooter,
            ShooterPitch shooterPitch, Turret turret, Drive drive) {
        this.fuelSim = fuelSim;
        this.indexer = indexer;
        this.shooter = shooter;
        this.shooterPitch = shooterPitch;
        this.turret = turret;
        this.drive = drive;
    }

    public void update() {
        // Advance turret lane → ready to fire
        if (turretLaneCount > 0
                && indexer.getTurretLaneRPMs() > RPM_THRESHOLD
                && turretLaneTimer.hasElapsed(TURRET_LANE_DWELL)) {
            int transfer = turretLaneCount;
            turretLaneCount = 0;
            readyToFireCount += transfer;
            turretLaneTimer.stop();
        }

        // Advance hopper → turret lane
        if (hopperCount > 0
                && indexer.getHopperLanesRPMs() > RPM_THRESHOLD
                && hopperTimer.hasElapsed(HOPPER_DWELL)) {
            int transfer = hopperCount;
            hopperCount = 0;
            turretLaneCount += transfer;
            turretLaneTimer.restart();
        }

        // Advance intake → hopper
        if (intakeCount > 0 && intakeTimer.hasElapsed(INTAKE_DWELL)) {
            int transfer = intakeCount;
            intakeCount = 0;
            hopperCount += transfer;
            hopperTimer.restart();
        }

        Logger.recordOutput("GamePieceTracker/Intake", intakeCount);
        Logger.recordOutput("GamePieceTracker/Hopper", hopperCount);
        Logger.recordOutput("GamePieceTracker/TurretLane", turretLaneCount);
        Logger.recordOutput("GamePieceTracker/ReadyToFire", readyToFireCount);
        Logger.recordOutput("GamePieceTracker/Total", getPieceCount());
        Logger.recordOutput("GamePieceTracker/ShotCount", shotCount);
    }

    public int getPieceCount() {
        return intakeCount + hopperCount + turretLaneCount + readyToFireCount;
    }

    public boolean hasGamePiece() {
        return getPieceCount() > 0;
    }

    public boolean isReadyToFire() {
        return readyToFireCount > 0;
    }

    public boolean tryShoot() {
        if (readyToFireCount <= 0) return false;

        double mps = shooter.getAverageFlywheelRPMs() * RPM_TO_MPS_FACTOR;

        fuelSim.launchFuel(
                MetersPerSecond.of(mps),
                Radians.of(shooterPitch.getPitchPositionRads()),
                Radians.of(turret.getTurretPositionRads()),
                Meters.of(LAUNCH_HEIGHT_METERS));

        readyToFireCount--;
        shotCount++;
        return true;
    }

    public void onIntake() {
        if (intakeCount == 0) {
            intakeTimer.restart();
        }
        intakeCount++;
    }

    public void updateSim() {
        fuelSim.updateSim();
        update();
    }
}
