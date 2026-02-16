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
    private enum State {
        EMPTY,
        INTAKE,
        HOPPER,
        TURRET_LANE,
        READY_TO_FIRE
    }

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

    private State state = State.EMPTY;
    private final Timer dwellTimer = new Timer();
    private int shotCount = 0;

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
        switch (state) {
            case INTAKE:
                if (dwellTimer.hasElapsed(INTAKE_DWELL)) {
                    state = State.HOPPER;
                    dwellTimer.restart();
                }
                break;
            case HOPPER:
                if (indexer.getHopperLanesRPMs() > RPM_THRESHOLD && dwellTimer.hasElapsed(HOPPER_DWELL)) {
                    state = State.TURRET_LANE;
                    dwellTimer.restart();
                }
                break;
            case TURRET_LANE:
                if (indexer.getTurretLaneRPMs() > RPM_THRESHOLD && dwellTimer.hasElapsed(TURRET_LANE_DWELL)) {
                    state = State.READY_TO_FIRE;
                    dwellTimer.stop();
                }
                break;
            case READY_TO_FIRE:
            case EMPTY:
                break;
        }

        Logger.recordOutput("GamePieceTracker/State", state.name());
        Logger.recordOutput("GamePieceTracker/HasGamePiece", hasGamePiece());
        Logger.recordOutput("GamePieceTracker/ShotCount", shotCount);
    }

    public boolean hasGamePiece() {
        return state != State.EMPTY;
    }

    public boolean isReadyToFire() {
        return state == State.READY_TO_FIRE;
    }

    public boolean tryShoot() {
        if (!isReadyToFire()) return false;

        double mps = shooter.getAverageFlywheelRPMs() * RPM_TO_MPS_FACTOR;

        fuelSim.launchFuel(
                MetersPerSecond.of(mps),
                Radians.of(shooterPitch.getPitchPositionRads()),
                Radians.of(turret.getTurretPositionRads()),
                Meters.of(LAUNCH_HEIGHT_METERS));

        state = State.EMPTY;
        shotCount++;
        return true;
    }

    public void onIntake() {
        if (state == State.EMPTY) {
            state = State.INTAKE;
            dwellTimer.restart();
        }
    }

    public void updateSim() {
        fuelSim.updateSim();
        update();
    }
}
