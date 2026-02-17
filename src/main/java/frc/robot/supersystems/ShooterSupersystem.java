package frc.robot.supersystems;

import java.util.function.BooleanSupplier;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooterpitch.ShooterPitch;
import frc.robot.subsystems.turret.Turret;

public class ShooterSupersystem {
    private final Turret turretPivot;
    private final ShooterPitch turretPitch;
    private final Shooter shooter;
    private final Indexer indexer;

    public ShooterSupersystem(Turret turretPivot, ShooterPitch turretPitch, Shooter shooter, Indexer indexer) {
        this.turretPivot = turretPivot;
        this.turretPitch = turretPitch;
        this.shooter = shooter;
        this.indexer = indexer;
    }

    public boolean isReadyToFire(Angle turretAngle, Angle pitchAngle) {
        return turretPivot.isAtAngle(turretAngle)
                && turretPitch.isAtAngle(pitchAngle)
                && shooter.isAtSpeed();
    }

    public Command shootWhenReady(Angle turretAngle, Angle pitchAngle, BooleanSupplier fireTrigger) {
        BooleanSupplier shouldFeed = () ->
                fireTrigger.getAsBoolean() && isReadyToFire(turretAngle, pitchAngle);

        return Commands.parallel(
                turretPivot.setAngle(turretAngle),
                turretPitch.setAngle(pitchAngle),
                shooter.runFlywheels(),
                indexer.prepareAndFeedWhen(shouldFeed)
        );
    }
}
