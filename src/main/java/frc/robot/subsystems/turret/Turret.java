package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Radians;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.turret.pitch.TurretPitch;
import frc.robot.subsystems.turret.pivot.TurretPivot;

public class Turret extends SubsystemBase {
    private static class Constants {
        public final static Transform2d turretOffset = new Transform2d();
    }

    private final TurretPivot pivot;
    private final TurretPitch pitch;

    public Turret(TurretPivot pivot, TurretPitch pitch) {
        this.pivot = pivot;
        this.pitch = pitch;
    }

    // TODO: exact logic for handling different shoot circumstances (cross field, into hub, etc)

    // retained temporarily, this isnt that useful here
    private Command pivotToFieldCentric(Supplier<Pose2d> robotPose, Pose2d goalPose) {
        Supplier<Angle> pointTowardGoal = () -> {
            Pose2d turretPose = robotPose.get().transformBy(Constants.turretOffset);

            Angle angleToGoal = Radians.of(Math.atan2(
                    goalPose.getY() - turretPose.getY(),
                    goalPose.getX() - turretPose.getX()));

            return Radians
                    .of(MathUtil.angleModulus(angleToGoal.minus(turretPose.getRotation().getMeasure()).in(Radians)));
        };

        return pivot.followAngle(pointTowardGoal);
    }
}
