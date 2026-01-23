package frc.robot.subsystems.turretpivot;

import static edu.wpi.first.units.Units.Radians;
import static frc.robot.subsystems.turretpivot.TurretPivotConstants.*;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TurretPivot extends SubsystemBase {
    private TurretPivotIO io;

    TurretPivot(TurretPivotIO io) {
        this.io = io;
    }

    public Command pointTowardsFieldCentric(Supplier<Pose2d> robot_pose, Pose2d goal_pose) {
        Supplier<Angle> point_towards_goal = () -> {
            Pose2d turret_pose = robot_pose.get().transformBy(offset); // NOTE: the angle here is the robot's angle, not the turret's angle

            Angle diff = Radians.of(Math.atan2(
                goal_pose.getY() - turret_pose.getY(),
                goal_pose.getX() - turret_pose.getX()
            ));

            return io.getPosition().plus(diff);
        };

        return run(() -> io.followPosition(point_towards_goal));
    }
}
