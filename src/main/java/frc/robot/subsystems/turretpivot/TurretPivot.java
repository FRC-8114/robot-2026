package frc.robot.subsystems.turretpivot;

import static edu.wpi.first.units.Units.Radians;
import static frc.robot.subsystems.turretpivot.TurretPivotConstants.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TurretPivot extends SubsystemBase {
    private final TalonFX pivotMotor = new TalonFX(pivotMotorID, new CANBus("canivore"));
    private final CANcoder pivotEncoder = new CANcoder(pivotEncoderID);

    private final MotionMagicVoltage control = new MotionMagicVoltage(0);
    private final StatusSignal<Angle> pivotAngle = pivotMotor.getPosition();

    TurretPivot() {
        pivotMotor.getConfigurator().apply(pivotMotorCfg);
        pivotEncoder.getConfigurator().apply(pivotEncoderCfg);
    }

    private Command setPosition(Angle angle) {
        return run(() -> pivotMotor.setControl(control.withPosition(angle)));
    }

    private Command followPosition(Supplier<Angle> angle) {
        return run(() -> pivotMotor.setControl(control.withPosition(angle.get())));
    }

    public Command pointTowardsFieldCentric(Supplier<Angle> robot_angle, Supplier<Angle> field_centric_angle) {
        return followPosition(() ->
            robot_angle.get().minus(field_centric_angle.get())
        );
    }

    public Command pointTowardsFieldCentric(Supplier<Pose2d> robot_pose, Pose2d goal_pose) {
        Supplier<Angle> point_towards_goal = () -> {
            Pose2d turret_pose = robot_pose.get().transformBy(offset); // NOTE: the angle here is the robot's angle, not the turret's angle

            Angle diff = Radians.of(Math.atan2(
                goal_pose.getY() - turret_pose.getY(),
                goal_pose.getX() - turret_pose.getX()
            ));

            return pivotAngle.getValue().plus(diff);
        };

        return run(() -> followPosition(point_towards_goal));
    }
}
