package frc.robot.subsystems.climbing;

import static frc.robot.subsystems.climbing.climberConstants.*;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class climb extends SubsystemBase {
    private static final TalonFX climb_motor = new TalonFX(climb_motorID, "canivore"); 
    private static final MotionMagicVoltage mm_voltage = new MotionMagicVoltage(0).withEnableFOC(true).withSlot(0);
    public Command setHeight(double goalHeight) {
        return run(() -> {
            climb_motor.setControl(mm_voltage.withPosition(translateHeightToRotations(goalHeight)));
        });
    }
    private static double translateHeightToRotations(double goalHeight) {
        return goalHeight / (Math.PI * drumDiameter);
    }

}
