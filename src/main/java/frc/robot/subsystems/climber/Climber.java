package frc.robot.subsystems.climber;

import static frc.robot.subsystems.climber.ClimberConstants.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    private static final TalonFX climbMotor = new TalonFX(climbMotorID, new CANBus("canivore"));

    private static final MotionMagicVoltage control = new MotionMagicVoltage(0).withEnableFOC(true);

    Climber() {
        climbMotor.getConfigurator().apply(climbMotorCfg);
    }

    private static double translateHeightToRotations(double goalHeight) {
        return goalHeight / (Math.PI * drumDiameter);
    }

    public Command setHeight(double goalHeight) {
        return run(() -> {
            climbMotor.setControl(control.withPosition(translateHeightToRotations(goalHeight)));
        });
    }
}
