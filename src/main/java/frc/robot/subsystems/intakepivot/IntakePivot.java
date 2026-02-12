package frc.robot.subsystems.intakepivot;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.intakepivot.IntakePivotConstants.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class IntakePivot extends SubsystemBase {
    private final TalonFX pivotMotor = new TalonFX(pivotMotorID, new CANBus("canivore"));

    private final MotionMagicVoltage control = new MotionMagicVoltage(0).withEnableFOC(true);
    private final VoltageOut control_volts = new VoltageOut(0);

    public IntakePivot() {
        pivotMotor.getConfigurator().apply(pivotMotorCfg);
    }

    public Trigger atAngle(Angle angle) {
        return new Trigger(() -> pivotMotor.getPosition().getValue().isNear(angle, angleTolerance));
    }

    // Ex. setAngle(IntakePivotConstants.INIT_ANGLE)
    public Command setAngle(Angle angle) {
        return run(() -> pivotMotor.setControl(control.withPosition(angle)));
    }

    // SysId Routines
    private SysIdRoutine routine = new SysIdRoutine(
        new SysIdRoutine.Config(
            Volts.per(Second).of(1),
            Volts.of(4),
            null,
            state -> SignalLogger.writeString("state", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> pivotMotor.setControl(control_volts.withOutput(output)),
            null,
            this
        )
    );
    public Command runSysId() {
        return Commands.sequence(
            routine.dynamic(SysIdRoutine.Direction.kForward).until(atAngle(DEPLOY_ANGLE)),
            routine.dynamic(SysIdRoutine.Direction.kReverse).until(atAngle(INIT_ANGLE)),
            routine.quasistatic(SysIdRoutine.Direction.kForward).until(atAngle(DEPLOY_ANGLE)),
            routine.quasistatic(SysIdRoutine.Direction.kReverse).until(atAngle(DEPLOY_ANGLE))
        );
    }
}
