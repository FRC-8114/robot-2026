package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.units.measure.Voltage;
import frc.robot.RobotConstants;

public class ClimberIOReal implements ClimberIO {
    private static class Constants {
        public static final int climbMotorID = 60;

        private static final double gearRatio = 36.0;
        private static final Slot0Configs climbMotorPIDs = new Slot0Configs()
                .withGravityType(GravityTypeValue.Elevator_Static)
                .withKP(0)
                .withKI(0)
                .withKD(0);
        private static final FeedbackConfigs fusedEncoderCfg = new FeedbackConfigs()
                .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
                .withRotorToSensorRatio(gearRatio);
        public static final TalonFXConfiguration climbMotorCfg = new TalonFXConfiguration()
                .withSlot0(climbMotorPIDs)
                .withFeedback(fusedEncoderCfg);
                // .withMotorOutput(new MotorOutputConfigs() .withInverted(InvertedValue.CounterClockwise_Positive)); // uhh gears I guess
    }

    private static final TalonFX climbMotor = new TalonFX(Constants.climbMotorID, RobotConstants.canBus);

    private static final PositionVoltage control = new PositionVoltage(0).withEnableFOC(true);
    private static final VoltageOut voltageOut = new VoltageOut(0);

    public ClimberIOReal() {
        climbMotor.getConfigurator().apply(Constants.climbMotorCfg);
    }

    public void setVoltage(Voltage volts) {
        climbMotor.setControl(voltageOut.withOutput(volts));
    }

    public void doRotations(double rotations) {
        climbMotor.setControl(control.withPosition(Rotations.of(rotations)));
    }

    public void updateInputs(ClimberIOInputs inputs) {
        inputs.rotations = climbMotor.getPosition().getValue().in(Rotations);
        inputs.velocityRPM = climbMotor.getVelocity().getValue().in(RPM);
        inputs.currentAmps = climbMotor.getSupplyCurrent().getValue().in(Amps);
    }
}
