package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.units.measure.Voltage;

public class ClimberIOReal implements ClimberIO {
    private static class Constants {
        public static final double drumDiameter = 0;
        public static final int climbMotorID = 0;

        private static final double gearRatio = 0;
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
    }

    private static final TalonFX climbMotor = new TalonFX(Constants.climbMotorID, new CANBus("canivore"));

    private static final MotionMagicVoltage control = new MotionMagicVoltage(0).withEnableFOC(true);
    private static final VoltageOut voltageOut = new VoltageOut(0);

    public ClimberIOReal() {
        climbMotor.getConfigurator().apply(Constants.climbMotorCfg);
    }

    private static double translateHeightToRotations(double goalHeight) {
        return goalHeight / (Math.PI * Constants.drumDiameter);
    }

    public void setVoltage(Voltage volts) {
        climbMotor.setControl(voltageOut.withOutput(volts));
    }

    public void setTargetHeight(double heightMeters) {
        climbMotor.setControl(control.withPosition(translateHeightToRotations(heightMeters)));
    }

    public void updateInputs(ClimberIOInputs inputs) {
        inputs.heightMeters = climbMotor.getPosition().getValueAsDouble() * (Math.PI * Constants.drumDiameter);
        inputs.velocityRPM = climbMotor.getVelocity().getValue().in(RPM);
        inputs.currentAmps = climbMotor.getSupplyCurrent().getValue().in(Amps);
    }
}
