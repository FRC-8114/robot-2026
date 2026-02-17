package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IntakeIOSim implements IntakeIO {
    private static final DCMotor DEPLOY_GEARBOX = DCMotor.getKrakenX60Foc(1);
    private static final double DEPLOY_GEAR_RATIO = 25.0;
    private static final double DEPLOY_MOI = 0.01;

    private static final DCMotor ROLLER_GEARBOX = DCMotor.getKrakenX60Foc(1);
    private static final double ROLLER_GEAR_RATIO = 1.0;
    private static final double ROLLER_MOI = 0.001;

    private static final double DEPLOY_KP = 5.0;
    private static final double ROLLER_KP = 0.1;

    private final DCMotorSim deploySim;
    private final DCMotorSim rollerSim;

    private final PIDController deployController = new PIDController(DEPLOY_KP, 0, 0);
    private final PIDController rollerController = new PIDController(ROLLER_KP, 0, 0);

    private boolean deployClosedLoop = false;
    private boolean rollerClosedLoop = false;
    private double deployAppliedVolts = 0.0;
    private double rollerAppliedVolts = 0.0;

    public IntakeIOSim() {
        deploySim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(DEPLOY_GEARBOX, DEPLOY_MOI, DEPLOY_GEAR_RATIO),
                DEPLOY_GEARBOX);
        rollerSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(ROLLER_GEARBOX, ROLLER_MOI, ROLLER_GEAR_RATIO),
                ROLLER_GEARBOX);
    }

    public void setDeployTarget(Angle angle) {
        deployClosedLoop = true;
        deployController.setSetpoint(angle.in(Radians));
    }

    public void setDeployVoltage(double volts) {
        deployClosedLoop = false;
        deployAppliedVolts = volts;
    }

    public void setRollerVelocity(AngularVelocity velocity) {
        rollerClosedLoop = true;
        rollerController.setSetpoint(velocity.in(RPM) / 60.0 * 2.0 * Math.PI); // convert to rad/s
    }

    public void setRollerVoltage(double volts) {
        rollerClosedLoop = false;
        rollerAppliedVolts = volts;
    }

    public void stopRollers() {
        rollerClosedLoop = false;
        rollerAppliedVolts = 0.0;
    }

    public void updateInputs(IntakeInputs inputs) {
        if (deployClosedLoop) {
            deployAppliedVolts = deployController.calculate(deploySim.getAngularPositionRad());
        } else {
            deployController.reset();
        }

        if (rollerClosedLoop) {
            rollerAppliedVolts = rollerController.calculate(rollerSim.getAngularVelocityRadPerSec());
        } else {
            rollerController.reset();
        }

        deploySim.setInputVoltage(MathUtil.clamp(deployAppliedVolts, -12.0, 12.0));
        rollerSim.setInputVoltage(MathUtil.clamp(rollerAppliedVolts, -12.0, 12.0));
        deploySim.update(0.02);
        rollerSim.update(0.02);

        inputs.deployPositionRads = deploySim.getAngularPositionRad();
        inputs.deployVelocityRadsPerSec = deploySim.getAngularVelocityRadPerSec();
        inputs.deployAppliedVoltage = deployAppliedVolts;
        inputs.deployCurrentAmps = Math.abs(deploySim.getCurrentDrawAmps());

        inputs.rollerRPMs = rollerSim.getAngularVelocityRadPerSec() / (2.0 * Math.PI) * 60.0;
        inputs.rollerAppliedVoltage = rollerAppliedVolts;
        inputs.rollerCurrentAmps = Math.abs(rollerSim.getCurrentDrawAmps());
    }
}
