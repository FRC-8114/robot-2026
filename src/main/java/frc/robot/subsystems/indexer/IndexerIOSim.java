package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IndexerIOSim implements IndexerIO {
    private static final DCMotor MOTOR_GEARBOX = DCMotor.getKrakenX60Foc(1);
    private static final double GEAR_RATIO = 1.0;
    private static final double MOI = 0.001;

    private static final double KP = 0.1;
    private static final double REVERSE_VOLTS = -2.4;

    private final DCMotorSim turretLaneSim;
    private final DCMotorSim hopperLanesSim;

    private final PIDController turretLaneController = new PIDController(KP, 0, 0);
    private final PIDController hopperLanesController = new PIDController(KP, 0, 0);

    private boolean turretLaneClosedLoop = false;
    private boolean hopperLanesClosedLoop = false;
    private double turretLaneVolts = 0.0;
    private double hopperLanesVolts = 0.0;
    private boolean hopperReverse = false;

    public IndexerIOSim() {
        turretLaneSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(MOTOR_GEARBOX, MOI, GEAR_RATIO),
                MOTOR_GEARBOX);
        hopperLanesSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(MOTOR_GEARBOX, MOI, GEAR_RATIO),
                MOTOR_GEARBOX);
    }

    public void setTurretLaneVelocity(AngularVelocity velocity) {
        turretLaneClosedLoop = true;
        turretLaneController.setSetpoint(velocity.in(RPM) / 60.0 * 2.0 * Math.PI);
    }

    public void setHopperLaneVelocity(AngularVelocity velocity) {
        hopperLanesClosedLoop = true;
        hopperReverse = false;
        hopperLanesController.setSetpoint(velocity.in(RPM) / 60.0 * 2.0 * Math.PI);
    }

    public void setHopperReverse(boolean runReverse) {
        hopperReverse = runReverse;
        if (runReverse) {
            hopperLanesClosedLoop = false;
            hopperLanesVolts = REVERSE_VOLTS;
        } else {
            hopperLanesVolts = 0.0;
        }
    }

    public void updateInputs(IndexerInputs inputs) {
        if (turretLaneClosedLoop) {
            turretLaneVolts = turretLaneController.calculate(turretLaneSim.getAngularVelocityRadPerSec());
        } else {
            turretLaneController.reset();
        }

        if (hopperLanesClosedLoop) {
            hopperLanesVolts = hopperLanesController.calculate(hopperLanesSim.getAngularVelocityRadPerSec());
        } else if (!hopperReverse) {
            hopperLanesController.reset();
        }

        turretLaneSim.setInputVoltage(MathUtil.clamp(turretLaneVolts, -12.0, 12.0));
        hopperLanesSim.setInputVoltage(MathUtil.clamp(hopperLanesVolts, -12.0, 12.0));
        turretLaneSim.update(0.02);
        hopperLanesSim.update(0.02);

        inputs.turretLaneRPMs = turretLaneSim.getAngularVelocityRadPerSec() / (2.0 * Math.PI) * 60.0;
        inputs.turretLaneCurrentAmps = Math.abs(turretLaneSim.getCurrentDrawAmps());

        inputs.hopperLanesRPMs = hopperLanesSim.getAngularVelocityRadPerSec() / (2.0 * Math.PI) * 60.0;
        inputs.hopperLanesCurrentAmps = Math.abs(hopperLanesSim.getCurrentDrawAmps());
    }
}
