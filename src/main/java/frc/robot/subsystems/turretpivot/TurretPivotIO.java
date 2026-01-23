package frc.robot.subsystems.turretpivot;

import java.util.function.Supplier;

import edu.wpi.first.units.measure.Angle;

public interface TurretPivotIO {
    abstract public Angle getPosition();

    abstract public void setPosition(Angle angle);
    abstract public void followPosition(Supplier<Angle> angle);
}
