package org.usfirst.frc.team2077.common.drivetrain;

public interface ChassisProvider<ChassisType extends AbstractChassis> {
    ChassisType getChassis();
}
