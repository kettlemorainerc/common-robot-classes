package org.usfirst.frc.team2077.common;

import com.kauailabs.navx.frc.*;
import edu.wpi.first.wpilibj2.command.*;
import org.usfirst.frc.team2077.common.drivetrain.*;
import org.usfirst.frc.team2077.common.sensor.*;

public interface RobotHardware<DriveModule, Chassis extends AbstractChassis<DriveModule>> {
    Subsystem getHeading();
    Subsystem getPosition();
    Chassis getChassis();
    AngleSensor getAngleSensor();
    AHRS getNavX();
    DriveModule getWheel(WheelPosition position);
}
