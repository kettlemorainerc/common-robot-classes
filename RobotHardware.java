package org.usfirst.frc.team2077.common;

import com.kauailabs.navx.frc.*;
import edu.wpi.first.wpilibj2.command.*;
import org.usfirst.frc.team2077.common.drivetrain.*;
import org.usfirst.frc.team2077.common.sensors.*;
import org.usfirst.frc.team2077.common.subsystems.*;

public interface RobotHardware<WheelDriveModule> {
    Subsystem getHeading();
    Subsystem getPosition();
    AbstractChassis getChassis();
    default AngleSensor getAngleSensor() {return null;}
    AHRS getNavX();
    WheelDriveModule getWheel(MecanumMath.WheelPosition position);
}
