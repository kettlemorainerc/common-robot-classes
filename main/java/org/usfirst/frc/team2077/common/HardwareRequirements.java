package org.usfirst.frc.team2077.common;

import com.kauailabs.navx.frc.*;
import edu.wpi.first.wpilibj2.command.*;
import org.usfirst.frc.team2077.common.drivetrain.*;
import org.usfirst.frc.team2077.common.sensor.*;

public abstract class HardwareRequirements<DriveModule, Chassis extends AbstractChassis<? super DriveModule>> {
    private final Subsystem heading;
    private final Subsystem position;
    private final AngleSensor angleSensor;
    private final AHRS navX;

    public HardwareRequirements(Subsystem heading, Subsystem position, AngleSensor angleSensor, AHRS navX) {
        this.heading = heading;
        this.position = position;
        this.angleSensor = angleSensor;
        this.navX = navX;
    }

    public Subsystem getHeading() {
        return heading;
    }

    public Subsystem getPosition() {
        return position;
    }

    public AngleSensor getAngleSensor() {
        return angleSensor;
    }

    public AHRS getNavX() {
        return navX;
    }

    public abstract Chassis getChassis();
    public abstract DriveModule getWheel(WheelPosition position);
}
