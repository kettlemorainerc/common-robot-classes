package org.usfirst.frc.team2077.common;

import com.kauailabs.navx.frc.*;
import edu.wpi.first.wpilibj2.command.*;
import org.usfirst.frc.team2077.common.drivetrain.*;
import org.usfirst.frc.team2077.common.sensor.*;

import java.util.EnumMap;
import java.util.Map;

public abstract class HardwareRequirements<DriveModule, Chassis extends AbstractChassis<DriveModule>> {
    private final Subsystem heading = makeHeading();
    private final Subsystem position = makePosition();
    private final Chassis chassis = makeChassis();
    private final Map<WheelPosition, DriveModule> modules = new EnumMap<>(WheelPosition.class);
    private final AngleSensor angleSensor = null;
    private final AHRS navX = makeNavX();

    public HardwareRequirements() {
        for(WheelPosition pos : WheelPosition.values()) {
            modules.put(pos, makeModule(pos));
        }
    }

    protected AngleSensor makeAngleSensor() {return null;}
    protected AHRS makeNavX() {return null;}
    protected Subsystem makeHeading() {return new Subsystem() {};}
    protected Subsystem makePosition() {return new Subsystem() {};}

    protected abstract Chassis makeChassis();
    protected abstract DriveModule makeModule(WheelPosition position);

    public DriveModule getWheel(WheelPosition pos) {
        return modules.get(pos);
    }

    public Subsystem getHeading() {
        return heading;
    }

    public Subsystem getPosition() {
        return position;
    }

    public Chassis getChassis() {
        return chassis;
    }

    public AngleSensor getAngleSensor() {
        return angleSensor;
    }

    public AHRS getNavX() {
        return navX;
    }
}
