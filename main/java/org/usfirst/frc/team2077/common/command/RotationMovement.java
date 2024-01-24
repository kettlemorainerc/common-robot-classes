package org.usfirst.frc.team2077.common.command;

import edu.wpi.first.wpilibj2.command.*;
import org.usfirst.frc.team2077.common.drivetrain.*;
import org.usfirst.frc.team2077.common.control.DriveStick;
import org.usfirst.frc.team2077.common.HardwareRequirements;

import java.util.Set;

public class RotationMovement implements Command {
    protected final Set<Subsystem> requirements;
    protected final DriveStick stick;
    protected final DriveChassisIF chassis;

    public RotationMovement(HardwareRequirements hardware, DriveStick stick) {
        requirements = Set.of(hardware.getHeading());

        this.stick = stick;
        this.chassis = hardware.getChassis();
    }

    @Override public Set<Subsystem> getRequirements() {
        return Set.of();
    }

    @Override public void execute() {
        chassis.setRotationPercent(stick.getRotation());
    }

    @Override public void end(boolean interrupted) {

    }

    @Override public boolean isFinished() {
        return false;
    }
}
