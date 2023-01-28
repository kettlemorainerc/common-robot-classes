package org.usfirst.frc.team2077.common.command;

import edu.wpi.first.wpilibj2.command.*;
import org.usfirst.frc.team2077.common.*;
import org.usfirst.frc.team2077.common.control.DriveStick;
import org.usfirst.frc.team2077.common.drivetrain.*;

public class RotationMovement extends CommandBase {
    protected final DriveStick stick;
    protected final DriveChassisIF chassis;

    public RotationMovement(HardwareRequirements hardware, DriveStick stick) {
        addRequirements(hardware.getHeading());

        this.stick = stick;
        this.chassis = hardware.getChassis();
    }

    @Override public void execute() {
        chassis.setRotationPercent(stick.getRotation());
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
