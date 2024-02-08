package org.usfirst.frc.team2077.common.drivetrain;

public interface WheelProvider<WheelType extends DriveModuleIF, Identifier extends Enum<Identifier>> {
    WheelType getWheel(Identifier identifier);

}
