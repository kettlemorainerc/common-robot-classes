package org.usfirst.frc.team2077.common.drivetrain;

import org.usfirst.frc.team2077.common.WheelPosition;

public interface DrivePosition {
    double gearRatio();
    double radius();
    double maxRpm();
    int motorId();
    boolean inverse();
    boolean shouldSetPid();

    double P();
    double I();
    double D();

    WheelPosition getWheelPosition();
}
