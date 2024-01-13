package org.usfirst.frc.team2077.common.subsystem;

import org.usfirst.frc.team2077.common.WheelPosition;

public interface SwervePosition {
    WheelPosition wheelPosition();

    int directionId();
    int magnitudeId();

    double maxRpm();
    double encoderOffset();

    double maxRotationRpm();
    double minPercent();
}
