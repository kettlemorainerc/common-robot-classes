package org.usfirst.frc.team2077.common.subsystem;

import org.usfirst.frc.team2077.common.RectangularWheelPosition;

public interface SwervePosition {
    RectangularWheelPosition wheelPosition();

    int directionId();
    int magnitudeId();

    double maxRpm();
    double encoderOffset();

    double maxRotationRpm();
    double minPercent();
}
