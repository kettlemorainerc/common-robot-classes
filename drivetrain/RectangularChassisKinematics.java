package org.usfirst.frc.team2077.common.drivetrain;

import org.usfirst.frc.team2077.common.VelocityDirection;
import org.usfirst.frc.team2077.common.WheelPosition;

import java.util.EnumMap;

public abstract class RectangularChassisKinematics<InverseResult, ForwardArguments> implements ChassisKinematics<InverseResult, ForwardArguments> {
    /** The distance between front and rear wheels (length). Center to center */
    private double wheelbase;
    /** The distance between left and right wheels (width). Center to center */
    private double trackWidth;

    public RectangularChassisKinematics(double wheelbase, double trackWidth) {
        setWheelbase(wheelbase);
        setTrackWidth(trackWidth);
    }

    public void setWheelbase(double wheelbase) {
        this.wheelbase = wheelbase;
    }

    public double getWheelbase() {
        return wheelbase;
    }

    public void setTrackWidth(double trackWidth) {
        this.trackWidth = trackWidth;
    }

    public double getTrackWidth() {
        return trackWidth;
    }

    /** Inverse */
    public abstract EnumMap<WheelPosition, InverseResult> targetsForVelocity(EnumMap<VelocityDirection, Double> targetVelocities);
    /** Forward */
    public abstract EnumMap<VelocityDirection, Double> velocityForTargets(EnumMap<WheelPosition, ForwardArguments> targets);
}
