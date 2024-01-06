/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FRC Team 2077. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams.            */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2077.common.drivetrain;

import org.usfirst.team2077.common.Clock;
import org.usfirst.team2077.common.WheelPosition;
import org.usfirst.frc.team2077.common.drivetrain.MecanumMath.*;
import org.usfirst.frc.team2077.common.math.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.usfirst.frc.team2077.common.math.AccelerationLimits.*;

import java.util.*;
import java.util.function.*;

import static org.usfirst.frc.team2077.common.drivetrain.MecanumMath.VelocityDirection.*;

public abstract class AbstractChassis<DriveModule> extends SubsystemBase implements DriveChassisIF {

    private static <T> EnumMap<VelocityDirection, T> defaultedDirectionMap(T defaultValue) {
        EnumMap<VelocityDirection, T> newMap = new EnumMap<>(VelocityDirection.class);
        for(VelocityDirection d : VelocityDirection.values()) newMap.put(d, defaultValue);
        return newMap;
    }

    public final EnumMap<WheelPosition, DriveModule> driveModule;
    protected final double wheelbase;
    protected final double trackWidth;
    protected final double wheelRadius;
    protected final Supplier<Double> getSeconds;

    protected double maximumSpeed;
    protected double maximumRotation;
    protected double minimumSpeed;
    protected double minimumRotation;

    // Ideally accel/decel values are set just below wheelspin or skidding to a stop.
    // Optimal values are highly dependent on wheel/surface traction and somewhat on
    // weight distribution.
    // For safety err on the low side for acceleration, high for deceleration.
    protected AccelerationLimits accelerationLimits = new AccelerationLimits(false, .5, .5, this);

    protected double lastUpdateTime = 0;
    protected double timeSinceLastUpdate = 0;

    protected final Position positionSet = new Position(); // Continuously updated by integrating velocity setpoints (velocitySet_).
    protected final Position positionMeasured = new Position(); // Continuously updated by integrating measured velocities (velocityMeasured_).

    protected Map<VelocityDirection, Double> velocity = defaultedDirectionMap(0d); // target velocity for next period
    protected Map<VelocityDirection, Double> targetVelocity = defaultedDirectionMap(0d); // Actual velocity target
    protected Map<VelocityDirection, Double> velocitySet = defaultedDirectionMap(0d); // The previous period's set
    protected Map<VelocityDirection, Double> velocityMeasured = defaultedDirectionMap(0d); // The next period's target velocities from mecanum math

// NOTE: If you uncomment the debug stuff here don't forget to do the same to the commented set in periodic
// Debug flag gets set to true every Nth call to beginUpdate().
//    protected int debugFrequency_ = 100; // 50 / s
//    private long debugCounter_ = 0; // internal counter
//    public boolean debug_ = false;

    public AbstractChassis(EnumMap<WheelPosition, DriveModule> driveModule, double wheelbase, double trackWidth, double wheelRadius, Supplier<Double> getSeconds) {
        this.driveModule = driveModule;
        this.wheelbase = wheelbase;
        this.trackWidth = trackWidth;
        this.wheelRadius = wheelRadius;
        this.getSeconds = getSeconds;
    }

    public AbstractChassis(EnumMap<WheelPosition, DriveModule> driveModule, double wheelbase, double trackWidth, double wheelRadius) {
        this(driveModule, wheelbase, trackWidth, wheelRadius, Clock::getSeconds);
    }

    @Override
    public void periodic() {
        double now = getSeconds.get();
        timeSinceLastUpdate = now - lastUpdateTime;
        lastUpdateTime = now;

        updatePosition();
        limitVelocity(NORTH, maximumSpeed);
        limitVelocity(EAST, maximumSpeed);
        limitVelocity(ROTATION, maximumRotation);
        updateDriveModules();
    }

    protected void limitVelocity(VelocityDirection direction, double max) {
        double currentVelocity = this.velocity.get(direction);
        double targetVelocity = this.targetVelocity.get(direction);

        boolean accelerating = Math.abs(targetVelocity) >= Math.abs(currentVelocity) && Math.signum(targetVelocity) == Math.signum(currentVelocity);
        double deltaLimit = accelerationLimits.get(direction, accelerating ? Type.ACCELERATION : Type.DECELERATION) *
                            timeSinceLastUpdate; // always positive
        double deltaRequested = targetVelocity - currentVelocity;
        double delta = Math.min(deltaLimit, Math.abs(deltaRequested)) * Math.signum(deltaRequested);
        double v = currentVelocity + delta;
        double newDirectionVelocity = Math.max(-max, Math.min(max, v));

        velocity.put(direction, newDirectionVelocity);
    }

    /**
     * Perform any position calculations neccessary to account for
     * movement since last upodate.
     * Called by {@link #periodic()}.
     */
    protected abstract void updatePosition();

    /**
     * Update drive module setpoints.
     * Called by {@link #periodic()}.
     */
    protected abstract void updateDriveModules();

    @Override
    public EnumMap<VelocityDirection, Double> getVelocitySet() {
        return new EnumMap<>(targetVelocity);
    }

    @Override
    public EnumMap<VelocityDirection, Double> getVelocityCalculated() {
        return new EnumMap<>(velocity);
    }

    @Override
    public EnumMap<VelocityDirection, Double> getVelocityMeasured() {
        return new EnumMap<>(velocityMeasured);
    }

    @Override
    public EnumMap<VelocityDirection, Double> getMaximumVelocity() {
        EnumMap<VelocityDirection, Double> stuff = new EnumMap<>(VelocityDirection.class);

        stuff.put(NORTH, maximumSpeed);
        stuff.put(EAST, maximumSpeed);
        stuff.put(ROTATION, maximumRotation);

        return stuff;
    }

    @Override
    public EnumMap<VelocityDirection, Double> getMinimumVelocity() {
        EnumMap<VelocityDirection, Double> stuff = new EnumMap<>(VelocityDirection.class);

        stuff.put(NORTH, minimumSpeed);
        stuff.put(EAST, minimumSpeed);
        stuff.put(ROTATION, minimumRotation);

        return stuff;
    }

    @Override
    public void moveAbsolute(double north, double east, double heading) {
        throw new UnsupportedOperationException();
    }

    @Override
    public void moveAbsolute(double north, double east) {
        throw new UnsupportedOperationException();
    }

    @Override
    public void rotateAbsolute(double clockwise) {
        throw new UnsupportedOperationException();
    }

    @Override
    public void moveRelative(double north, double east, double heading) {
        throw new UnsupportedOperationException();
    }

    @Override
    public void moveRelative(double north, double east) {
        throw new UnsupportedOperationException();
    }

    @Override
    public void rotateRelative(double clockwise) {
        throw new UnsupportedOperationException();
    }

    @Override
    public void setPosition(double north, double east, double heading) {
        positionSet.set(north, east, heading);
        positionMeasured.set(north, east, heading);
    }

    @Override
    public Position getPosition() {
        return positionSet.copy();
    }

    @Override
    public void setVelocity(double north, double east, double clockwise) {
        setVelocity(north, east, clockwise, getAccelerationLimits());
    }
    @Override
    public void setVelocity(double north, double east) {
        setVelocity(north, east, getAccelerationLimits());
    }
    @Override
    public void setRotation(double clockwise) {
        setRotation(clockwise, getAccelerationLimits());
    }

    @Override
    public final void setVelocityPercent(double north, double east, double clockwise) {
        EnumMap<VelocityDirection, Double> max = getMaximumVelocity();
        setVelocity(north * max.get(NORTH), east * max.get(EAST), clockwise * max.get(ROTATION));
    }

    @Override
    public final void setVelocityPercent(double north, double east) {
        EnumMap<VelocityDirection, Double> max = getMaximumVelocity();
        setVelocity(north * max.get(NORTH), east * max.get(EAST));
    }

    @Override
    public final void setRotationPercent(double clockwise) {
        setRotation(clockwise * getMaximumVelocity().get(ROTATION));
    }

    @Override
    public void halt() {
        double max = 10000;
        setVelocity(0, 0, 0, new AccelerationLimits(new double[][] {{max, max}, {max, max}, {max, max}}, this));
    }

    @Override
    public void setGLimits(double accelerationG, double decelerationG) {
        double eastAccelerationRatio = .7;
        accelerationLimits = new AccelerationLimits(false, accelerationG, decelerationG, this, new double[]{1, eastAccelerationRatio, 1});
    }

    @Override
    public AccelerationLimits getAccelerationLimits() {
        return accelerationLimits.getAdjustedAdjustments();
    }
}
