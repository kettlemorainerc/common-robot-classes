/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FRC Team 2077. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams.            */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2077.common.drivetrain;

import org.usfirst.frc.team2077.common.*;
import org.usfirst.frc.team2077.common.math.*;
import org.usfirst.frc.team2077.common.sensor.*;
import org.usfirst.frc.team2077.common.subsystem.CANLineSubsystem;

import java.util.AbstractMap.*;
import java.util.*;
import java.util.Map.*;
import java.util.function.*;

import static java.util.stream.Collectors.*;
import static org.usfirst.frc.team2077.common.VelocityDirection.*;

public class MecanumChassis extends AbstractChassis<DriveModuleIF> {
    private static final double WHEELBASE = 20.375; // inches
    private static final double TRACK_WIDTH = 25.5; // inches
    private static final double WHEEL_RADIUS = 4.0; // inches
    private static final double EAST_ADJUSTMENT = .65;

    private final MecanumMath mecanumMath;
    private final AngleSensor angleSensor;

    private static EnumMap<WheelPosition, DriveModuleIF> buildDriveModule(HardwareRequirements<?, ?> hardware) {
        EnumMap<WheelPosition, DriveModuleIF> driveModule = new EnumMap<>(WheelPosition.class);

        for (WheelPosition pos : WheelPosition.values()) {
            Object wheel = hardware.getWheel(pos);
            if (wheel instanceof DriveModuleIF) driveModule.put(pos, (DriveModuleIF) wheel);
            else if (wheel instanceof CANLineSubsystem)
                driveModule.put(pos, (DriveModuleIF) ((CANLineSubsystem<?>) wheel).motor);
            else throw new RuntimeException(String.format(
                        "Unexpected wheel type %s. Neither DriveModuleIF nor CANLineSubsystem?",
                        wheel.getClass()
                ));
        }

        return driveModule;
    }

    public MecanumChassis(HardwareRequirements<?, MecanumChassis> hardware) {
        this(hardware.getAngleSensor(), buildDriveModule(hardware), Clock::getSeconds);
    }

    MecanumChassis(AngleSensor angleSensor, Map<WheelPosition, DriveModuleIF> driveModule, Supplier<Double> getSeconds) {
        super(driveModule, WHEELBASE, TRACK_WIDTH, WHEEL_RADIUS, getSeconds);
        this.angleSensor = angleSensor;

        mecanumMath = new MecanumMath(wheelbase, trackWidth, wheelRadius, wheelRadius, 1, 180 / Math.PI);

        // north/south speed conversion from 0-1 range to DriveModule maximum (inches/second)
        maximumSpeed = this.driveModules.values()
                .stream()
                .map(DriveModuleIF::getMaximumSpeed)
                .min(Comparator.naturalOrder())
                .orElseThrow();

        // rotation speed conversion from 0-1 range to DriveModule maximum (degrees/second)
        maximumRotation = mecanumMath.velocityForTargets(MecanumMath.mapOf(
                WheelPosition.class,
                -maximumSpeed,
                -maximumSpeed,
                maximumSpeed,
                maximumSpeed
        )).get(ROTATION);

        // lowest chassis speeds supportable by the drive modules
        minimumSpeed = maximumSpeed * .1; // TODO: Test and configure.
        minimumRotation = maximumRotation * .1;

        AccelerationLimits a = getAccelerationLimits();
    }

    @Override
    public void setVelocity(double north, double east, double clockwise, AccelerationLimits accelerationLimits) {
        setVelocity(north, east, accelerationLimits);
        setRotation(clockwise, accelerationLimits);
    }

    @Override public void setVelocity(double north, double east, AccelerationLimits accelerationLimits) {
        targetVelocity.put(FORWARD, north);
        this.accelerationLimits.set(FORWARD, accelerationLimits.get(FORWARD));

        targetVelocity.put(STRAFE, east);
        this.accelerationLimits.set(STRAFE, accelerationLimits.get(STRAFE));
    }

    @Override public void setRotation(double clockwise, AccelerationLimits accelerationLimits) {
        targetVelocity.put(ROTATION, clockwise);
        this.accelerationLimits.set(ROTATION, accelerationLimits.get(ROTATION));
    }

    public void updateAccelerationLimits() {

    }

    @Override protected void updatePosition() {
        // chassis velocity from internal set point
        velocitySet = getVelocityCalculated();

        // chassis velocity from motor/wheel measurements
        Map<WheelPosition, Double> wheelVelocities = (
                driveModules.entrySet()
                        .stream()
                        .map(e -> new SimpleEntry<>(e.getKey(), e.getValue().getVelocity()))
                        .collect(toMap(
                                Entry::getKey,
                                Entry::getValue,
                                Math::min,
                                () -> new EnumMap<>(WheelPosition.class)
                        ))
        );
        velocityMeasured = mecanumMath.velocityForTargets(wheelVelocities);

        // TODO: E/W velocities are consistently lower than those calculated from wheel speeds.
        // TODO: Measure actual vs measured E/W distances and insert an adjustment factor here.
        velocitySet.compute(STRAFE, (k, v) -> v * EAST_ADJUSTMENT); // just a guess
        velocityMeasured.compute(STRAFE, (k, v) -> v * EAST_ADJUSTMENT); // just a guess

        // update position with motion since last update
        positionSet.moveRelative(
                velocitySet.get(FORWARD) * timeSinceLastUpdate,
                velocitySet.get(STRAFE) * timeSinceLastUpdate,
                velocitySet.get(ROTATION) * timeSinceLastUpdate
        );
        positionMeasured.moveRelative(
                velocityMeasured.get(FORWARD) * timeSinceLastUpdate,
                velocityMeasured.get(STRAFE) * timeSinceLastUpdate,
                velocityMeasured.get(ROTATION) * timeSinceLastUpdate
        );
        if (angleSensor != null) { // TODO: Confirm AngleSensor is actually reading. Handle bench testing.
            double[] pS = positionSet.get();
            double[] pM = positionMeasured.get();
            pS[2] = pM[2] = angleSensor.getAngle(); // TODO: conditional on gyro availability
            positionSet.set(pS[0], pS[1], pS[2]);
            positionMeasured.set(pM[0], pM[1], pM[2]);
        }
    }

    @Override protected void updateDriveModules() {
        Map<VelocityDirection, Double> botVelocity = getVelocityCalculated();

        // compute motor speeds
        Map<WheelPosition, Double> wheelSpeed = mecanumMath.targetsForVelocity(botVelocity);

        // scale all motors proportionally if any are out of range
        double max = (
                wheelSpeed.values()
                        .stream()
                        .map(vel -> Math.abs(vel) / maximumSpeed)
                        .max(Comparator.naturalOrder())
                        .map(val -> Math.max(1, val))
                        .orElseThrow()
        );

        for (WheelPosition position : WheelPosition.values()) {
            driveModules.get(position).setVelocity(
                    wheelSpeed.get(position) / max
            );
        }
    }

    @Override public String toString() {
        return String.format(
                "MecanumChassis\n\t" +
                        "Velocity: %s\n\t" +
                        "Set Velocity: %s\n\t" +
                        "MeasuredVelocity: %s\n\t" +
                        "Drive Modules: %s\n\t" +
                        "Set Position: %s\n\t" +
                        "Measured Position: %s\n",
                velocity,
                velocitySet,
                velocityMeasured,
                driveModules,
                positionSet,
                positionMeasured
        );
    }
}
