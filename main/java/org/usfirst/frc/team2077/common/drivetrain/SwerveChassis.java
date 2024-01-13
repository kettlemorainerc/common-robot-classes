package org.usfirst.frc.team2077.common.drivetrain;

import org.usfirst.frc.team2077.common.*;
import org.usfirst.frc.team2077.common.math.AccelerationLimits;
import org.usfirst.frc.team2077.common.sensor.AngleSensor;
import org.usfirst.frc.team2077.common.math.*;
import org.usfirst.frc.team2077.common.subsystem.*;

import java.util.*;
import java.util.function.Supplier;

import static org.usfirst.frc.team2077.common.VelocityDirection.*;

public class SwerveChassis extends AbstractChassis<SwerveMotor> {
    private final SwerveMath math;
    private final AngleSensor angleSensor;

    private static EnumMap<WheelPosition, SwerveMotor> buildDriveTrain(HardwareRequirements<SwerveMotor, SwerveChassis> hardware) {
        EnumMap<WheelPosition, SwerveMotor> map = new EnumMap<>(WheelPosition.class);

        for(WheelPosition p : WheelPosition.values()) {
            map.put(p, hardware.getWheel(p));
        }

        return map;
    }

    public SwerveChassis(
          HardwareRequirements<SwerveMotor, SwerveChassis> hardware,
          Supplier<Double> getSeconds,
          double wheelBase,
          double trackWidth,
          double wheelRadius
    ) {
        super(buildDriveTrain(hardware), wheelBase, trackWidth, wheelRadius, getSeconds);

        this.angleSensor = hardware.getAngleSensor();
        math = new SwerveMath(wheelBase, trackWidth);

        this.maximumSpeed = this.driveModules.values()
              .stream()
              .map(SwerveMotor::getMaximumSpeed)
              .min(Comparator.naturalOrder())
              .orElseThrow();
//        System.out.println();

        Map<WheelPosition, SwerveTargetValues> wheelTargets = math.swerveTargetsForBotVelocity(Map.of(
              FORWARD,
              0.0,
              STRAFE,
              0.0,
              ROTATION,
              1.0
        ));

        Map<WheelPosition, TestSwerve> map = new EnumMap<>(WheelPosition.class);
        wheelTargets.forEach((k, v) -> {
            map.put(k, new TestSwerve(v.getAngle(), v.getMagnitude() * maximumSpeed));
        });

        maximumRotation = math.botVelocityForSwerveStates(map)
              .get(ROTATION);

        this.minimumSpeed = this.maximumSpeed * 0.1;
    }

    public SwerveChassis(
          HardwareRequirements<SwerveMotor, SwerveChassis> hardware,
          double wheelBase,
          double trackWidth,
          double wheelRadius
    ) {
        this(hardware, Clock::getSeconds, wheelBase, trackWidth, wheelRadius);
    }

    @Override protected void updatePosition() {
        velocitySet = getVelocityCalculated();
        velocityMeasured = math.botVelocityForSwerveStates(driveModules);

        positionSet.moveRelative(velocitySet.get(FORWARD) * timeSinceLastUpdate,
              velocitySet.get(STRAFE) * timeSinceLastUpdate,
              velocitySet.get(ROTATION) * timeSinceLastUpdate
        );
        positionMeasured.moveRelative(velocityMeasured.get(FORWARD) * timeSinceLastUpdate,
              velocityMeasured.get(STRAFE) * timeSinceLastUpdate,
              velocityMeasured.get(ROTATION) * timeSinceLastUpdate
        );
    }

    public static WheelPosition LOGGED_POSITION = WheelPosition.FRONT_RIGHT;

    @Override protected void updateDriveModules() {
        System.out.printf("[target velocities=%s]%n", targetVelocity);
        Map<WheelPosition, SwerveTargetValues> wheelTargets = math.swerveTargetsForBotVelocity(
              targetVelocity,
              maximumSpeed,
              maximumRotation
        );


        wheelTargets.forEach((key, value) -> {

//            if(key != WheelPosition.FRONT_LEFT) return;

            SwerveMotor motor = this.driveModules.get(key);

            motor.setTargetAngle(value.getAngle());

            double targetVelocity = Math.abs(value.getMagnitude() * maximumSpeed);
            if(value.getMagnitude() > 0.0001) {
                targetVelocity = Math.max(targetVelocity, minimumSpeed);
            }
            motor.setVelocity(targetVelocity);
//            System.out.printf("[%s mag=%s]", key, value.getMagnitude());
        });

    }

    @Override public void setVelocity(
          double north,
          double east,
          double clockwise,
          AccelerationLimits accelerationLimits
    ) {
        setVelocity(north, east, accelerationLimits);
        setRotation(clockwise, accelerationLimits);
    }

    @Override public void setVelocity(
          double north,
          double east,
          AccelerationLimits accelerationLimits
    ) {
        targetVelocity.put(FORWARD, north);
        this.accelerationLimits.set(FORWARD, accelerationLimits.get(FORWARD));

        targetVelocity.put(STRAFE, east);
        this.accelerationLimits.set(STRAFE, accelerationLimits.get(STRAFE));
    }

    @Override public void setRotation(
          double clockwise,
          AccelerationLimits accelerationLimits
    ) {
        targetVelocity.put(ROTATION, clockwise);
        this.accelerationLimits.set(ROTATION, accelerationLimits.get(ROTATION));
    }

    private record TestSwerve(double wheelAngle, double velocity) implements SwerveModule {
        @Override public double getVelocity() {
            return this.velocity;
        }

        @Override public double getWheelAngle() {
            return this.wheelAngle;
        }

        // region Ignored Things
        @Override public double getMaximumSpeed() {
            return 0;
        }

        @Override public void setVelocity(double velocity) {

        }

        @Override public WheelPosition getWheelPosition() {
            return null;
        }

        @Override public double getDistance() {
            return 0;
        }

        @Override public void resetDistance() {

        }

        @Override public void setTargetDegrees(double degrees) {

        }

        @Override public void setTargetMagnitude(double magnitude) {

        }
        // endregion
    }
}