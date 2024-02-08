package org.usfirst.frc.team2077.common.subsystem;

import com.revrobotics.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.usfirst.frc.team2077.common.RectangularWheelPosition;
import org.usfirst.frc.team2077.common.drivetrain.DriveModuleIF;

public class SwerveMotor implements Subsystem, SwerveModule, DriveModuleIF {
    private static final double MAX_DRIVE_PERCENT = 0.2;

    private static final double WHEEL_RADIUS = 2.0;
    private static final double WHEEL_CIRCUMFERENCE = (2 * Math.PI * WHEEL_RADIUS);

    private static final double DRIVE_GEAR_RATIO = 5.08;

    private final BetterCanSparkMax directionMotor;
    private final BetterCanSparkMax magnitudeMotor;

    private final AbsoluteEncoder absoluteEncoder;

    private double targetAngle = 0;
    private double targetVelocity = 0;

    private boolean flipMagnitude;

    private PIDController pid = new PIDController(0.024, 0.009, 0);

    private SwervePosition position;

    public SwerveMotor(
          int directionId,
          int magnitudeId
    ) {
        directionMotor = new BetterCanSparkMax(directionId, CANSparkLowLevel.MotorType.kBrushless);
        magnitudeMotor = new BetterCanSparkMax(magnitudeId, CANSparkLowLevel.MotorType.kBrushless);

        absoluteEncoder = directionMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);

        pid.setSetpoint(0.0);

        this.register();

    }

    public SwerveMotor(SwervePosition motorPosition) {
        this(
              motorPosition.directionId(),
              motorPosition.magnitudeId()
        );

        position = motorPosition;
    }

    private void updatePID(double p, double i){
        pid.setPID(p, i, 0);
    }

    @Override public void setTargetDegrees(double degrees) {
        setTargetAngle(degrees);
    }

    public void setTargetAngle(double angle) {
        targetAngle = angle;

        double currentWheelAngle = getWheelAngle();
        double angleDifference = getAngleDifference(currentWheelAngle, targetAngle);

        flipMagnitude = false;
        if(Math.abs(angleDifference) > 90) {
            targetAngle -= 180;
            flipMagnitude = true;
        }

        targetAngle %= 360;
        if(targetAngle < 0) targetAngle += 360;
    }

    public void setDirectionPercent(double percent) {
        setDirectionMotor(percent);
    }

    private double getWheelRotation(){
        return absoluteEncoder.getPosition() - position.encoderOffset();
    }

    public double getWheelAngle() {
        double percentTurned = getWheelRotation();
        double angle = percentTurned * 360;

        angle %= 360.0;
        if(angle < 0) angle += 360.0;

        return angle;
    }

    @Override public void periodic() {
        updateMagnitude();

        updateRotation();
    }

    @Override public void setTargetMagnitude(double magnitude) {}

    private void setMagnitudePercent(double velocity) {
        if(flipMagnitude) velocity = -velocity;
        magnitudeMotor.setTargetVelocity(-velocity);
    }

    private void updateMagnitude() {
        setMagnitudePercent(targetVelocity);
    }

    private void updateRotation() {
        if(this.targetVelocity == 0){
            setDirectionMotor(0);
            return;
        }

        double currentAngle = getWheelAngle();
        double angleDifference = getAngleDifference(currentAngle, targetAngle);

        double c = pid.calculate( angleDifference );

        if(Math.abs(c) < 0.0001) c = 0;

        directionMotor.set(-c);
    }

    private void setDirectionMotor(double percent) {
        directionMotor.setTargetVelocity(percent);
    }

    public SwervePosition getPosition() {
        return position;
    }

    private double getAngleDifference(double from, double to) {
        double angleDifference = from - to;

        if(Math.abs(angleDifference) > 180) {
            angleDifference -= 360 * Math.signum(angleDifference);
        }

        return angleDifference;
    }

    public double getMaximumSpeed() {
        double rawRPM = position.maxRpm();
        double rawWheelRPM = rawRPM / DRIVE_GEAR_RATIO;
        double rawWheelRPS = rawWheelRPM / 60;
        double rawVelocity = rawWheelRPS * WHEEL_CIRCUMFERENCE;

        return rawVelocity;
    }

    @Override public void setVelocity(double velocity) {
        targetVelocity = MAX_DRIVE_PERCENT * velocity * DRIVE_GEAR_RATIO * 60 / WHEEL_CIRCUMFERENCE;
    }

    @Override public RectangularWheelPosition getWheelPosition() {
        return position.wheelPosition();
    }

    @Override public double getVelocity() {
        double rawRPM = magnitudeMotor.getEncoder().getVelocity();

        double rawWheelRPM = rawRPM / DRIVE_GEAR_RATIO;
        double rawWheelRPS = rawWheelRPM / 60;
        double rawDistance = rawWheelRPS * WHEEL_CIRCUMFERENCE;

        return rawDistance;
    }

    // public boolean isAtTarget(){
    //     return Math.abs(getAngleDifference(getWheelAngle(), targetAngle)) < DEAD_ANGLE;
    // }

    // TODO: Move elsewhere in robot-specific code
    // //Checks if motors are all facing towards target, if so removes the "rotate" flag, allowing the magnitude motors to run
    // public static boolean checkDirection(){
    //     boolean atDirection = true;
    //     RobotHardware hardware = RobotHardware.getInstance();
    //
    //     for(MotorPosition position : MotorPosition.values()){
    //         SwerveMotor module = hardware.getWheel(position.wheelPosition);
    //         atDirection &= module.isAtTarget();
    //     }
    //
    //     return atDirection;
    //     //        if(atDirection) rotateFirst = false;
    // }

    @Override public String toString() {
        return Double.toString(getVelocity());
    }
}
