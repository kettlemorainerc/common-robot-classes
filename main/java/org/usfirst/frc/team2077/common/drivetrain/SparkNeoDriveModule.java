
package org.usfirst.frc.team2077.common.drivetrain;

import com.revrobotics.*;
import org.usfirst.frc.team2077.common.WheelPosition;

public class SparkNeoDriveModule extends CANSparkMax implements DriveModuleIF {
    private static final boolean USE_SOFTWARE_PID = true;

    //6 inch wheels on rnd bot
    private final SparkPIDController pidController;
    private final RelativeEncoder encoder;
    private double setPoint;
    public final double circumference;
    public final double maxRPM;
    private final DrivePosition position;

    public SparkNeoDriveModule(final DrivePosition pos) {
        super(pos.motorId(), MotorType.kBrushless);
        this.position = pos;
        maxRPM = pos.maxRpm();
        circumference = pos.radius() * 2 * Math.PI;
        pidController = this.getPIDController();
        encoder = this.getEncoder();

        if(USE_SOFTWARE_PID || pos.shouldSetPid()) {
            pidController.setP(position.P());
            pidController.setI(position.I());
            pidController.setD(position.D());
            pidController.setIZone(0);
            pidController.setFF(0);
            pidController.setOutputRange(-1, 1);
        }

    }

    public DrivePosition getPosition() {
        return position;
    }
    
    @Override
    public double getMaximumSpeed() {
        return (maxRPM/position.gearRatio()) / (60 / (2 * Math.PI * position.radius()));
    }

    public void setVelocity(final double velocity) {
        //convert from inches/second to rpm
        setPoint = velocity*position.gearRatio() * 60 / circumference;
//        if (setPoint > maxRPM) {
//            setPoint = maxRPM;
//        }
        setRPM(setPoint > maxRPM ? maxRPM : setPoint);
    }

    @Override public WheelPosition getWheelPosition() {return position.getWheelPosition();}

    public void setRPM(double rpm) {
        setPoint = Math.min(rpm, maxRPM);
        if (position.inverse()) {
            pidController.setReference(-setPoint, ControlType.kVelocity);
        } else {
            pidController.setReference(setPoint, ControlType.kVelocity);
        }
//        if (position == DrivePosition.SHOOTER) System.out.println("SET RPM: " + setPoint);//PRINTS TOO OFTEN WHEN IN goIfShooterSpeedReady
    }

    public double getRPM() {
        final double velocity = encoder.getVelocity();
        if (position.inverse()) {
            return -velocity;
        } else {
            return velocity;
        }
    }

    public double getSetPoint() {
        if (position.inverse()) {
            return -setPoint;
        }
        return setPoint;
    }

    public double getVelocity() {
        final double velocity = encoder.getVelocity() / 60 / position.gearRatio() * circumference; //need to still convert to inches per second
        if (position.inverse()) {
            return -velocity;
        } else {
            return velocity;
        }
    }

    public double getDistance() {
        return encoder.getPosition() / position.gearRatio() * circumference;
    }

    public void resetDistance() {
        encoder.setPosition(0);
    }
}