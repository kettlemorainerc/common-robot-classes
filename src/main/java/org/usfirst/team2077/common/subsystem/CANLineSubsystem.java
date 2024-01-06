package org.usfirst.frc.team2077.common.subsystem;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.wpilibj2.command.*;
import org.usfirst.team2077.common.WheelPosition;
import org.usfirst.frc.team2077.common.drivetrain.*;

/**
 * Represents some subsystem on the CAN-line
 * @param <T> The actual motor's representation
 */
public abstract class CANLineSubsystem<T> implements Subsystem {
    public final T motor;
    public final int canId;

    protected CANLineSubsystem(T motor, int canId) {
        this.motor = motor;
        this.canId = canId;
    }

    @Override public void register() {
        Subsystem.super.register();
    }

    public abstract void setRPM(double RPM);
    public abstract void setPercent(double percent);

    public static final class Talon extends CANLineSubsystem<TalonSRX> {
        public Talon(int canId) {
            super(new TalonSRX(canId), canId);
        }

        @Override public void setRPM(double RPM) {
            throw new UnsupportedOperationException("Talon motors do not support set RPM. Either determine their maximum RPM and update Talon#setRPM, or use Talon#setPercent");
        }

        @Override public void setPercent(double percent) {
            motor.set(ControlMode.PercentOutput, percent);
        }
    }

    public static final class SparkNeo extends CANLineSubsystem<SparkNeoDriveModule> {
        public SparkNeo(SparkNeoDriveModule.DrivePosition position) {
            super(new SparkNeoDriveModule(position), position.ID);
        }

        public SparkNeo(WheelPosition position) {
            this(SparkNeoDriveModule.DrivePosition.forWheelPosition(position));
        }

        @Override public void setRPM(double RPM) {
            motor.setRPM(RPM);
        }

        @Override public void setPercent(double percent) {
            motor.setRPM(motor.maxRPM * percent);
        }
    }
}
