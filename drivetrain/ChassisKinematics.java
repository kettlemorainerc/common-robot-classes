package org.usfirst.frc.team2077.common.drivetrain;

import org.usfirst.frc.team2077.common.VelocityDirection;
import org.usfirst.frc.team2077.common.WheelPosition;

import java.util.EnumMap;

public interface ChassisKinematics<InverseResult, ForwardArgs> {
    EnumMap<WheelPosition, InverseResult> targetsForVelocity(EnumMap<VelocityDirection, Double> velocityTarget);
    EnumMap<VelocityDirection, Double> velocityForTargets(EnumMap<WheelPosition, ForwardArgs> targets);
}
