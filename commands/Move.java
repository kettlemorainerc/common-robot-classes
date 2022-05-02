/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FRC Team 2077. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams.            */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2077.common.commands;

import edu.wpi.first.wpilibj2.command.*;
import org.usfirst.frc.team2077.common.*;
import org.usfirst.frc.team2077.common.drivetrain.*;
import org.usfirst.frc.team2077.common.drivetrain.MecanumMath.*;
import org.usfirst.frc.team2077.common.math.*;
import org.usfirst.frc.team2077.common.math.AccelerationLimits.*;

import java.util.*;

public class Move extends CommandBase {
	public static final double ACCELERATION_G_LIMIT = .1;
	public static final double DECELERATION_G_LIMIT = .3;
	private final AbstractChassis chassis;
	private final double[] distanceTotal; // {north, east, rotation} (signed)
	private final int method; // 1 2 or 3 (#args to setVelocity/setRotation)

	private double[] fast; // {north, east, rotation} (signed)
	public double[] slow; // {north, east, rotation} (signed)
	private AccelerationLimits acceleration; // like getAccelerationLimits, but scaled

	private double[] distanceRemaining; // {north, east, rotation} (signed)
	private boolean[] finished; // {north, east, rotation}

	private Position origin;

	public Move(RobotHardware<?> hardware, double north, double east, double rotation) {
		this(hardware, north, east, rotation, 3, hardware.getPosition(), hardware.getHeading());
	}

	public Move(RobotHardware<?> hardware, double north, double east) {
		this(hardware, north, east, 0, 2, hardware.getPosition());
	}

	public Move(RobotHardware<?> hardware, double rotation) {
		this(hardware, 0, 0, rotation, 1, hardware.getHeading());
	}

	private Move(RobotHardware<?> hardware, double north, double east, double rotation, int method, Subsystem... requirements) {
		addRequirements(requirements);
		this.chassis = hardware.getChassis();

//		distanceTotal_ = new double[]{north, east * .68, rotation * 7 / 8}; //fudged values for the multipliers
		distanceTotal = new double[]{north, east * .68, rotation * 7 / 8}; //fudged values for the multipliers
		this.method = method;
	}


	@Override
	public void initialize() {

		EnumMap<VelocityDirection, Double> max = chassis.getMaximumVelocity();
		EnumMap<VelocityDirection, Double> min = chassis.getMinimumVelocity();

		// scale factors for north/east/rotation by fraction of maximum velocity
		double[] scale = {
			Math.abs(distanceTotal[0]) / max.get(VelocityDirection.NORTH),
			Math.abs(distanceTotal[1]) / max.get(VelocityDirection.EAST),
			Math.abs(distanceTotal[2]) / max.get(VelocityDirection.ROTATION)
		};
		double maxScale = Math.max(scale[0], Math.max(scale[1], scale[2]));
		scale = new double[]{scale[0] / maxScale, scale[1] / maxScale, scale[2] / maxScale}; // 0 - 1
		double[] sign = {
			Math.signum(distanceTotal[0]),
			Math.signum(distanceTotal[1]),
			Math.signum(distanceTotal[2])
		};

		// scale speeds and acceleration/deceleration
		fast = new double[]{
			Math.max(min.get(VelocityDirection.NORTH), max.get(VelocityDirection.NORTH) * scale[0]) * sign[0],
			Math.max(min.get(VelocityDirection.EAST), max.get(VelocityDirection.EAST) * scale[1]) * sign[1],
			Math.max(min.get(VelocityDirection.ROTATION), max.get(VelocityDirection.ROTATION) * scale[2]) * sign[2]
		}; // don't let maximum scale below minimum
		slow = new double[]{
			min.get(VelocityDirection.NORTH) * sign[0],
			min.get(VelocityDirection.EAST) * sign[1],
			min.get(VelocityDirection.ROTATION) * sign[2]
		}; // don't scale below minimum
		acceleration = new AccelerationLimits(ACCELERATION_G_LIMIT, DECELERATION_G_LIMIT, chassis, scale);

		origin = new Position(chassis.getPosition());
		distanceRemaining = new double[]{distanceTotal[0], distanceTotal[1], distanceTotal[2]};
		finished = new boolean[]{
			Math.abs(distanceRemaining[0]) == 0.,
			Math.abs(distanceRemaining[1]) == 0.,
			Math.abs(distanceRemaining[2]) == 0.
		};
	}

	@Override
	public void execute() {

		EnumMap<VelocityDirection, Double> vCurrent = chassis.getVelocityCalculated();
		double[] vNew = {0, 0, 0};
		EnumMap<VelocityDirection, Double> distanceTraveled = (new Position(chassis.getPosition())).distanceRelative(
			origin);
		boolean[] slow = {false, false, false};
		for(int i = 0; i < 3; i++) {
			VelocityDirection direction = VelocityDirection.values()[i];
			distanceRemaining[i] = distanceTotal[i] - distanceTraveled.get(direction);
			double distanceToStop = vCurrent.get(direction) * vCurrent.get(direction) /
									acceleration.get(direction, Type.DECELERATION) /
									2.;// exact absolute value per physics
			distanceToStop += Math.max(
				distanceToStop * .05,
				Math.abs(vCurrent.get(direction)) * .04
			); // pad just a bit to avoid overshoot
			slow[i] = finished[i] ||
					  Math.abs(distanceRemaining[i]) <= distanceToStop; // slow down within padded stopping distance
		}
		boolean s = Math.abs(distanceTotal[2]) > 0 ? slow[2] : (slow[0] && slow[1]);
		for(int i = 0; i < 3; i++) {
			vNew[i] = finished[i] ? 0. : s ? this.slow[i] : fast[i];
		}

		switch(method) {
			case 3:
				chassis.setVelocity(vNew[0], vNew[0], vNew[2], acceleration);
				break;
			case 2:
				chassis.setVelocity(vNew[0], vNew[0], acceleration);
				break;
			case 1:
				chassis.setRotation(vNew[2], acceleration);
				break;
		}
	}

	@Override
	public boolean isFinished() {
		for(int i = 0; i < 3; i++) {
			finished[i] = finished[i] || (Math.signum(distanceRemaining[i]) != Math.signum(distanceTotal[i]));
		}
		return Math.abs(distanceTotal[2]) > 0 ? finished[2] : (finished[0] && finished[1]);
	}
}
