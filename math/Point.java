package org.usfirst.frc.team2077.common.math;

import org.usfirst.frc.team2077.common.WheelPosition;
import org.usfirst.frc.team2077.common.drivetrain.*;

import static org.usfirst.frc.team2077.common.drivetrain.MecanumMath.VelocityDirection.*;
import static org.usfirst.frc.team2077.common.WheelPosition.*;

public class Point {
    public final double north, east;

    /**
     * (0, 0) is assumed to be the <strong>center</strong> of the bot in question
     *
     * @param north coordinate
     * @param east coordinate
     */
    public Point(double north, double east) {
        this.north = north;
        this.east = east;
    }

    public EnumMatrix<MecanumMath.VelocityDirection, WheelPosition> inverseMatrixForBotSize(
        double length,
        double width
    ) {
        EnumMatrix<WheelPosition, MecanumMath.VelocityDirection> wheelCoords = new EnumMatrix<>(
            WheelPosition.class,
            MecanumMath.VelocityDirection.class
        );
        wheelCoords.set(NORTH_EAST, NORTH, length / 2 - north);
        wheelCoords.set(NORTH_EAST, EAST, width / 2 - east);
        wheelCoords.set(NORTH_WEST, NORTH, length / 2 - north);
        wheelCoords.set(NORTH_WEST, EAST, -width / 2 - east);

        wheelCoords.set(SOUTH_EAST, NORTH, -length / 2 - north);
        wheelCoords.set(SOUTH_EAST, EAST, width / 2 - east);
        wheelCoords.set(SOUTH_WEST, NORTH, -length / 2 - north);
        wheelCoords.set(SOUTH_WEST, EAST, -width / 2 - east);

        EnumMatrix<MecanumMath.VelocityDirection, WheelPosition> inverseMatrix = new EnumMatrix<>(
            MecanumMath.VelocityDirection.class,
            WheelPosition.class
        );
        inverseMatrix.set(NORTH, NORTH_EAST, 1d);
        inverseMatrix.set(EAST, NORTH_EAST, -1d);
        inverseMatrix.set(
            ROTATION,
            NORTH_EAST,
            -wheelCoords.get(NORTH_EAST, NORTH) - wheelCoords.get(NORTH_EAST, EAST)
        );
        inverseMatrix.set(NORTH, NORTH_WEST, 1d);
        inverseMatrix.set(EAST, NORTH_WEST, 1d);
        inverseMatrix.set(
            ROTATION,
            NORTH_WEST,
            wheelCoords.get(NORTH_WEST, NORTH) - wheelCoords.get(NORTH_WEST, EAST)
        );

        inverseMatrix.set(NORTH, SOUTH_EAST, 1d);
        inverseMatrix.set(EAST, SOUTH_EAST, 1d);
        inverseMatrix.set(
            ROTATION,
            SOUTH_EAST,
            wheelCoords.get(SOUTH_EAST, NORTH) - wheelCoords.get(SOUTH_EAST, EAST)
        );
        inverseMatrix.set(NORTH, SOUTH_WEST, 1d);
        inverseMatrix.set(EAST, SOUTH_WEST, -1d);
        inverseMatrix.set(
            ROTATION,
            SOUTH_WEST,
            -wheelCoords.get(SOUTH_WEST, NORTH) - wheelCoords.get(SOUTH_WEST, EAST)
        );

        return inverseMatrix;
    }
}
