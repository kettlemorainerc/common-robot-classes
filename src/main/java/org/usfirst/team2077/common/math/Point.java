package org.usfirst.frc.team2077.common.math;

import org.usfirst.team2077.common.WheelPosition;
import org.usfirst.frc.team2077.common.drivetrain.*;

import static org.usfirst.frc.team2077.common.drivetrain.MecanumMath.VelocityDirection.*;
import static org.usfirst.team2077.common.WheelPosition.*;

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
        wheelCoords.set(FRONT_RIGHT, NORTH, length / 2 - north);
        wheelCoords.set(FRONT_RIGHT, EAST, width / 2 - east);
        wheelCoords.set(FRONT_LEFT, NORTH, length / 2 - north);
        wheelCoords.set(FRONT_LEFT, EAST, -width / 2 - east);

        wheelCoords.set(BACK_RIGHT, NORTH, -length / 2 - north);
        wheelCoords.set(BACK_RIGHT, EAST, width / 2 - east);
        wheelCoords.set(BACK_LEFT, NORTH, -length / 2 - north);
        wheelCoords.set(BACK_LEFT, EAST, -width / 2 - east);

        EnumMatrix<MecanumMath.VelocityDirection, WheelPosition> inverseMatrix = new EnumMatrix<>(
            MecanumMath.VelocityDirection.class,
            WheelPosition.class
        );
        inverseMatrix.set(NORTH, FRONT_RIGHT, 1d);
        inverseMatrix.set(EAST, FRONT_RIGHT, -1d);
        inverseMatrix.set(
            ROTATION,
                FRONT_RIGHT,
            -wheelCoords.get(FRONT_RIGHT, NORTH) - wheelCoords.get(FRONT_RIGHT, EAST)
        );
        inverseMatrix.set(NORTH, FRONT_LEFT, 1d);
        inverseMatrix.set(EAST, FRONT_LEFT, 1d);
        inverseMatrix.set(
            ROTATION,
                FRONT_LEFT,
            wheelCoords.get(FRONT_LEFT, NORTH) - wheelCoords.get(FRONT_LEFT, EAST)
        );

        inverseMatrix.set(NORTH, BACK_RIGHT, 1d);
        inverseMatrix.set(EAST, BACK_RIGHT, 1d);
        inverseMatrix.set(
            ROTATION,
                BACK_RIGHT,
            wheelCoords.get(BACK_RIGHT, NORTH) - wheelCoords.get(BACK_RIGHT, EAST)
        );
        inverseMatrix.set(NORTH, BACK_LEFT, 1d);
        inverseMatrix.set(EAST, BACK_LEFT, -1d);
        inverseMatrix.set(
            ROTATION,
                BACK_LEFT,
            -wheelCoords.get(BACK_LEFT, NORTH) - wheelCoords.get(BACK_LEFT, EAST)
        );

        return inverseMatrix;
    }
}
