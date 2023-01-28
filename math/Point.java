package org.usfirst.frc.team2077.common.math;

import org.usfirst.frc.team2077.common.VelocityDirection;
import org.usfirst.frc.team2077.common.WheelPosition;

import static org.usfirst.frc.team2077.common.VelocityDirection.*;
import static org.usfirst.frc.team2077.common.WheelPosition.*;

public class Point {
    public final double north, east;

    /**
     * (0, 0) is assumed to be the <strong>center</strong> of the bot in question
     *
     * @param north coordinate
     * @param east  coordinate
     */
    public Point(double north, double east) {
        this.north = north;
        this.east = east;
    }

    public EnumMatrix<VelocityDirection, WheelPosition> inverseMatrixForBotSize(
            double length,
            double width
    ) {
        EnumMatrix<WheelPosition, VelocityDirection> wheelCoords = new EnumMatrix<>(
                WheelPosition.class,
                VelocityDirection.class
        );
        wheelCoords.set(FRONT_RIGHT, FORWARD, length / 2 - north);
        wheelCoords.set(FRONT_RIGHT, STRAFE, width / 2 - east);
        wheelCoords.set(FRONT_LEFT, FORWARD, length / 2 - north);
        wheelCoords.set(FRONT_LEFT, STRAFE, -width / 2 - east);

        wheelCoords.set(BACK_RIGHT, FORWARD, -length / 2 - north);
        wheelCoords.set(BACK_RIGHT, STRAFE, width / 2 - east);
        wheelCoords.set(BACK_LEFT, FORWARD, -length / 2 - north);
        wheelCoords.set(BACK_LEFT, STRAFE, -width / 2 - east);

        EnumMatrix<VelocityDirection, WheelPosition> inverseMatrix = new EnumMatrix<>(
                VelocityDirection.class,
                WheelPosition.class
        );
        inverseMatrix.set(FORWARD, FRONT_RIGHT, 1d);
        inverseMatrix.set(STRAFE, FRONT_RIGHT, -1d);
        inverseMatrix.set(
                ROTATION,
                FRONT_RIGHT,
                -wheelCoords.get(FRONT_RIGHT, FORWARD) - wheelCoords.get(FRONT_RIGHT, STRAFE)
        );
        inverseMatrix.set(FORWARD, FRONT_LEFT, 1d);
        inverseMatrix.set(STRAFE, FRONT_LEFT, 1d);
        inverseMatrix.set(
                ROTATION,
                FRONT_LEFT,
                wheelCoords.get(FRONT_LEFT, FORWARD) - wheelCoords.get(FRONT_LEFT, STRAFE)
        );

        inverseMatrix.set(FORWARD, BACK_RIGHT, 1d);
        inverseMatrix.set(STRAFE, BACK_RIGHT, 1d);
        inverseMatrix.set(
                ROTATION,
                BACK_RIGHT,
                wheelCoords.get(BACK_RIGHT, FORWARD) - wheelCoords.get(BACK_RIGHT, STRAFE)
        );
        inverseMatrix.set(FORWARD, BACK_LEFT, 1d);
        inverseMatrix.set(STRAFE, BACK_LEFT, -1d);
        inverseMatrix.set(
                ROTATION,
                BACK_LEFT,
                -wheelCoords.get(BACK_LEFT, FORWARD) - wheelCoords.get(BACK_LEFT, STRAFE)
        );

        return inverseMatrix;
    }
}
