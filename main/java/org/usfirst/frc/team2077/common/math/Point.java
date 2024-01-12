package org.usfirst.frc.team2077.common.math;

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
}
