package org.firstinspires.ftc.teamcode.custompurepursuit;

/**
 * A point class to carry pathing data and perform operations on
 */
public class Point {
    /**
     * The x value of the point in space
     */
    public double x;
    /**
     * The y value of the point in space
     */
    public double y;


    /**
     * The maximum allowable deviation from this specific point
     */
    public double maxAllowableDeviation;

    /**
     * The heading that the robot should be at by this point
     */
    public double endHeading;

    /**
     * Constructor to set the coordinate location
     * @param x the x coordinate
     * @param y the y coordinate
     */
    public Point(double x, double y){
        this.x = x;
        this.y = y;
        maxAllowableDeviation = 0.1;
        endHeading = 0;
    }

    /**
     * Pattern builder to optionally set the heading
     * @param endHeading the end heading the robot should be at by this point
     * @return this object to use in a pattern builder
     */
    public Point setEndHeading(double endHeading){
        this.endHeading = endHeading;
        return this;
    }

    /**
     * Pattern builder to optionally set the maximum allowable distance deviation
     * @param maxAllowableDeviation is the straight-line distance from the point close enough to be
     *                              considered on the point.
     * @return this object to use in a pattern builder
     */
    public Point setMaximumAllowableDeviation(double maxAllowableDeviation){
        this.maxAllowableDeviation = maxAllowableDeviation;
        return this;
    }


    public Point subtract(Point point){
        return new Point(x-point.x, y-point.y)
                .setMaximumAllowableDeviation(maxAllowableDeviation).setEndHeading(endHeading);
    }
    public Point add(Point point){
        return new Point(x+point.x, y+point.y)
                .setMaximumAllowableDeviation(maxAllowableDeviation).setEndHeading(endHeading);
    }
    public Point multiply(Point point){
        return new Point(x*point.x, y*point.y)
                .setMaximumAllowableDeviation(maxAllowableDeviation).setEndHeading(endHeading);
    }
    public Point divide(Point point){
        return new Point(x/point.x, y/point.y)
                .setMaximumAllowableDeviation(maxAllowableDeviation).setEndHeading(endHeading);
    }

    public Point subtract(double a){
        return new Point(x-a, y-a)
                .setMaximumAllowableDeviation(maxAllowableDeviation).setEndHeading(endHeading);
    }
    public Point add(double a){
        return new Point(x+a, y+a)
                .setMaximumAllowableDeviation(maxAllowableDeviation).setEndHeading(endHeading);
    }
    public Point multiply(double a){
        return new Point(x*a, y*a)
                .setMaximumAllowableDeviation(maxAllowableDeviation).setEndHeading(endHeading);
    }
    public Point divide(double a){
        return new Point(x/a, y/a)
                .setMaximumAllowableDeviation(maxAllowableDeviation).setEndHeading(endHeading);
    }
}