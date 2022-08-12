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
     * Constructor to set the coordinate location
     * @param x the x coordinate
     * @param y the y coordinate
     */
    public Point(double x, double y){
        this.x = x;
        this.y = y;
        maxAllowableDeviation = 0.1;
    }


    /**
     * Pattern builder to optionally set the maximum allowable distance deviation
     * @param maxAllowableDeviation is the straight-line distance from the point close enough to be
     *                              considered on the point.
     * @return this object to use in a pattern builder
     */
    public Point maximumAllowableDeviation(double maxAllowableDeviation){
        this.maxAllowableDeviation = maxAllowableDeviation;
        return this;
    }


    public Point subtract(Point point){
        return new Point(x-point.x, y-point.y)
                .maximumAllowableDeviation(maxAllowableDeviation);
    }
    public Point add(Point point){
        return new Point(x+point.x, y+point.y)
                .maximumAllowableDeviation(maxAllowableDeviation);
    }
    public Point multiply(Point point){
        return new Point(x*point.x, y*point.y)
                .maximumAllowableDeviation(maxAllowableDeviation);
    }
    public Point divide(Point point){
        return new Point(x/point.x, y/point.y)
                .maximumAllowableDeviation(maxAllowableDeviation);
    }

    public Point subtract(double a){
        return new Point(x-a, y-a)
                .maximumAllowableDeviation(maxAllowableDeviation);
    }
    public Point add(double a){
        return new Point(x+a, y+a)
                .maximumAllowableDeviation(maxAllowableDeviation);
    }
    public Point multiply(double a){
        return new Point(x*a, y*a)
                .maximumAllowableDeviation(maxAllowableDeviation);
    }
    public Point divide(double a){
        return new Point(x/a, y/a)
                .maximumAllowableDeviation(maxAllowableDeviation);
    }
}