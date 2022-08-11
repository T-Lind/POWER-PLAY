package org.firstinspires.ftc.teamcode.custompurepursuit;

public class Point {
    public double x;
    public double y;

    public double maxAllowableDeviation;

    public Point(double x, double y){
        this.x = x;
        this.y = y;
        maxAllowableDeviation = 0.2;
    }

    public Point(double x, double y, double maxAllowableDeviation){
        this.x = x;
        this.y = y;
        this.maxAllowableDeviation = maxAllowableDeviation;
    }

    public Point subtract(Point point){
        return new Point(x-point.x, y-point.y, maxAllowableDeviation);
    }
    public Point add(Point point){
        return new Point(x+point.x, y+point.y, maxAllowableDeviation);
    }
    public Point multiply(Point point){
        return new Point(x*point.x, y*point.y, maxAllowableDeviation);
    }
    public Point divide(Point point){
        return new Point(x/point.x, y/point.y,maxAllowableDeviation);
    }

    public Point subtract(double a){
        return new Point(x-a, y-a, maxAllowableDeviation);
    }
    public Point add(double a){
        return new Point(x+a, y+a, maxAllowableDeviation);
    }
    public Point multiply(double a){
        return new Point(x*a, y*a,maxAllowableDeviation);
    }
    public Point divide(double a){
        return new Point(x/a, y/a, maxAllowableDeviation);
    }
}