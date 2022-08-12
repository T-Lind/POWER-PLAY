package org.firstinspires.ftc.teamcode.custompurepursuit;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.custompurepursuit.bezier.BezierCurve;

import java.util.ArrayList;

/**
 * A class to follow a set of pathing commands using the custom pure pursuit library
 */
public class Sequence {
    private Follower follower;
    private Point startPos;
    private Telemetry telemetry;

    private ArrayList<Point> waypoints;

    public Sequence(HardwareMap hardwareMap){
        follower = new Follower(hardwareMap);
        startPos = new Point(0,0);
        waypoints = new ArrayList<>();
        waypoints.add(startPos);
    }

    public Sequence telemetry(Telemetry telemetry){
        follower.telemetry(telemetry);
        this.telemetry = telemetry;
        return this;
    }

    public Sequence startPos(Point startPos){
        this.startPos = startPos;
        return this;
    }

    /**
     * Go forward a certain distance
     * @param d the distance in meters to go forward
     * @return this object to be used in a builder design pattern
     */
    public Sequence forward(double d){
        if(d <= 0)
            throw new RuntimeException("Cannot go forward a negative distance!");

        Point prev = waypoints.get(waypoints.size()-1);
        waypoints.add(new Point(prev.x+d, prev.y));
        return this;
    }

    /**
     * Go back a certain distance
     * @param d the distance in meters to go back
     * @return this object to be used in a builder design pattern
     */
    public Sequence back(double d){
        if(d <= 0)
            throw new RuntimeException("Cannot go back a negative distance!");

        Point prev = waypoints.get(waypoints.size()-1);
        waypoints.add(new Point(prev.x-d, prev.y));
        return this;
    }

    /**
     * Strafe left a certain distance
     * @param d the distance in meters to strafe left
     * @return this object to be used in a builder design pattern
     */
    public Sequence strafeLeft(double d){
        if(d <= 0)
            throw new RuntimeException("Cannot strafe left a negative distance!");

        Point prev = waypoints.get(waypoints.size()-1);
        waypoints.add(new Point(prev.x, prev.y-d));
        return this;
    }

    /**
     * Strafe right a certain distance
     * @param d the distance in meters to strafe right
     * @return this object to be used in a builder design pattern
     */
    public Sequence strafeRight(double d){
        if(d <= 0)
            throw new RuntimeException("Cannot strafe right a negative distance!");

        Point prev = waypoints.get(waypoints.size()-1);
        waypoints.add(new Point(prev.x, prev.y+d));
        return this;
    }

    /**
     * Strafe left a certain distance
     * @param p1 the point to go to (meters)
     * @return this object to be used in a builder design pattern
     */
    public Sequence lineTo(Point p1){
        if(p1 == null)
            throw new RuntimeException("Cannot lineTo a null point!");

        waypoints.add(p1);
        return this;
    }

    /**
     * Strafe left a certain distance
     * @param controlPoints the points to construct a bezier curve with
     * @return this object to be used in a builder design pattern
     */
    public Sequence bezier(int numWaypoints, Point... controlPoints){
        if(controlPoints.length == 0)
            throw new RuntimeException("Cannot bezier curve with no points!");

        waypoints.addAll(BezierCurve.generate(numWaypoints, controlPoints));

        return this;
    }

    /**
     * Strafe left a certain distance
     * @param controlPoints the points to construct a bezier curve with
     * @return this object to be used in a builder design pattern
     */
    public Sequence bezier(int numWaypoints, double maxAllowableDeviation, Point... controlPoints){
        if(controlPoints.length == 0)
            throw new RuntimeException("Cannot bezier curve with no points!");

        ArrayList<Point> points = (BezierCurve.generate(numWaypoints, controlPoints));
        for(Point p1 : points){
            p1.maxAllowableDeviation = maxAllowableDeviation;
        }

        waypoints.addAll(points);

        return this;
    }

    public void follow(){
        follower.followSequence(startPos, waypoints);
        follower.stop();
    }
}
