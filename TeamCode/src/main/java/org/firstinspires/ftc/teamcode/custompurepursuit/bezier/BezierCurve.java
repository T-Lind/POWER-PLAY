package org.firstinspires.ftc.teamcode.custompurepursuit.bezier;

import org.firstinspires.ftc.teamcode.custompurepursuit.Point;

import java.util.ArrayList;

/**
 * A class to generate points along a bezier curve given a certain set of inputs. A class completely
 * composed of static methods with recursive lerping being a private method.
 */
public class BezierCurve {
    /**
     * Perform a singular liner interpolation given a certain time
     * @param p0 staring point
     * @param p1 ending point
     * @param t current time
     * @return the position along the line based on time
     */
    private static Point lerp(Point p0, Point p1, double t){
        return p0.multiply((1-t)).add(p1.multiply(t));
    }

    /**
     * A recursive method to calculate the current position along a bezier curve given the time
     * into it and a set of control points.
     * @param controlPoints the set of control points to perform recursive linear interpolation on.
     * @param t the time into the bezier curve
     * @return the current point along the bezier curve given the time
     */
    private static Point recursiveLerp(ArrayList<Point> controlPoints, double t){
        if(controlPoints.size() == 2){
            return BezierCurve.lerp(controlPoints.get(0), controlPoints.get(1), t);
        }

        int index = 0;
        ArrayList<Point> points = new ArrayList<>();

        while(index < controlPoints.size()-1){
            Point point = BezierCurve.lerp(controlPoints.get(index),controlPoints.get(index+1),t);
            points.add(point);
            index++;
        }

        return recursiveLerp(points, t);
    }

    /**
     * Generate an ArrayList of points along a bezier curve given a set of control points and number
     * of points to generate.
     * @param controlPoints the set of control points to perform recursive linear interpolation on.
     * @param numPointsToGenerate the number of points to generate along the bezier curve
     *                            (resolution of the path)
     * @return the ArrayList of points along the path.
     */
    public static ArrayList<Point> generate(ArrayList<Point> controlPoints, int numPointsToGenerate){
        ArrayList<Point> points = new ArrayList<>();
        for(double time=0;time<1;time+=1.0/numPointsToGenerate){
            points.add(recursiveLerp(controlPoints, time));
        }

        return points;
    }
}
