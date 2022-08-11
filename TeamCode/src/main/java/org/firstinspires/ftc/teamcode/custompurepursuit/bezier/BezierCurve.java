package org.firstinspires.ftc.teamcode.custompurepursuit.bezier;

import org.firstinspires.ftc.teamcode.custompurepursuit.Point;

import java.util.ArrayList;

public class BezierCurve {
    private static Point lerp(Point p0, Point p1, double t){
        return p0.multiply((1-t)).add(p1.multiply(t));
    }

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

    public static ArrayList<Point> generate(ArrayList<Point> controlPoints, int numPointsToGenerate){
        ArrayList<Point> points = new ArrayList<>();
        for(double time=0;time<1;time+=1.0/numPointsToGenerate){
            points.add(recursiveLerp(controlPoints, time));
        }

        return points;
    }
}
