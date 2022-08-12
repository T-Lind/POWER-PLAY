package org.firstinspires.ftc.teamcode.custompurepursuit;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.drive.StandardTrackingWheelLocalizer;

import java.util.ArrayList;

public class Follower {
    private StandardTrackingWheelLocalizer localizer;
    private HardwareMap hardwareMap;
    private SampleMecanumDrive drive;


    private static final double IN_TO_M = 0.0254;


    public static double KP = 3.5;
    public static double KI = 2.0;
    public static double KD = 1.5;
    public static double KF = 0.5;


    /**
     * Assign the hardware map to the localizer
     * @param hardwareMap the opMode's hardware map
     */
    public Follower(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        localizer = new StandardTrackingWheelLocalizer(hardwareMap);
        drive = new SampleMecanumDrive(hardwareMap);
    }

    /**
     * A pure pursuit algorithm to go to a certain position using PIDF
     * @param p0 is the starting point
     * @param p1 is the point to go to
     * @param pidFactor the amount to multiply the PID output by
     */
    public Point goToPositionConstantHeading(Point p0, Point p1, double pidFactor) {
        PIDFController xPID = new PIDFController(KP, KI, KD, KF);
        PIDFController yPID = new PIDFController(KP, KI, KD, KF);
        PIDFController headingPID = new PIDFController(KP, KI, KD, KF);

        double x1 = p1.subtract(p0).x;
        double y1 = p1.subtract(p0).y;

        if(localizer == null)
            throw new RuntimeException("Localizer has not been set in follower.java");


        double positionError = Math.hypot(
                x1-localizer.getPoseEstimate().getX()*IN_TO_M,
                y1-localizer.getPoseEstimate().getY()*IN_TO_M
        );

        while(positionError > p1.maxAllowableDeviation){
            localizer.update();

            double xPos = localizer.getPoseEstimate().getX()*IN_TO_M;
            double yPos = localizer.getPoseEstimate().getY()*IN_TO_M;
            double heading = localizer.getPoseEstimate().getHeading();

            double xCorrect = xPID.calculate(xPos, x1);
            double yCorrect = yPID.calculate(yPos, y1);
            double headingCorrect = 1E-3*headingPID.calculate(heading, 0);


            drive.setMotorPowers(
                    pidFactor*(xCorrect+yCorrect)-headingCorrect,
                    pidFactor*(xCorrect-yCorrect)-headingCorrect,
                    pidFactor*(xCorrect+yCorrect)+headingCorrect,
                    pidFactor*(xCorrect-yCorrect)+headingCorrect
            );

            positionError = Math.hypot(x1-xPos, y1-yPos);
        }

        return new Point(localizer.getPoseEstimate().getX()*IN_TO_M, localizer.getPoseEstimate().getY()*IN_TO_M);
    }

    /**
     * Follow a set of points with a constant heading
     * @param currentPos the current position
     * @param points the points to go to
     */
    public void followSequence(Point currentPos, ArrayList<Point> points){
        for(Point point : points){
            currentPos = goToPositionConstantHeading(currentPos, point, 0.5);
        }
    }

    public void stop(){
        drive.stop();
    }
}