package org.firstinspires.ftc.teamcode.experimental;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.custompurepursuit.Follower;
import org.firstinspires.ftc.teamcode.custompurepursuit.Point;
import org.firstinspires.ftc.teamcode.custompurepursuit.bezier.BezierCurve;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

import java.util.ArrayList;

@Autonomous(name="PurePursuitTest")
public class PurePursuitTest extends LinearOpMode {
    /**
     * Override this method and place your code here.
     * <p>
     * Please do not swallow the InterruptedException, as it is used in cases
     * where the op mode needs to be terminated early.
     *
     * @throws InterruptedException
     */
    @Override
    public void runOpMode() throws InterruptedException {
        Point currentPos = new Point(0, 0);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        waitForStart();

//        ArrayList<Point> controlPoints = new ArrayList<>();
//
//        controlPoints.add(new Point(0, 0));
//        controlPoints.add(new Point(0.25, 4));
//        controlPoints.add(new Point(2, 0));
//        controlPoints.add(new Point(0.25,0, 0.05));
//
//        Follower f = new Follower(hardwareMap, drive);
//
//        f.followSequence(currentPos, BezierCurve.generate(controlPoints, 12));

        Follower f = new Follower(hardwareMap, drive);
        f.goToPosition(currentPos, new Point(0.25, 0.5)
                .setMaximumAllowableDeviation(0.05), 0);


        drive.stop();
    }
}
