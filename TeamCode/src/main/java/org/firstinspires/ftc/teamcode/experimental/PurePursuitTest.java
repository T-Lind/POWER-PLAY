package org.firstinspires.ftc.teamcode.experimental;


import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.custompurepursuit.Follower;
import org.firstinspires.ftc.teamcode.custompurepursuit.Point;
import org.firstinspires.ftc.teamcode.custompurepursuit.Sequence;
import org.firstinspires.ftc.teamcode.custompurepursuit.bezier.BezierCurve;
import org.openftc.revextensions2.ExpansionHubEx;

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

        ExpansionHubEx hub = hardwareMap.get(ExpansionHubEx.class, "Control Hub");
        hub.setLedColor(255, 255, 0);


        Sequence sequence = new Sequence(hardwareMap)
                .startPos(currentPos)
                .telemetry(telemetry)
//                .bezier(6, 0.05,
//                        new Point(0, 0),
//                        new Point(0.25, 2),
//                        new Point(1, 0),
//                        new Point(0.25,0))
                .forward(0.25)
                .strafeRight(0.5)
                .back(0.25)
                .strafeLeft(0.5)
                .forward(0.25)
                .strafeRight(0.5)
                .back(0.25)
                .strafeLeft(0.5)
                ;



        waitForStart();

        sequence.follow();


        hub.setLedColor(0,255,0);

    }
}
