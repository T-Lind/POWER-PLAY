package org.firstinspires.ftc.teamcode.coyote;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.coyote.auto.DefineEquations;
import org.firstinspires.ftc.teamcode.coyote.auto.SampleDifferentialSwerve;

/**
 * An example class to move in a straight line according to a sine position curve for five seconds.
 * If your robot doesn't move in a straight line go back to MotorTest.
 */
public class StraightTest extends LinearOpMode implements DefineEquations {
    @Override
    public double getX(double time) {
        if(time < 5)
            return 0;
        return Double.MIN_VALUE;
    }

    @Override
    public double getY(double time) {
        if(time < 5)
            return Math.sin(5*time/(2*Math.PI));
        return Double.MIN_VALUE;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        SampleDifferentialSwerve drive = new SampleDifferentialSwerve(hardwareMap);

        waitForStart();

        drive.followEquations(this, 0);
        drive.stop();
    }

}
