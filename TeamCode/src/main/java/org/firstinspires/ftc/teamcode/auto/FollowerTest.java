package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.differentialequationfollower.DefineEquations;

public class FollowerTest extends LinearOpMode implements DefineEquations{
    @Override
    public double getX(double time) {
        return 0;
    }

    @Override
    public double getY(double time) {
        return 0;
    }

    @Override
    public double getHeading(double time) {
        return 0;
    }

    @Override
    public void runOpMode() throws InterruptedException {

    }

}
