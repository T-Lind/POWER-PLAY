package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.teleop.support.DiffyTeleOpContainer;

@TeleOp
public class DiffyBaseOp extends DiffyTeleOpContainer {
    private Motor leftPodEncoder;
    private Motor rightPodEncoder;

    private PIDFController leftPodPIDF;
    private PIDFController rightPodPIDF;

    private double getPodAngle(String pod){
        if(pod.equals("left")){
            return leftPodEncoder.getDistance()/8192.0*360;
        }
        else if(pod.equals("right")){
            return rightPodEncoder.getDistance()/8192.0*360;
        }
        else{
            throw new IllegalArgumentException("Improper argument specified!");
        }
    }

    @Override
    protected void initSpecificMechanisms() {
        leftPodPIDF = new PIDFController(0.05, 0, 0.001, 0.0);
        rightPodPIDF = new PIDFController(0.065, 0, 0.0015, 0.001);

        leftPodEncoder = new Motor(hardwareMap, "FR", Motor.GoBILDA.RPM_312);
        rightPodEncoder = new Motor(hardwareMap, "FL", Motor.GoBILDA.RPM_312);

        leftPodEncoder.setRunMode(Motor.RunMode.PositionControl);
        rightPodEncoder.setRunMode(Motor.RunMode.PositionControl);
    }

    @Override
    protected void updateMechanisms() {
        double leftAngle = getPodAngle("left");
        double rightAngle = getPodAngle("right");
        telemetry.addData("Left encoder:", leftAngle);
        telemetry.addData("Right encoder", rightAngle);

        double left_stick_x = gamepad1.left_stick_x;
        double left_stick_y = -gamepad1.left_stick_y;
        double right_stick_x = gamepad1.right_stick_x;

        double stickAngle = Math.toDegrees(Math.atan2(-left_stick_x, left_stick_y));
        double stickMagnitude = Math.hypot(left_stick_x, left_stick_y);
        if(left_stick_x == 0 && left_stick_y == 0)
            stickAngle = 0;

        double powerLeftVal = leftPodPIDF.calculate(leftAngle, stickAngle);
        double powerRightVal = rightPodPIDF.calculate(rightAngle, stickAngle);

        double avgError = (Math.abs(leftAngle-stickAngle)+Math.abs(rightAngle-stickAngle))/2;

        double FLP;
        double BLP;
        double FRP;
        double BRP;

        if(avgError < 10){
            FLP = powerLeftVal+stickMagnitude;
            BLP = powerLeftVal-stickMagnitude;
            FRP = powerRightVal+stickMagnitude;
            BRP = powerRightVal-stickMagnitude;
        }
        else{
            FLP = powerLeftVal;
            BLP = powerLeftVal;
            FRP = powerRightVal;
            BRP = powerRightVal;
        }

        double max = Math.max(Math.max(FLP, FRP), Math.max(BLP, BRP));
        FLP /= max;
        BRP /= max;
        BLP /= max;
        FRP /= max;

        frontLeft.set(FLP);
        backLeft.set(BLP);
        frontRight.set(FRP);
        backRight.set(BRP);


        telemetry.addData("Stick angle:", stickAngle);
        telemetry.update();
    }
}
