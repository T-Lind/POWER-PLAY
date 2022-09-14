package org.firstinspires.ftc.teamcode.coyote.teleop.container;

import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.coyote.auto.SampleDifferentialSwerve;
import org.firstinspires.ftc.teamcode.roadrunner.drive.StandardTrackingWheelLocalizer;

abstract public class TeleopContainer extends OpMode {
    private Timing.Timer matchTimer;
    private RevIMU imu;
    private StandardTrackingWheelLocalizer myLocalizer;

    private boolean timerStarted = false;

    public void initMechanisms(){
        imu.init();
        myLocalizer = new StandardTrackingWheelLocalizer(hardwareMap);
    }

    public void updateTimer(){
        if(!timerStarted)
            matchTimer = new Timing.Timer(120);
            timerStarted = true;
        alertToTiming();
    }

    public void updateDrivetrain(SampleDifferentialSwerve drive){
        updateTimer();

        double leftPodAngle = drive.getLeftPodAngle();
        double rightPodAngle = drive.getRightPodAngle();

        telemetry.addData("Left pod", leftPodAngle);
        telemetry.addData("Right pod", rightPodAngle);

        double leftStickX = gamepad1.left_stick_x;
        double leftStickY = -gamepad1.left_stick_y;
        double rightStickX = gamepad1.right_stick_x;


        double stickAngle = Math.toDegrees(Math.atan2(-leftStickX, leftStickY));
        double stickMagnitude = Math.hypot(leftStickX, leftStickY);
        if(leftStickX == 0 && leftStickY == 0)
            stickAngle = 0;


        drive.correctWheelAngles(leftPodAngle, rightPodAngle, stickAngle);

        double avgError = (Math.abs(leftPodAngle-stickAngle)+Math.abs(rightPodAngle-stickAngle))/2;

        if(avgError < 90){

            double FLP = stickMagnitude+rightStickX;
            double BLP = -stickMagnitude-rightStickX;
            double FRP = stickMagnitude-rightStickX;
            double BRP = -stickMagnitude+rightStickX;
            drive.addMotorPowers(FLP, BLP, FRP, BRP);
        }

        drive.updateMotorPowers();
        telemetry.update();
    }


    /**
     * Alert the driver at different time intervals. Note that it takes a while to say the words
     * so they're started speaking ahead of where they should be.
     */
    private void alertToTiming() {
        if (matchTimer.elapsedTime() == 30L) telemetry.speak("30 seconds into tele-op");

        else if (matchTimer.remainingTime() == 45L) telemetry.speak("15 seconds until end game");

        else if (matchTimer.remainingTime() == 5L) telemetry.speak("three");

        else if (matchTimer.remainingTime() == 3L) telemetry.speak("two");

        else if (matchTimer.remainingTime() == 1L) telemetry.speak("one");
    }
}
