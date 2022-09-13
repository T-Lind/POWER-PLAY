package org.firstinspires.ftc.teamcode.coyote.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.coyote.coyote.SampleDifferentialSwerve;
import org.firstinspires.ftc.teamcode.coyote.teleop.container.TeleopContainer;

@TeleOp
public class DiffyTestCv2 extends TeleopContainer {
    private SampleDifferentialSwerve drive;

    @Override
    public void init() {
        drive = new SampleDifferentialSwerve(hardwareMap);
        initMechanisms();
    }

    @Override
    public void loop() {
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

        telemetry.addData("Stick angle", stickAngle);
        telemetry.addData("Stick magnitude", stickMagnitude);

        drive.correctWheelAngles(leftPodAngle, rightPodAngle, stickAngle);

        double avgError = (Math.abs(leftPodAngle-stickAngle)+Math.abs(rightPodAngle-stickAngle))/2;

        telemetry.addData("avg error",avgError);

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
}
