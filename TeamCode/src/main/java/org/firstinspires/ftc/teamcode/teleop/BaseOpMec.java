package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.teleop.support.MecTeleOpContainer;


/**
 * Class to run a mecanum drivetrain
 * with basic TeleOp controls.
 */
@TeleOp(name="BaseOp")
public class BaseOpMec extends MecTeleOpContainer {
//    protected ServoEx servo;
    @Override
    protected void initSpecificMechanisms() {
//        servo = new SimpleServo(
//                hardwareMap, "s1", 0, 180
//        );

    }

    /**
     * Method from TeleOpContainer that must be used in order for the teleOp to work.
     * Either put setFieldCentric() or put setRobotCentric() at the start of the method.
     */
    @Override
    protected void updateMechanisms() {
        // Use the left bumper to switch whether or not the drive type is field centric
//        updateServo();

        telemetry.addData("Time left in match: ", getRemainingMatchTime());
        telemetry.addData("Field Centric Drive Type", getFieldCentric());

        telemetry.addData("x", getX());
        telemetry.addData("y", getY());
        telemetry.addData("heading", Math.toDegrees(getAngle()));
        telemetry.update();

    }


}
