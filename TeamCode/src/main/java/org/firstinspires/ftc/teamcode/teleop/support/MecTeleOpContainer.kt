package org.firstinspires.ftc.teamcode.teleop.support

import com.arcrobotics.ftclib.drivebase.MecanumDrive
import org.firstinspires.ftc.teamcode.teleop.Toggle
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import kotlin.math.pow

@Strictfp
abstract class MecTeleOpContainer : UniversalTeleopContainer() {
    /**
     * Drivetrain object from FTCLib
     */
    @Transient
    protected var drive: MecanumDrive? = null

    /**
     * Stores the state of the field centric variable locally
     */
    @Transient
    private var isFieldCentric = false

    /**
     * Toggle object to store whether or not the drivetrain is in field centric mode
     */
    @Transient
    private var fieldCentric: Toggle? = null

    /**
     * Method to get the field centric state variable for displaying in telemetry, etc.
     * @return If the drivetrain is in field centric mode or not
     */
    protected fun getFieldCentric(): Boolean {
        return isFieldCentric
    }

    /**
     * Initialize the motors, zero power behavior, drivetrain object, imu, and whatever other
     * hardware you want to initialize every time
     */
    override fun init() {
        super.init()

        drive = MecanumDrive(
            frontLeft,
            frontRight,
            backLeft,
            backRight
        )


        // Create a new toggle object for field-robot centric switching mid match
        fieldCentric = Toggle(true)


    }

    /**
     * Method of teleop code to loop over every time
     */
    override fun loop() {
        super.loop()

        // Driving the mecanum base takes 3 joystick parameters: leftX, leftY, rightX.
        // These are related to the left stick x value, left stick y value, and
        // right stick x value respectively. These values are passed in to represent the
        // strafing speed, the forward speed, and the turning speed of the robot frame
        // respectively from [-1, 1].
        if (!isFieldCentric) {

            // For a robot centric model, the input of (0,1,0) for (leftX, leftY, rightX)
            // will move the robot in the direction of its current heading. Every movement
            // is relative to the frame of the robot itself.
            //
            //                 (0,1,0)
            //                   /
            //                  /
            //           ______/_____
            //          /           /
            //         /           /
            //        /___________/
            //           ____________
            //          /  (0,0,1)  /
            //         /     ↻     /
            //        /___________/

            // optional fourth parameter for squared inputs
            drive!!.driveRobotCentric(
                convertSpeed(driverOp!!.leftX),
                convertSpeed(driverOp!!.leftY),
                convertSpeed(driverOp!!.rightX),
                false
            )
        } else {

            // Below is a model for how field centric will drive when given the inputs
            // for (leftX, leftY, rightX). As you can see, for (0,1,0), it will travel forward
            // regardless of the heading. For (1,0,0) it will strafe right (ref to the 0 heading)
            // regardless of the heading.
            //
            //                   heading
            //                     /
            //            (0,1,0) /
            //               |   /
            //               |  /
            //            ___|_/_____
            //          /           /
            //         /           / ---------- (1,0,0)
            //        /__________ /

            // optional fifth parameter for squared inputs
            drive!!.driveFieldCentric(
                convertSpeed(driverOp!!.leftX),
                convertSpeed(driverOp!!.leftY),
                convertSpeed(driverOp!!.rightX),
                imu!!.rotation2d.degrees,  // gyro value passed in here must be in degrees
                false
            )
        }
        // Update what type of driving type is used
        updateDriveType()

    }

    /**
     * Check to see whether or not to change the driving type
     */
    private fun updateDriveType() {
        // Use the left bumper to switch whether or not the drive type is field centric
        fieldCentric!!.updateLeadingEdge(driverOp!!.getButton(GamepadKeys.Button.LEFT_BUMPER))
        isFieldCentric = fieldCentric!!.toggleState
    }

    private fun convertSpeed(input: Double): Double {
        return VEL_MULTIPLIER * input.pow(5.0) + input * (1 - VEL_MULTIPLIER)
    }

    companion object {
        /**
         * Constant to multiply velocities by to improve motion
         */
        private const val VEL_MULTIPLIER = 1
    }
}