package org.firstinspires.ftc.teamcode.teleop.support

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.arcrobotics.ftclib.drivebase.MecanumDrive
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.arcrobotics.ftclib.hardware.RevIMU
import com.arcrobotics.ftclib.gamepad.GamepadEx
import org.firstinspires.ftc.teamcode.teleop.Toggle
import org.firstinspires.ftc.teamcode.roadrunner.drive.StandardTrackingWheelLocalizer
import com.arcrobotics.ftclib.hardware.motors.Motor
import org.firstinspires.ftc.teamcode.teleop.PoseStorage
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.arcrobotics.ftclib.util.Timing
import kotlin.math.pow

@Strictfp
abstract class TeleOpContainer : OpMode() {
    /**
     * Drivetrain object from FTCLib
     */
    @Transient
    protected var drive: MecanumDrive? = null

    /**
     * Streamlined IMU object from FTCLib
     */
    @Transient
    protected var imu: RevIMU? = null

    /**
     * Gamepad object from FTCLib
     */
    @Transient
    protected var driverOp: GamepadEx? = null

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
     * Timer to keep track of the current time into the match
     */
    @Transient
    private var matchTimer: Timing.Timer? = null

    /**
     * Status variable to signal if the timer is started
     */
    @Transient
    private var matchTimerStarted = false

    /**
     * 3 wheel odo localizer
     */
    @Transient
    private var myLocalizer: StandardTrackingWheelLocalizer? = null

    /**
     * Current robot pose
     */
    @Transient
    private var myPose: Pose2d? = null

    /**
     * Method to get the field centric state variable for displaying in telemetry, etc.
     * @return If the drivetrain is in field centric mode or not
     */
    protected fun getFieldCentric(): Boolean {
        return isFieldCentric
    }

    /**
     * Method to get the remaining time used in the match for displaying in telemetry, etc.
     * @return The time remaining in the match
     */
    protected val remainingMatchTime: Long
        get() = matchTimer!!.remainingTime()

    /**
     * Initialize the motors, zero power behavior, drivetrain object, imu, and whatever other
     * hardware you want to initialize every time
     */
    override fun init() {

        // constructor takes in frontLeft, frontRight, backLeft, backRight motors
        // IN THAT ORDER
        val frontLeft = Motor(hardwareMap, "FL", Motor.GoBILDA.RPM_312)
        val frontRight = Motor(hardwareMap, "FR", Motor.GoBILDA.RPM_312)
        val backLeft = Motor(hardwareMap, "BL", Motor.GoBILDA.RPM_312)
        val backRight = Motor(hardwareMap, "BR", Motor.GoBILDA.RPM_312)

        // Sets zero power behavior to braking
        frontLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE)
        frontRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE)
        backLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE)
        backRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE)
        drive = MecanumDrive(
            frontLeft,
            frontRight,
            backLeft,
            backRight
        )

        // This is the built-in IMU in the REV hub.
        // We're initializing it by its default parameters
        // and name in the config ('imu'). The orientation
        // of the hub is important. Below is a model
        // of the REV Hub and the orientation axes for the IMU.
        //
        //                           | Z axis
        //                           |
        //     (Motor Port Side)     |   / X axis
        //                       ____|__/____
        //          Y axis     / *   | /    /|   (IO Side)
        //          _________ /______|/    //      I2C
        //                   /___________ //     Digital
        //                  |____________|/      Analog
        //
        //                 (Servo Port Side)
        //
        // (unapologetically stolen from the road-runner-quickstart)

        // Create the IMU object and initialize
        imu = RevIMU(hardwareMap)
        imu!!.init()

        // Create the extended gamepad object
        driverOp = GamepadEx(gamepad1)

        // Create a new toggle object for field-robot centric switching mid match
        fieldCentric = Toggle(true)


        // This is assuming you are using StandardTrackingWheelLocalizer.java
        // Switch this class to something else (Like TwoWheeTrackingLocalizer.java) if your
        // configuration differs
        myLocalizer = StandardTrackingWheelLocalizer(hardwareMap)

        // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details
        myLocalizer!!.poseEstimate = PoseStorage.currentPose


        // Create the new timer object to keep track of match time
        matchTimer = Timing.Timer(120)
        matchTimerStarted = false

        // Initialize the mechanisms of the robot
        initSpecificMechanisms()
    }

    protected val x: Double
        get() = myPose!!.x
    protected val y: Double
        get() = myPose!!.y

    protected fun getAngle(): Double {
        return myPose!!.heading
    }

    /**
     * Method of teleop code to loop over every time
     */
    override fun loop() {
        // Statement to only run once at the beginning of the loop
        if (!matchTimerStarted) {
            matchTimer!!.start()
            matchTimerStarted = true
        }


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

        // Alert the driver to the amount of time left through a voice command.
        alertToTiming()
        myLocalizer!!.update()

        // Retrieve your pose
        myPose = myLocalizer!!.poseEstimate

        // Update the mechanisms
        updateMechanisms()
    }

    /**
     * Check to see whether or not to change the driving type
     */
    private fun updateDriveType() {
        // Use the left bumper to switch whether or not the drive type is field centric
        fieldCentric!!.updateLeadingEdge(driverOp!!.getButton(GamepadKeys.Button.LEFT_BUMPER))
        isFieldCentric = fieldCentric!!.toggleState
    }

    /**
     * Alert the driver at different time intervals. Note that it takes a while to say the words
     * so they're started speaking ahead of where they should be.
     */
    private fun alertToTiming() {
        if (matchTimer!!.elapsedTime() == 30L) telemetry.speak("30 seconds into tele-op")

        else if (matchTimer!!.remainingTime() == 45L) telemetry.speak("15 seconds until end game")

        else if (matchTimer!!.remainingTime() == 5L) telemetry.speak("three")

        else if (matchTimer!!.remainingTime() == 3L) telemetry.speak("two")

        else if (matchTimer!!.remainingTime() == 1L) telemetry.speak("one")
    }

    private fun convertSpeed(input: Double): Double {
        return VEL_MULTIPLIER * input.pow(5.0) + input * (1 - VEL_MULTIPLIER)
    }

    /**
     * Abstract method to initialize specific mechanisms you want for a teleop
     */
    protected abstract fun initSpecificMechanisms()

    /**
     * Abstract method to update any mechanisms that you're running
     */
    protected abstract fun updateMechanisms()

    companion object {
        /**
         * Constant to multiply velocities by to improve motion
         */
        private const val VEL_MULTIPLIER = 1
    }
}