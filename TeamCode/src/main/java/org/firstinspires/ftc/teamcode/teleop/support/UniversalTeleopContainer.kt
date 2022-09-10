package org.firstinspires.ftc.teamcode.teleop.support

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.arcrobotics.ftclib.drivebase.MecanumDrive
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.hardware.RevIMU
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.arcrobotics.ftclib.util.Timing
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.teamcode.roadrunner.drive.StandardTrackingWheelLocalizer
import org.firstinspires.ftc.teamcode.teleop.PoseStorage

@Strictfp
abstract class UniversalTeleopContainer : OpMode(){
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

    protected lateinit var frontLeft: Motor
    protected lateinit var frontRight: Motor
    protected lateinit var backLeft: Motor
    protected lateinit var backRight: Motor

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
        frontLeft = Motor(hardwareMap, "FL", Motor.GoBILDA.RPM_312)
        frontRight = Motor(hardwareMap, "FR", Motor.GoBILDA.RPM_312)
        backLeft = Motor(hardwareMap, "BL", Motor.GoBILDA.RPM_312)
        backRight = Motor(hardwareMap, "BR", Motor.GoBILDA.RPM_312)

        // Sets zero power behavior to braking
        frontLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE)
        frontRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE)
        backLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE)
        backRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE)

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

        // Alert the driver to the amount of time left through a voice command.
        alertToTiming()
        myLocalizer!!.update()

        // Retrieve your pose
        myPose = myLocalizer!!.poseEstimate

        // Update the mechanisms
        updateMechanisms()
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

    /**
     * Abstract method to initialize specific mechanisms you want for a teleop
     */
    protected abstract fun initSpecificMechanisms()

    /**
     * Abstract method to update any mechanisms that you're running
     */
    protected abstract fun updateMechanisms()
}