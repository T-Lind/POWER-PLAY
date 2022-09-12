package org.firstinspires.ftc.teamcode.teleop.support

import com.arcrobotics.ftclib.controller.PIDController
import com.arcrobotics.ftclib.controller.PIDFController
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.arcrobotics.ftclib.hardware.motors.Motor
import org.firstinspires.ftc.teamcode.teleop.Toggle

@Strictfp
abstract class DiffyTeleOpContainer : UniversalTeleopContainer() {
    /**
     * Toggle object to store whether or not the drivetrain is in field centric mode
     */
    @Transient
    private var highControl: Toggle? = null

    @Transient
    private var mecTypeControl: Toggle? = null

    val VEL_COEFF = 4.0

    private var leftPodEncoder: Motor? = null
    private var rightPodEncoder: Motor? = null

    private var leftPodPIDF = PIDController(0.1, 0.0, 0.0)
    private var rightPodPIDF = PIDController(0.1, 0.0, 0.0)


    private fun getPodAngle(pod: String): Double {
        return if (pod == "left") {
            leftPodEncoder!!.distance / 8192.0 * 360
        } else if (pod == "right") {
            rightPodEncoder!!.distance / 8192.0 * 360
        } else {
            throw IllegalArgumentException("Improper argument specified!")
        }
    }

    override fun init() {
        super.init()

        // Run using motor power - change to .VelocityControl when using encoders
        frontLeft.setRunMode(Motor.RunMode.RawPower)
        frontRight.setRunMode(Motor.RunMode.RawPower)
        backLeft.setRunMode(Motor.RunMode.RawPower)
        backRight.setRunMode(Motor.RunMode.RawPower)

        highControl = Toggle(false)
        mecTypeControl = Toggle(false)

//        leftPodEncoder = Motor(hardwareMap, "FR", Motor.GoBILDA.RPM_312)
//        rightPodEncoder = Motor(hardwareMap, "FL", Motor.GoBILDA.RPM_312)
//
//        leftPodEncoder!!.setRunMode(Motor.RunMode.PositionControl)
//        rightPodEncoder!!.setRunMode(Motor.RunMode.PositionControl)
    }

    override fun loop() {
        super.loop()

        mecTypeControl!!.updateLeadingEdge(driverOp!!.getButton(GamepadKeys.Button.X))

        // get the x and y values - reorient y to follow unit circle, of both sticks
        var left_stick_x = gamepad1.left_stick_x.toDouble()
        var left_stick_y = (-1 * gamepad1.left_stick_y).toDouble()
        var right_stick_x = gamepad1.right_stick_x.toDouble()
        var right_stick_y = (-1 * gamepad1.right_stick_y).toDouble()


//        var FLP: Double = 0.0
//        var FRP: Double = 0.0
//        var BLP: Double = 0.0
//        var BRP: Double = 0.0
//
//        // VARIABLE TO REDUCE CODE
//
//        // VARIABLE TO REDUCE CODE
//        val driveSimilarity: Double = left_stick_x * -VEL_COEFF + left_stick_y * VEL_COEFF
//
//        // HIGH DEGREE OF USER CONTROL
//
//        if (mecTypeControl!!.toggleState) {
//            telemetry.update()
//
//            var stickAngle: Double = Math.atan2(-left_stick_x, left_stick_y)
//            if(left_stick_x == 0.0 && right_stick_x == 0.0)
//                stickAngle = 0.0
//
//            var leftPodAngle: Double = getPodAngle("left")
//            var rightPodAngle: Double = getPodAngle("right")
//
//
//
//
//            frontLeft.set(FLP)
//            backLeft.set(BLP)
//            frontRight.set(FRP)
//            backRight.set(BRP)
//        } else {
//            FLP = driveSimilarity
//            BLP = left_stick_x * -VEL_COEFF - left_stick_y * 2
//            FRP = right_stick_x * -VEL_COEFF - right_stick_y * VEL_COEFF
//            BRP = right_stick_x * -VEL_COEFF + right_stick_y * VEL_COEFF
//
//            val max = Math.max(Math.max(FLP, FRP), Math.max(BLP, BRP))
//            FLP /= max
//            BLP /= max
//            FRP /= max
//            BRP /= max
//
//            frontLeft.set(FLP)
//            backLeft.set(BLP)
//            frontRight.set(FRP)
//            backRight.set(BRP)
//
//        }
    }
}