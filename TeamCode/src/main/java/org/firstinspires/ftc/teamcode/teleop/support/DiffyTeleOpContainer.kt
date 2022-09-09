package org.firstinspires.ftc.teamcode.teleop.support

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

    val VEL_COEFF = 4.0

    private fun convert(p: Double): Double {
        if (p > 0) return Math.log(p + 1) + 1 else if (p < 0) return -(Math.log(Math.abs(p) + 1) + 1)
        return 0.0
    }

    override fun init() {
        super.init()

        // Run using motor power - change to .VelocityControl when using encoders
        frontLeft.setRunMode(Motor.RunMode.RawPower)
        frontRight.setRunMode(Motor.RunMode.RawPower)
        backLeft.setRunMode(Motor.RunMode.RawPower)
        backRight.setRunMode(Motor.RunMode.RawPower)

        highControl = Toggle(false)
    }

    override fun loop() {
        super.loop()

        highControl!!.updateLeadingEdge(driverOp!!.getButton(GamepadKeys.Button.A))


        // get the x and y values - reorient y to follow unit circle, of both sticks
        var left_stick_x = gamepad1.left_stick_x.toDouble()
        var left_stick_y = (-1 * gamepad1.left_stick_y).toDouble()
        var right_stick_x = gamepad1.right_stick_x.toDouble()
        var right_stick_y = (-1 * gamepad1.right_stick_y).toDouble()
        left_stick_x = convert(left_stick_x)
        left_stick_y = -convert(left_stick_y)
        right_stick_x = convert(right_stick_x)
        right_stick_y = -convert(right_stick_y)

        var FLP: Double
        var FRP: Double
        var BLP: Double
        var BRP: Double

        // VARIABLE TO REDUCE CODE

        // VARIABLE TO REDUCE CODE
        val driveSimilarity: Double = left_stick_x * -VEL_COEFF + left_stick_y * VEL_COEFF

        // HIGH DEGREE OF USER CONTROL


        // HIGH DEGREE OF USER CONTROL
        if (highControl!!.toggleState) {
            FLP = driveSimilarity
            BLP = left_stick_x * -VEL_COEFF - left_stick_y * 2
            FRP = right_stick_x * -VEL_COEFF - right_stick_y * VEL_COEFF
            BRP = right_stick_x * -VEL_COEFF + right_stick_y * VEL_COEFF

//            leftFront.setVelocity(driveSimilarity, AngleUnit.RADIANS);
//            leftBack.setVelocity(left_stick_x*-VEL_COEFF-left_stick_y*2, AngleUnit.RADIANS);
//
//            rightFront.setVelocity(right_stick_x*-VEL_COEFF-right_stick_y*VEL_COEFF, AngleUnit.RADIANS);
//            rightBack.setVelocity(right_stick_x*-VEL_COEFF+right_stick_y*VEL_COEFF, AngleUnit.RADIANS);
        } else {
            // set the velocity based on left stick
            FLP = driveSimilarity
            BLP = left_stick_x * -VEL_COEFF - left_stick_y * VEL_COEFF
            FRP = left_stick_y * -VEL_COEFF - left_stick_x * VEL_COEFF
            BRP = -left_stick_y * -VEL_COEFF - left_stick_x * VEL_COEFF

            // add turning motion
            FLP += right_stick_x * 0.5 * VEL_COEFF
            BLP += -right_stick_x * 0.5 * VEL_COEFF
            FRP += right_stick_x * 0.5 * VEL_COEFF
            BRP += -right_stick_x * 0.5 * VEL_COEFF
            if (left_stick_x > 0) {
                FLP *= -1.0
                FRP *= -1.0
                BLP *= -1.0
                BRP *= -1.0
            }
        }


        val max = Math.max(Math.max(FLP, FRP), Math.max(BLP, BRP))
        FLP /= max
        BLP /= max
        FRP /= max
        BRP /= max

        frontLeft.set(FLP * 0.5)
        backLeft.set(BLP * 0.5)
        frontRight.set(FRP * 0.5)
        backRight.set(BRP * 0.5)

    }
}