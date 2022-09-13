package org.firstinspires.ftc.teamcode.teleop.support

import com.arcrobotics.ftclib.controller.PIDController
import com.arcrobotics.ftclib.controller.PIDFController
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.arcrobotics.ftclib.hardware.motors.Motor
import org.firstinspires.ftc.teamcode.teleop.Toggle

@Strictfp
abstract class DiffyTeleOpContainer : UniversalTeleopContainer() {
    override fun init() {
        super.init()

        // Run using motor power - change to .VelocityControl when using encoders
        frontLeft.setRunMode(Motor.RunMode.RawPower)
        frontRight.setRunMode(Motor.RunMode.RawPower)
        backLeft.setRunMode(Motor.RunMode.RawPower)
        backRight.setRunMode(Motor.RunMode.RawPower)


    }
}