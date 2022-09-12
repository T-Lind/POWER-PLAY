package org.firstinspires.ftc.teamcode.auto.differentialequationfollower;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.drive.StandardTrackingWheelLocalizer;

public class DifferentialEquationFollower {
    private DefineEquations equations;
    private StandardTrackingWheelLocalizer localizer;

    private Motor frontLeft;
    private Motor frontRight;
    private Motor backLeft;
    private Motor backRight;


    public DifferentialEquationFollower(DefineEquations equations, HardwareMap hardwareMap){
        this.equations = equations;
        localizer = new StandardTrackingWheelLocalizer(hardwareMap);

        frontLeft = new Motor(hardwareMap, "FL", Motor.GoBILDA.RPM_312);
        frontRight = new Motor(hardwareMap, "FR", Motor.GoBILDA.RPM_312);
        backLeft = new Motor(hardwareMap, "BL", Motor.GoBILDA.RPM_312);
        backRight = new Motor(hardwareMap, "BR", Motor.GoBILDA.RPM_312);

        frontLeft.setRunMode(Motor.RunMode.RawPower);
        frontRight.setRunMode(Motor.RunMode.RawPower);
        backLeft.setRunMode(Motor.RunMode.RawPower);
        backRight.setRunMode(Motor.RunMode.RawPower);

        frontLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }

    public void follow(){
        ElapsedTime currentTime = new ElapsedTime();

        PIDFController xPid = new PIDFController(3, 0.5, 0.5, 0.5);
        PIDFController yPid = new PIDFController(3, 0.5, 0.5, 0.5);
        PIDFController headingPid = new PIDFController(3, 0.5, 0.5, 0.5);

        while(true){
            double targetX = equations.getX(currentTime.milliseconds()/1000);
            double targetY = equations.getY(currentTime.milliseconds()/1000);
            double targetHeading = equations.getHeading(currentTime.milliseconds()/1000);

            double currentX = localizer.getPoseEstimate().getX();
            double currentY = localizer.getPoseEstimate().getY();
            double currentHeading = localizer.getPoseEstimate().getHeading();

//            xPid.calculate(currentX, targetX)
        }
    }
}
