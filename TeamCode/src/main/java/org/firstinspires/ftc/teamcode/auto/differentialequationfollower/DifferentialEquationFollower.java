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

    double lastTime;
    double lastVelMeasurement;

    double endX;
    double endY;
    double endHeading;

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

        lastTime = 0;
        lastVelMeasurement = 0;

        double time = 0.1;
        while(true){
            double xPos = equations.getX(time);
            double yPos = equations.getY(time);
            double heading = equations.getHeading(time);

            if(xPos == Double.MIN_VALUE && yPos == Double.MIN_VALUE && heading == Double.MIN_VALUE){
                endX = xPos;
                endY = yPos;
                endHeading = heading;
                break;
            }

            time += 0.001;
        }
    }

    public void follow(){
        ElapsedTime currentTime = new ElapsedTime();

        PIDFController xPid = new PIDFController(3, 0.5, 0.5, 0.5);
        PIDFController yPid = new PIDFController(3, 0.5, 0.5, 0.5);
        PIDFController headingPid = new PIDFController(3, 0.5, 0.5, 0.5);

        lastTime = currentTime.milliseconds()/1000;

        while(true){
            // Grab what the x, y, and heading should be at this instant in time
            double targetX = equations.getX(currentTime.milliseconds()/1000);
            double targetY = equations.getY(currentTime.milliseconds()/1000);
            double targetHeading = equations.getHeading(currentTime.milliseconds()/1000);

            // Get the current x, y, and heading
            double currentX = localizer.getPoseEstimate().getX();
            double currentY = localizer.getPoseEstimate().getY();
            double currentHeading = localizer.getPoseEstimate().getHeading();

            // Get the x, y, and heading corrections thru PIDF
            double xCorrect = xPid.calculate(currentX, targetX);
            double yCorrect = yPid.calculate(currentY, targetY);
            double headingCorrect = headingPid.calculate(currentHeading, targetHeading);

            // Get the heading the bot is going to be travelling in and the linear velocity it should be taking
            double headingToMove = Math.atan2(xCorrect, yCorrect);
            double linearVelocity = (Math.hypot(xCorrect, yCorrect)-lastVelMeasurement)/(currentTime.milliseconds()/1000-lastTime);

            // TODO: Figure out the set of equations to convert this data into motor powers

            lastTime = currentTime.milliseconds()/1000;

            if(errorIsSmall(endX, endY, endHeading, currentX, currentY, currentHeading)){
                break;
            }
        }

        frontLeft.set(0);
        frontRight.set(0);
        backLeft.set(0);
        backRight.set(0);
    }

    private boolean errorIsSmall(double endX, double endY, double endHeading, double currentX, double currentY, double currentHeading){
        double xError = Math.abs(endX-currentX);
        double yError = Math.abs(endY-currentY);
        double headingError = Math.abs(endHeading-currentHeading)/360;

        // TODO: Figure out if dividing by 360 is enough to 'normalize'
        //  the heading error to the same scale the x and y error is
        return (xError + yError + headingError) / 3 < 0.1;
    }
}
