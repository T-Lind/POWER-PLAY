package org.firstinspires.ftc.teamcode.coyote.auto;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.coyote.teleop.container.PoseStorage;
import org.firstinspires.ftc.teamcode.roadrunner.drive.StandardTrackingWheelLocalizer;

/**
 * A class to control the differential swerve. Can turn the entire robot or follow a multivariable
 * path function. Things to change are as follows:
 *
 * ENCODER IMPLEMENTATION - edit the getLeftPodAngle and getRightPodAngle methods to whatever implementation
 * you have to measure the pod angles. There is example code for REV encoders and MA-3 absolute encoders.
 * The output should be in degrees and 0 degrees is forward, -180 is CCW and 180 is CW
 *
 * TRACK WIDTH - make sure this is in meters - it should be the distance between the two driving wheels
 *
 * MOTOR OBJECTS - change the constructor calls of the Motor class  in the constructor of this class
 * to suit your drivetrain - motor IDs and RPM/motor types, frontLeft translates loosely to the left top pod,
 * backLeft translates loosely to the left bottom pod, frontRight to the right bottom pod and
 * backRight to the right top pod. Additionally, invert the appropriate motors - use the MotorTest
 * class to help you invert the appropriate ones
 *
 * PIDF tuning - tune the PIDF objects for the pod heading PID values and x/y PID values
 */
public class SampleDifferentialSwerve {
    // TODO: Change these objects to keep track of pod angles based on your system
    private Motor leftPodEncoder;
    private Motor rightPodEncoder;

    // TODO: Change the track width to whatever the track width is on your differential swerve IN M
    private static final double TRACK_WIDTH = 0.045;

    private Motor frontLeft;
    private Motor frontRight;
    private Motor backLeft;
    private Motor backRight;

    private StandardTrackingWheelLocalizer localizer;

    private PIDFController leftPodPIDF;
    private PIDFController rightPodPIDF;

    private PIDFController xPid;
    private PIDFController yPid ;
    private PIDFController headingPid;

    private double FLP;
    private double FRP;
    private double BLP;
    private double BRP;

    private HardwareMap hardwareMap;

    public SampleDifferentialSwerve(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;

        localizer = new StandardTrackingWheelLocalizer(hardwareMap);
        localizer.setPoseEstimate(PoseStorage.currentPose);

        // TODO: Edit the ID tags based on what your configuration says
        frontLeft = new Motor(hardwareMap, "FL", Motor.GoBILDA.RPM_312);
        frontRight = new Motor(hardwareMap, "FR", Motor.GoBILDA.RPM_312);
        backLeft = new Motor(hardwareMap, "BL", Motor.GoBILDA.RPM_312);
        backRight = new Motor(hardwareMap, "BR", Motor.GoBILDA.RPM_312);

        frontLeft.setRunMode(Motor.RunMode.RawPower);
        frontRight.setRunMode(Motor.RunMode.RawPower);
        backLeft.setRunMode(Motor.RunMode.RawPower);
        backRight.setRunMode(Motor.RunMode.RawPower);

        // TODO: Invert motors until MotorTest works as intended

        frontLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        // TODO: Instantiate objects to keep track of pod angles

        // Default implementation - REV encoders
        leftPodEncoder = new Motor(hardwareMap, "FR", Motor.GoBILDA.RPM_312);
        rightPodEncoder = new Motor(hardwareMap, "FL", Motor.GoBILDA.RPM_312);
        leftPodEncoder.setRunMode(Motor.RunMode.PositionControl);
        rightPodEncoder.setRunMode(Motor.RunMode.PositionControl);

        createTunedPIDF();
    }

    public void createTunedPIDF(){
        // TODO: Edit all these values to work with your drivetrain - will require a good amount of tuning!

        // Pod turning PIDF
        leftPodPIDF = new PIDFController(0.05, 0, 0.001, 0.0);
        rightPodPIDF = new PIDFController(0.065, 0, 0.0015, 0.001);


        // X, Y, and heading PIDF
        xPid = new PIDFController(3, 0.5, 0.5, 0.5);
        yPid = new PIDFController(3, 0.5, 0.5, 0.5);
        headingPid = new PIDFController(3, 0.5, 0.5, 0.5);
    }

    public double getLeftPodAngle(){
        // TODO: Define a method to return the left pod angle - -180 to 180 degrees, 0 degrees is
        //  straight forward

        // Default implementation - REV encoders
        return leftPodEncoder.getDistance()/8192.0*360;
    }

    public double getRightPodAngle(){
        // TODO: Define a method to return the right pod angle - -180 to 180 degrees, 0 degrees is
        //  straight forward

        // Default implementation - REV encoders
        return rightPodEncoder.getDistance()/8192.0*360;
    }

    public void correctWheelAngles(double leftAngle, double rightAngle, double setAngle){
        double powerLeftVal = leftPodPIDF.calculate(leftAngle, setAngle);
        double powerRightVal = rightPodPIDF.calculate(rightAngle, setAngle);

        FLP += powerLeftVal;
        BLP += powerLeftVal;
        FRP += powerRightVal;
        BRP += powerRightVal;
    }

    public void setMotorPowers(double flp, double blp, double frp, double brp){
        frontLeft.set(flp);
        frontRight.set(blp);
        backLeft.set(frp);
        backRight.set(brp);
    }

    public void addMotorPowers(double flp, double blp, double frp, double brp){
        FLP += flp;
        BLP += blp;
        FRP += frp;
        BRP += brp;
    }

    public void updateMotorPowers(){
//        double max = Math.max(Math.max(FLP, FRP), Math.max(BLP, BRP));
//        FLP /= max;
//        BRP /= max;
//        BLP /= max;
//        FRP /= max;

        frontLeft.set(FLP);
        frontRight.set(FRP);
        backLeft.set(BLP);
        backRight.set(BRP);

        FLP = 0;
        BLP = 0;
        FRP = 0;
        BRP = 0;
    }

    public void turn(double degrees){
//        PIDFController turnPIDF = new PIDFController();
    }

    public void followEquations(DefineEquations equations, double startHeading){
        ElapsedTime currentTime = new ElapsedTime();

        PIDFController xPid = new PIDFController(3, 0.5, 0.5, 0.5);
        PIDFController yPid = new PIDFController(3, 0.5, 0.5, 0.5);
        PIDFController headingPid = new PIDFController(3, 0.5, 0.5, 0.5);

        double lastTime = currentTime.milliseconds()/1000;
        double lastDistanceMeasurement = 0;

        double endX;
        double endY;

        double time = 0.1;
        while(true){
            double xPos = equations.getX(time);
            double yPos = equations.getY(time);

            if(xPos == Double.MIN_VALUE && yPos == Double.MIN_VALUE){
                endX = xPos;
                endY = yPos;
                break;
            }

            time += 0.001;
        }

        while(true){
            // Grab what the x, y, and heading should be at this instant in time
            double targetX = equations.getX(currentTime.milliseconds()/1000);
            double targetY = equations.getY(currentTime.milliseconds()/1000);

            // Get the current x, y, and heading
            double currentX = localizer.getPoseEstimate().getX();
            double currentY = localizer.getPoseEstimate().getY();
            double currentHeading = localizer.getPoseEstimate().getHeading();

            // Get the x, y, and heading corrections thru PIDF
            double xCorrect = xPid.calculate(currentX, targetX);
            double yCorrect = yPid.calculate(currentY, targetY);
            double headingCorrect = headingPid.calculate(currentHeading, startHeading);

            // Get the heading the bot is going to be travelling in and the linear velocity it should be taking
            double headingToMove = Math.atan2(xCorrect, yCorrect);
            double distance = Math.hypot(xCorrect, yCorrect)-lastDistanceMeasurement;
            double linearVelocity = distance/(currentTime.milliseconds()/1000-lastTime);

            // TODO: Figure out the set of equations to convert this data into motor powers

            lastDistanceMeasurement = distance;
            lastTime = currentTime.milliseconds()/1000;

            if(errorIsSmall(endX, endY, startHeading, currentX, currentY, currentHeading)){
                break;
            }
        }

        setMotorPowers(0, 0, 0, 0);
    }

    private boolean errorIsSmall(double endX, double endY, double endHeading, double currentX, double currentY, double currentHeading){
        double xError = Math.abs(endX-currentX);
        double yError = Math.abs(endY-currentY);
        double headingError = Math.abs(endHeading-currentHeading)/360;

        // TODO: Figure out if dividing by 360 is enough to 'normalize'
        //  the heading error to the same scale the x and y error is
        return (xError + yError + headingError) / 3 < 0.1;
    }

    public void stop(){
        setMotorPowers(0, 0, 0, 0);
    }

    public double getX(){
        return localizer.getPoseEstimate().getX()*0.0254;
    }

    public double getY(){
        return localizer.getPoseEstimate().getY()*0.0254;
    }

    public double getHeading(){
        return Math.toDegrees(localizer.getPoseEstimate().getHeading());
    }

    private PIDFController copyPIDF(PIDFController input){
        return new PIDFController(input.getP(), input.getI(), input.getD(), input.getF());
    }
}
