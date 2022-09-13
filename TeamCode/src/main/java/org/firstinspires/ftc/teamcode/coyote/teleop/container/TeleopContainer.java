package org.firstinspires.ftc.teamcode.coyote.teleop.container;

import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.roadrunner.drive.StandardTrackingWheelLocalizer;

abstract public class TeleopContainer extends OpMode {
    private Timing.Timer matchTimer;
    private RevIMU imu;
    private StandardTrackingWheelLocalizer myLocalizer;

    private boolean timerStarted = false;

    public void initMechanisms(){
        imu.init();
        myLocalizer = new StandardTrackingWheelLocalizer(hardwareMap);
    }

    public void updateTimer(){
        if(!timerStarted)
            matchTimer = new Timing.Timer(120);
            timerStarted = true;
        alertToTiming();
    }


    /**
     * Alert the driver at different time intervals. Note that it takes a while to say the words
     * so they're started speaking ahead of where they should be.
     */
    private void alertToTiming() {
        if (matchTimer.elapsedTime() == 30L) telemetry.speak("30 seconds into tele-op");

        else if (matchTimer.remainingTime() == 45L) telemetry.speak("15 seconds until end game");

        else if (matchTimer.remainingTime() == 5L) telemetry.speak("three");

        else if (matchTimer.remainingTime() == 3L) telemetry.speak("two");

        else if (matchTimer.remainingTime() == 1L) telemetry.speak("one");
    }
}
