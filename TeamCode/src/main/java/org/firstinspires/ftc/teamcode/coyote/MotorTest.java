package org.firstinspires.ftc.teamcode.coyote;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.coyote.coyote.SampleDifferentialSwerve;

/**
 * A testing program to invert the appropriate motors. The intended behavior is that both pods
 * rotate counter-clockwise and do not translate at all. If this is not the case, go to
 * SampleDifferentialSwerve and invert the appropriate motors until this class works as intended.
 */
@Config
@Disabled
public class MotorTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleDifferentialSwerve drive = new SampleDifferentialSwerve(hardwareMap);

        ElapsedTime timing = new ElapsedTime();


        while(timing.milliseconds()/1000 < 5) drive.setMotorPowers(0.5, 0.5, 0.5, 0.5);
        drive.stop();
    }
}
