package org.firstinspires.ftc.teamcode.coyote.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.coyote.auto.SampleDifferentialSwerve;
import org.firstinspires.ftc.teamcode.coyote.teleop.container.TeleopContainer;

@TeleOp
public class BaseOp extends TeleopContainer {
    private SampleDifferentialSwerve drive;

    @Override
    public void init() {
        drive = new SampleDifferentialSwerve(hardwareMap);
        initMechanisms();
    }

    @Override
    public void loop() {
        updateDrivetrain(drive);
    }
}
