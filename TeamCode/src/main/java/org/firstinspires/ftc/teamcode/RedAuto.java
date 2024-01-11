package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@Autonomous
public class RedAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Drive drive = new Drive(this);
        Arm arm = new Arm(this);
        Claw claw = new Claw(this);
        Climber climber= new Climber(this);

        waitForStart();
        drive.encoderDrive(0.5, 24, 24);

    }
}