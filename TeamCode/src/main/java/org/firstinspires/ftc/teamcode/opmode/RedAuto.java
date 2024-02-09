package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystem.FieldCentricDrive;

@Autonomous
public class RedAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        FieldCentricDrive drive = new FieldCentricDrive(this);
        //Arm arm = new Arm(this);
        //Claw claw = new Claw(this);
        //Climber climber= new Climber(this);

        waitForStart();
        drive.encoderDrive(0.5, 24, 24);

    }
}