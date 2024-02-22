package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystem.Vision;
import org.firstinspires.ftc.teamcode.subsystem.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.container.LineToLinearHeading;

@Autonomous
@Config
public class RedAuto extends LinearOpMode {
//    public static LineToLinearHeading first = new LineToLinearHeading(0, 0, 0);

    @Override
    public void runOpMode() throws InterruptedException {

//        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        FieldCentricDrive drive = new FieldCentricDrive(this);
        Vision vision = new Vision(hardwareMap, telemetry);

        while(!isStarted()){
            vision.setPosition(vision.getPosition());
            telemetry.addData("Vision", vision.getFinalPosition());
            telemetry.update();
        }


        waitForStart();
        Vision.FFPosition position = vision.getFinalPosition();
//        drive.encoderDrive(0.5, 24, 24);
        switch (position){
            case LEFT:
                drive.encoderDrive(0.5, 24, 24);
            case RIGHT:
                drive.encoderDrive(0.5, -12, 12);
            case MIDDLE:
                drive.encoderDrive(0.5, -24, -24);
        }

    }
}