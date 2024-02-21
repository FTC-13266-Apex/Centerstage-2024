package org.firstinspires.ftc.teamcode.opmode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Launcher;
import org.firstinspires.ftc.teamcode.subsystem.Outtake;
import org.firstinspires.ftc.teamcode.subsystem.Slides;

@TeleOp
public class FieldCentricMecanumTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        FieldCentricDrive drive = new FieldCentricDrive(this);
        //Arm arm = new Arm(this);
        //Claw claw = new Claw(this);
        //Climber climber= new Climber(this);
        Slides slides = new Slides(this);
        Intake intake = new Intake(this);
        Outtake outtake = new Outtake(this);
        Launcher launcher = new Launcher(this);
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            drive.teleOp();
            //arm.teleOp();
            //claw.teleOp();
            //climber.teleOp();
            slides.teleOp();
            intake.teleOp();
            outtake.teleOp();
            launcher.teleOp();
            telemetry.update();

        }
    }
}