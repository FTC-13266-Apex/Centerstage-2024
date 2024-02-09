package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            drive.teleOp();
            //arm.teleOp();
            //claw.teleOp();
            //climber.teleOp();
            slides.teleOp();
            intake.teleOp();
            telemetry.update();
        }
    }
}