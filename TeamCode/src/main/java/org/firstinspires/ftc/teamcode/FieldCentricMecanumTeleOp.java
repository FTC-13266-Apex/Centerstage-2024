package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class FieldCentricMecanumTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        FeildCentricDrive drive = new FeildCentricDrive(this);
        //Arm arm = new Arm(this);
        //Claw claw = new Claw(this);
        //Climber climber= new Climber(this);
        Slides slides = new Slides(this);
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            drive.teleOp();
            //arm.teleOp();
            //claw.teleOp();
            //climber.teleOp();
            slides.teleOp();
            telemetry.update();
        }
    }
}