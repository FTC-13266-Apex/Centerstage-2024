package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.command.OptimizeStuff;
import org.firstinspires.ftc.teamcode.command.Sensor;
import org.firstinspires.ftc.teamcode.command.Sussy;
import org.firstinspires.ftc.teamcode.subsystem.Drive;
import org.firstinspires.ftc.teamcode.subsystem.Gripper;
import org.firstinspires.ftc.teamcode.subsystem.Arm;
import org.firstinspires.ftc.teamcode.util.PoseStorage;

@TeleOp
public class TeleOpMain extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Arm arm = new Arm(this);
        Gripper gripper = new Gripper(this);
        // TODO: get this to false
        Drive drive = new Drive(this,true);

        new OptimizeStuff(this);
        new Sussy(this);

//        Sensor sensor = new Sensor(this, gripper, arm);

        drive.setPoseEstimate(PoseStorage.currentPose);
        gripper.open();
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {
            arm.runIteratively();
            //junction.runIteratively();
            gripper.runIteratively();


            drive.runIteratively();
            telemetry.update();
        }

    }
}
