package org.firstinspires.ftc.teamcode.opmode.tuner.lift;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.command.OptimizeStuff;
import org.firstinspires.ftc.teamcode.subsystem.Arm;

@TeleOp(group="a tuner")
public class LiftTuner extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Arm arm = new Arm(this);
        new OptimizeStuff(this);
        //telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {
            arm.runIteratively();
//            lift.updateDashboard();
            telemetry.update();
        }

    }
}
