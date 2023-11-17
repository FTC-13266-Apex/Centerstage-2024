package org.firstinspires.ftc.teamcode.opmode.sample.roadrunner.advanced;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class NoRoadrunnerTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor leftFront = (DcMotor) hardwareMap.get("leftFront");
        DcMotor frontRight = (DcMotor) hardwareMap.get("rightFront");
        DcMotor backLeft = (DcMotor) hardwareMap.get("leftRear");
        DcMotor backRight = (DcMotor) hardwareMap.get("rightRear");
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, Y stick is reversed!
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            leftFront.setPower(y + x + rx);
            backLeft.setPower(y - x + rx);
            frontRight.setPower(y - x - rx);
            backRight.setPower(y + x - rx);
        }



    }
}
