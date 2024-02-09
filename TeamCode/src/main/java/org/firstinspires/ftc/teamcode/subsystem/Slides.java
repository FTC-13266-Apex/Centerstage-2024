package org.firstinspires.ftc.teamcode.subsystem;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Slides  {
    private final DcMotor leftSlide;
    private final DcMotor rightSlide;
    private final Gamepad gamepad2;
    private final Telemetry telemetry;

    public static int intake = 0;
    public static int low = 1;
    public static int mid = 2;
    public static int high = 3;
    public static double multiplier = 10;
    public static double power = 0.5;
    int targetPosition = intake;

    public Slides (OpMode opMode) {
        HardwareMap hardwareMap = opMode.hardwareMap;

        // Find a motor in the hardware map named "Arm Motor
        telemetry = opMode.telemetry;
        leftSlide = hardwareMap.dcMotor.get("leftSlide");
        rightSlide = hardwareMap.dcMotor.get("rightSlide");


        // Reset the motor encoder so that it reads zero ticks
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Sets the starting position of the arm to the down position
        leftSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        rightSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        leftSlide.setTargetPosition(intake);
        rightSlide.setTargetPosition(intake);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlide.setPower(power);
        rightSlide.setPower(power);
        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        gamepad2 = opMode.gamepad2;
    }

    public void teleOp(){
        // Get the current position of the armMotor
        double leftPosition = leftSlide.getCurrentPosition();
        double rightPosition = rightSlide.getCurrentPosition();

        // Get the target position of the armMotor
        double leftTarget = leftSlide.getTargetPosition();
        double rightTarget = rightSlide.getTargetPosition();

        // Show the position of the armMotor on telemetry
        telemetry.addData("Encoder Position", leftPosition);
        telemetry.addData("Encoder Position", rightPosition);

        // Show the target position of the armMotor on telemetry
        telemetry.addData("Desired Position", leftTarget);
        telemetry.addData("Desired Position", rightTarget);


        leftSlide.setTargetPosition(targetPosition);
        rightSlide.setTargetPosition(targetPosition);

        if (gamepad2.a) {
            targetPosition= intake;
        }
        if (gamepad2.b) {
            targetPosition = low;
        }
        if (gamepad2.y) {
            targetPosition = mid;
        }
        if (gamepad2.x) {
            targetPosition = high;
        }
        if (gamepad2.right_stick_y > 0.1 || gamepad2.right_stick_y < -0.1) {
            targetPosition += (int) (multiplier * gamepad2.right_stick_y);
        }
    }
}