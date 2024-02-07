package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class Slides  {
    private final DcMotor leftSlide;
    private final DcMotor rightSlide;
    private final Gamepad gamepad2;
    private final Telemetry telemetry;

    int intake = 0;
    int low = 1;
    int mid = 2;
    int high = 3;
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
        leftSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        rightSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlide.setPower(0.5);
        rightSlide.setPower(0.5);
        leftSlide.setTargetPosition(intake);
        rightSlide.setTargetPosition(intake);
        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // waitForStart();
        gamepad2 = opMode.gamepad2;


    }

    void teleOp(){




        // Get the current position of the armMotor
        double leftposition = leftSlide.getCurrentPosition();
        double rightposition = rightSlide.getCurrentPosition();

        // Get the target position of the armMotor
        double lefttarget = leftSlide.getTargetPosition();
        double righttarget = rightSlide.getTargetPosition();

        // Show the position of the armMotor on telemetry
        telemetry.addData("Encoder Position", leftposition);
        telemetry.addData("Encoder Position", rightposition);

        // Show the target position of the armMotor on telemetry
        telemetry.addData("Desired Position", lefttarget);
        telemetry.addData("Desired Position", righttarget);


        if (gamepad2.a){
            leftSlide.setTargetPosition(intake);
            rightSlide.setTargetPosition(intake);
        }
        if (gamepad2.b){
            leftSlide.setTargetPosition(low);
            rightSlide.setTargetPosition(low);
        }
        if (gamepad2.y){
            leftSlide.setTargetPosition(mid);
            leftSlide.setTargetPosition(mid);
        }
        if (gamepad2.x){
            leftSlide.setTargetPosition(high);
            rightSlide.setTargetPosition(high);
        }
    }
}