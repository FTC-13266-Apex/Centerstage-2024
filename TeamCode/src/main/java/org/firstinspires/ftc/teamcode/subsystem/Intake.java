package org.firstinspires.ftc.teamcode.subsystem;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class Intake  {
    private final DcMotor intake;
    private final Gamepad gamepad2;

    public static double power = 1;


    public Intake (OpMode opMode) {
        HardwareMap hardwareMap = opMode.hardwareMap;

        // Find a motor in the hardware map named Intake
        intake = hardwareMap.dcMotor.get("intake");

        //Set up motor
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        gamepad2 = opMode.gamepad2;
    }

    public void teleOp(){


        if (gamepad2.right_trigger > 0.25) {
            intake.setPower(power);
        }
        else if (gamepad2.left_trigger > 0.25) {
            intake.setPower(-power);
        }
        else {
            intake.setPower(0);
        }
    }
}