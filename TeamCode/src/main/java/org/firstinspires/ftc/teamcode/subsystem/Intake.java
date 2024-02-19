package org.firstinspires.ftc.teamcode.subsystem;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Intake  {
    private final DcMotor intake;
    private final Gamepad gamepad1;

    public static double INTAKE_POWER = 0.3;


    public Intake (OpMode opMode) {
        HardwareMap hardwareMap = opMode.hardwareMap;

        // Find a motor in the hardware map named Intake
        intake = hardwareMap.dcMotor.get("intake");

        //Set up motor
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        gamepad1 = opMode.gamepad1;
    }

    public void teleOp(){


        if (gamepad1.right_trigger > 0.25) {
            intake.setPower(INTAKE_POWER);
        }
        else if (gamepad1.left_trigger > 0.25) {
            intake.setPower(-INTAKE_POWER);
        }
        else {
            intake.setPower(0);
        }
    }
}