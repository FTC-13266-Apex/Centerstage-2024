package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Launcher {
    private final Gamepad gamepad1;
    private final Telemetry telemetry;
    private final Servo planeServo;
    public Launcher (OpMode opMode) {
        HardwareMap hardwareMap = opMode.hardwareMap;
        planeServo = hardwareMap.servo.get("PlaneServo");

        gamepad1 = opMode.gamepad1;
        telemetry = opMode.telemetry;

        planeServo.setPosition(0);

    }
    public void teleOp(){

        if (gamepad1.right_bumper){
            planeServo.setPosition(1);
        } else if (gamepad1.left_bumper) {
            planeServo.setPosition(0);
        }

    }
}
