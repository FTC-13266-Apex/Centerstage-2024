package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {
    private final Servo oClaw;
    private final Gamepad gamepad2;
    public Claw(OpMode opMode) {
        HardwareMap hardwareMap = opMode.hardwareMap;
        gamepad2 = opMode.gamepad2;
        oClaw = (Servo) hardwareMap.get("oClaw");


        oClaw.setDirection(Servo.Direction.REVERSE);
        oClaw.setPosition(0.4);
    }
    public void teleOp() {
        //TODO Change position left trigger outtake
        if (gamepad2.dpad_up) oClaw.setPosition(0.59);
        else if (gamepad2.dpad_right)oClaw.setPosition (0.4);
    }


}

