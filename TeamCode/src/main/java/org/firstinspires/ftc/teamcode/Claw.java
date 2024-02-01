/*
package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Claw {
    private final Servo oClaw;
    private final CRServo Intake;
    private final Gamepad gamepad2;

    private final Telemetry telemetry;

    public Claw(OpMode opMode) {
        HardwareMap hardwareMap = opMode.hardwareMap;
        this.gamepad2 = opMode.gamepad2;
        Intake = (CRServo) hardwareMap.get("Intake");
        oClaw = (Servo) hardwareMap.get("oClaw");
        telemetry = opMode.telemetry;

        oClaw.setDirection(Servo.Direction.FORWARD);
       Intake.setDirection(DcMotorSimple.Direction.FORWARD);
        oClaw.setPosition(0.87);
    }

    void teleOp () {

        double leftStickY = gamepad2.left_stick_y;
        double oclawPos = oClaw.getPosition();

        telemetry.addData("oClawPosition", oclawPos);

        if (leftStickY < 0.1 || leftStickY > -0.1) {

            oClaw.setPosition(leftStickY / 500 + oclawPos);
        }

        if (gamepad2.left_bumper) oClaw.setPosition(0.87);
        if (gamepad2.right_bumper) oClaw.setPosition (0.5);


     if (gamepad2.right_trigger > 0.2)  {
         Intake.setDirection(DcMotorSimple.Direction.FORWARD);
            Intake.setPower(gamepad2.right_trigger);
        }
     else if (gamepad2.left_trigger > 0.2) {
         Intake.setDirection(DcMotorSimple.Direction.REVERSE);
           Intake.setPower(gamepad2.left_trigger);
        }
     else {
         Intake.setPower(0);
     }


    }

}

*/
