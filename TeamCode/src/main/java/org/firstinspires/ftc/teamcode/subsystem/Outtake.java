package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

    @Config
    public class Outtake {
        private final Gamepad gamepad1;
        private final Gamepad gamepad2;
        private final Telemetry telemetry;

        private final Servo leftServo;
        private final Servo rightServo;
        private final CRServo intakeServo;

        public Outtake (OpMode opMode) {
            HardwareMap hardwareMap = opMode.hardwareMap;

            intakeServo = hardwareMap.crservo.get("intakeServo");
            leftServo = hardwareMap.servo.get("leftServo");
            rightServo = hardwareMap.servo.get("rightServo");

            telemetry = opMode.telemetry;
            gamepad1 = opMode.gamepad1;
            gamepad2 = opMode.gamepad2;

            leftServo.setPosition(1);
            rightServo.setPosition(0);

        }

        public void teleOp() {

            if (gamepad1.right_trigger >0.1){

                intakeServo.setPower(1);

            } else if (gamepad1.left_trigger >0.1) {

                intakeServo.setPower(-1);
            }
            else {
                intakeServo.setPower(0);
            }
            if (gamepad2.a){
                leftServo.setPosition(1);
                rightServo.setPosition(0);
            } else if (gamepad2.b || gamepad2.x || gamepad2.y) {
                leftServo.setPosition(0.5);
                rightServo.setPosition(0.5);
            }
        }
    }
