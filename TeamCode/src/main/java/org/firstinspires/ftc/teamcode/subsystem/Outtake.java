package org.firstinspires.ftc.teamcode.subsystem;

import android.drm.DrmStore;

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
        final double intakeLeftPos = 0.935;
        final double intakeRightPos = 0.06;
        final double outtakeLeftPos = 0.68;
        final double outtakeRightPos = 0.32;

        double leftTarget = intakeLeftPos;
        double rightTarget = intakeRightPos;

        public Outtake (OpMode opMode) {
            HardwareMap hardwareMap = opMode.hardwareMap;
            telemetry = opMode.telemetry;

            intakeServo = hardwareMap.crservo.get("intakeServo");
            leftServo = hardwareMap.servo.get("leftServo");
            rightServo = hardwareMap.servo.get("rightServo");

            gamepad1 = opMode.gamepad1;
            gamepad2 = opMode.gamepad2;

            leftServo.setPosition(1);
            rightServo.setPosition(0);

        }

        public void teleOp() {

            leftServo.setPosition(leftTarget);
            rightServo.setPosition(rightTarget);

            if (gamepad1.right_trigger >0.1){

                intakeServo.setPower(1);

            } else if (gamepad1.left_trigger >0.1) {

                intakeServo.setPower(-1);
            }
            else {
                intakeServo.setPower(0);
            }
            if (gamepad2.a){
                leftTarget = intakeLeftPos;
                rightTarget = intakeRightPos;

            } else if (gamepad2.b || gamepad2.x || gamepad2.y) {
                leftTarget = outtakeLeftPos;
                rightTarget = outtakeRightPos;
            }
            if (gamepad2.left_stick_y >0.1 || gamepad2.left_stick_y <-0.1){
                leftServo.setPosition((leftServo.getPosition())+gamepad2.left_stick_y/1000);
                rightServo.setPosition((rightServo.getPosition())-gamepad2.left_stick_y/1000);
            }
            telemetry.addData("leftservo",leftServo.getPosition());
            telemetry.addData("rightservo",rightServo.getPosition());
        }
    }

