package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

    @Config
    public class Outtake {
        private final Gamepad gamepad1;
        private final Telemetry telemetry;

        private final CRServo intakeServo;

        public Outtake (OpMode opMode) {
            HardwareMap hardwareMap = opMode.hardwareMap;

            intakeServo = hardwareMap.crservo.get("intakeServo");

            telemetry = opMode.telemetry;

            gamepad1 = opMode.gamepad1;
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
        }
    }