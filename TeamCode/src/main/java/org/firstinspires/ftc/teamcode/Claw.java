package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

    public class Claw {
        private Servo oClaw, tClaw;
        private final Gamepad gamepad2;
        public Claw(HardwareMap hardwareMap, Gamepad gamepad2) {
            oClaw = (Servo) hardwareMap.get("oClaw");
            tClaw = (Servo) hardwareMap.get("tClaw");
            this.gamepad2 = gamepad2;
            oClaw.setDirection(Servo.Direction.REVERSE);
            tClaw.setDirection(Servo.Direction.FORWARD);
            clawServo(0.42,0.42);
        }
        public void teleOp() {
            //TODO Change position left trigger outtake
            if (gamepad2.dpad_up) clawServo(0.59, 0.59);
            else if (gamepad2.dpad_right) clawServo(0.4,0.4);
        }
        public void clawServo(double setPositionOpen, double setPositionTurn) {
            oClaw.setPosition(setPositionOpen);
            tClaw.setPosition(setPositionTurn);
        }


    }

