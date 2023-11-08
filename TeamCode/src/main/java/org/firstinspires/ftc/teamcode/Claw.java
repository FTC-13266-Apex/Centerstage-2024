package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

    public class Claw {
        private Servo lClaw, rClaw;
        private final Gamepad gamepad2;
        public Claw(HardwareMap hardwareMap, Gamepad gamepad2) {
            lClaw = (Servo) hardwareMap.get("lClaw");
            rClaw = (Servo) hardwareMap.get("rClaw");
            this.gamepad2 = gamepad2;

            lClaw.setDirection(Servo.Direction.REVERSE);
            rClaw.setDirection(Servo.Direction.FORWARD);
            clawServo(0.42,0.42);
        }
        public void teleOp() {
            //TODO Change position left trigger outtake
            if (gamepad2.dpad_down) clawServo(0.59, 0.59);
            else if (gamepad2.dpad_left) clawServo(0.4,0.4);
        }
        public void clawServo(double setPositionRight, double setPositionLeft) {
            rClaw.setPosition(setPositionRight);
            lClaw.setPosition(setPositionLeft);
        }
    }

