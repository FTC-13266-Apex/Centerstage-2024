package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class Arm extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Position of the arm when it's lifted
        int armUpPosition = 1000;

        // Position of the arm when it's down
        int armDownPosition = 0;

        // Find a motor in the hardware map named "Arm Motor"
        DcMotor armMotor = hardwareMap.dcMotor.get("Arm Motor");


        // Reset the motor encoder so that it reads zero ticks
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Sets the starting position of the arm to the down position
        armMotor.setTargetPosition(armDownPosition);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();

        while (opModeIsActive()) {
            // If the A button is pressed, raise the arm
            if (gamepad1.a) {
                armMotor.setTargetPosition(armUpPosition);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(0.5);
            }

            // If the B button is pressed, lower the arm
            if (gamepad1.b) {
                armMotor.setTargetPosition(armDownPosition);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(0.5);
            }

            // Get the current position of the armMotor
            double position = armMotor.getCurrentPosition();

            // Get the target position of the armMotor
            double desiredPosition = armMotor.getTargetPosition();

            // Show the position of the armMotor on telemetry
            telemetry.addData("Encoder Position", position);

            // Show the target position of the armMotor on telemetry
            telemetry.addData("Desired Position", desiredPosition);

            telemetry.update();
        }
    }
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

}