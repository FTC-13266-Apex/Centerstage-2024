package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class Arm extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor armMotor = hardwareMap.dcMotor.get("Arm Motor");
        int pos = armMotor.getCurrentPosition();
        System.out.println(pos);
        int desiredPosition = 0; // The position (in ticks) that you want the motor to move to
        armMotor.setTargetPosition(desiredPosition); // Tells the motor that the position it should go to is desiredPosition
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // Position of the arm when it's lifted
        int armUpPosition = -700;
       // int arm middle Position = 0;
        // Position of the arm when it's down
        int armDownPosition = 0;

        // Find a motor in the hardware map named "Arm Motor"


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
                armMotor.setPower(0.8);
            }

            // If the B button is pressed, lower the arm
            if (gamepad1.b) {
                armMotor.setTargetPosition(armDownPosition);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(0.8);
            }
            if (gamepad1.y) {
                armMotor.setTargetPosition(armDownPosition);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(0.8);

                // Get the current position of the armMotor
                double position = armMotor.getCurrentPosition();

                // Get the target position of the armMotor
                //  double desiredPosition = armMotor.getTargetPosition();

                // Show the position of the armMotor on telemetry
                telemetry.addData("Encoder Position", position);

                // Show the target position of the armMotor on telemetry
                telemetry.addData("Desired Position", desiredPosition);

                telemetry.update();

            }
        }}{}}
