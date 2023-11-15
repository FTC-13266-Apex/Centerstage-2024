package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp
public class Arm  {
   private final DcMotor armMotor;
    private final Gamepad gamepad2;
    private final Telemetry telemetry;
    private final Servo tClaw;
    // Position of the arm when it's lifted
    int armUpPosition = -700;

    // Position of the arm when it's down
    int armDownPosition = -35;
    public Arm(OpMode opMode) {
        HardwareMap hardwareMap = opMode.hardwareMap;
        tClaw = (Servo) hardwareMap.get("tClaw");
        tClaw.setDirection(Servo.Direction.FORWARD);

        // Find a motor in the hardware map named "Arm Motor
        telemetry = opMode.telemetry;
        armMotor = hardwareMap.dcMotor.get("Arm Motor");


        // Reset the motor encoder so that it reads zero ticks
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Sets the starting position of the arm to the down position
        armMotor.setTargetPosition(armDownPosition);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

       // waitForStart();
        gamepad2 = opMode.gamepad2;



            


    }
    void teleOp(){
        //Arm(HardwareMap, hardwareMap, Gamepad ,gamepad2) {
        // If the A button is pressed, raise the arm
        if (gamepad2.x) {
            tClaw.setPosition(-0.7);
            armMotor.setTargetPosition(armUpPosition);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor.setPower(0.5);
        }

        // If the B button is pressed, lower the arm
        if (gamepad2.y) {
            tClaw.setPosition(0.2);
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

    }



}