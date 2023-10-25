package org.firstinspires.ftc.teamcode.subsystem;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
public class Arm extends Subsystem {
    public static class Constants {
        public static Hardware hardware;
        public static Controller controller;
        public static Position position;
        public static Speed speed;

        public static class Hardware {
            public static DcMotorSimple.Direction LEFT_DIRECTION = DcMotorSimple.Direction.FORWARD;
//            public static DcMotorSimple.Direction RIGHT_DIRECTION = DcMotorSimple.Direction.REVERSE;
            public static double
                    RPM = 84,
                    CPR = 145.090909;

        }
        public static class Controller {
            public static double
                    P = 2,
                    I = 0,
                    D = 0,
                    F = 0;
            public static int TOLERANCE = 8;
        }
        
        public static class Position {
            public static int
                    HIGH = 1077,
                    MID = 760,
                    LOW = 437,
                    INITIAL = 0;
        }
        public static class Speed {
            public static double NORMAL        = 10;   // ill use this but im testing something
            public static int MANUAL_MOVE_SPEED = 10;

        }
    }
    
    private final DcMotorEx arm;
    private int motorPosition;
    private boolean isMovingManually;




    public Arm(@NonNull OpMode opMode) {
        super(opMode);
        arm = opMode.hardwareMap.get(DcMotorEx.class, "arm");
        arm.setDirection(Constants.Hardware.LEFT_DIRECTION);

        // TODO: Find out what these constants are by default
//        leftLift.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDFCoefficients(Constants.Controller.P, Constants.Controller.I, Constants.Controller.D, Constants.Controller.F));
//        leftLift.setTargetPositionTolerance(Constants.Controller.TOLERANCE);

        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setTargetPosition(0);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);



    }

    @Override
    protected void manualControl() {
        if (opMode.gamepad2.dpad_down) moveInitial();
        else if(opMode.gamepad2.dpad_left) moveLow();
        else if(opMode.gamepad2.dpad_right) moveMid();
        else if (opMode.gamepad2.dpad_up) moveHigh();

        if (opMode.gamepad2.right_stick_y < -0.2 || opMode.gamepad2.right_stick_y > 0.2) {
            moveMotors((int)(motorPosition + Constants.Speed.MANUAL_MOVE_SPEED * -opMode.gamepad2.right_stick_y));
            isMovingManually = true;
        } else {
            isMovingManually = false;
        }

        opMode.telemetry.addData("Slide position", motorPosition);
        opMode.telemetry.addData("Slide P", arm.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION).p);
        opMode.telemetry.addData("Slide I", arm.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION).i);
        opMode.telemetry.addData("Slide D", arm.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION).d);
        opMode.telemetry.addData("Slide F", arm.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION).f);
        opMode.telemetry.addData("Slide Tolerance", arm.getTargetPositionTolerance());
    }

    public void moveMotors(int position) {
//        if (position > Constants.Position.MAX_POSITION || position < Constants.Position.MIN_POSITION) return;

        this.motorPosition = position;

        arm.setTargetPosition(position);
        arm.setPower(1);
    }

    public void moveInitial() {
        if (isMovingManually) {
            Constants.Position.INITIAL = motorPosition;
        }
        moveMotors(Constants.Position.INITIAL);
    }

    public void moveHigh() {
        if (isMovingManually) {
            Constants.Position.HIGH = motorPosition;
        }
        moveMotors(Constants.Position.HIGH);
    }

    public void moveMid() {
        if (isMovingManually) {
            Constants.Position.MID = motorPosition;
        }
        moveMotors(Constants.Position.MID);
    }

    public void moveLow() {
        if (isMovingManually) {
            Constants.Position.LOW = motorPosition;
        }
        moveMotors(Constants.Position.LOW);
    }

    public boolean isDown() {
        return motorPosition <= Constants.Position.INITIAL;
    }

    public boolean isUp() {return motorPosition > Constants.Position.INITIAL;}
}