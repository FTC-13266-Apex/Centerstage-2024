//package org.firstinspires.ftc.teamcode;

//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;


//@TeleOp
//public class Climber {
    //private final DcMotor climberMotor;
   // private final DcMotor climbMotorSeries;
   // private final Gamepad gamepad2;
    //private final Telemetry telemetry;

   // int clUpPosition = -700;
    // Position of the arm when it's down
   // int clDownPosition = 0;

    //  private final Servo tClaw;
    // Position of the arm when it's lifted
   // public Climber(OpMode opMode) {
     //   HardwareMap hardwareMap = opMode.hardwareMap;

        // Find a motor in the hardware map named "Arm Motor
       // telemetry = opMode.telemetry;
        //climberMotor = hardwareMap.dcMotor.get("climb motor");
        //climbMotorSeries = hardwareMap.dcMotor.get("climber motor");


        // Reset the motor encoder so that it reads zero ticks
        //climberMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //climbMotorSeries.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Sets the starting position of the arm to the down position
        //climberMotor.setTargetPosition(clUpPosition);
        //climbMotorSeries.setTargetPosition(clUpPosition);

        //climberMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //climbMotorSeries.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // waitForStart();
        //gamepad2 = opMode.gamepad2;


   // }

   // void teleOp() {

        //if (gamepad2.dpad_left) {
            // tClaw.setPosition(0.59);
           // climberMotor.setTargetPosition(clUpPosition);
         ///   climberMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
           // climberMotor.setPower(0.5);

           // climbMotorSeries.setTargetPosition(clUpPosition);
           // climbMotorSeries.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         //   climbMotorSeries.setPower(0.5);

       // }

        // If the B button is pressed, lower the arm
        //if (gamepad2.dpad_right) {
            // tClaw.setPosition(0.4);
           // climberMotor.setTargetPosition(clDownPosition);
           // climberMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
          //  climberMotor.setPower(0.5);

            //climbMotorSeries.setTargetPosition(clDownPosition);
          //  climbMotorSeries.setMode(DcMotor.RunMode.RUN_TO_POSITION);
          //  climbMotorSeries.setPower(0.5);
        //}

        // Get the current position of the armMotor
        //double position = climberMotor.getCurrentPosition();
        //  double positions = climbmotorseries.getCurrentPosition();

        // Get the target position of the armMotor
      //  double desiredPosition = climberMotor.getTargetPosition();
      //  double  desiredPosition = climbMotorSeries.getTargetPosition();

        // Show the position of the armMotor on telemetry
    //    telemetry.addData("Encoder Position", position);


        // Show the target position of the armMotor on telemetry
  //      telemetry.addData("Desired Position", desiredPosition);
//
  //  }
//}

