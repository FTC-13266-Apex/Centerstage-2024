package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.HashMap;
import java.util.Map;

@Config
public class FieldCentricDrive {
    private final DcMotor leftFront;
    private final DcMotor rightFront;
    private final DcMotor leftRear;
    private final DcMotor rightRear;
    private final Gamepad gamepad1;
    private final IMU imu;
    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    private static final double     COUNTS_PER_MOTOR_REV    = 537.68984;
    private static final double     DRIVE_GEAR_REDUCTION    = 1.0;     // No External Gearing.
    private static final double     WHEEL_DIAMETER_INCHES   = 3.77953;     // For figuring circumference
    private static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);

    public static double STRAFE_WEIGHT = 1.1;
    public static double MIN_HEADING_P = 0.05;
    public static double MAX_HEADING_P = 1.0;
    public static double STICK_THRESHOLD = 0.05;

    public static boolean FIELD_CENTRIC_DRIVING = true;
    public static boolean FIELD_CENTRIC_TURNING = false;

    public FieldCentricDrive(OpMode opMode) {
        this.hardwareMap = opMode.hardwareMap;
        this.gamepad1 = opMode.gamepad1;
        this.telemetry = opMode.telemetry;

        // Declare our motors
        // Make sure your ID's match your configuration
        leftFront = (DcMotor) hardwareMap.get("leftFront");
        rightFront = (DcMotor) hardwareMap.get("rightFront");
        leftRear = (DcMotor) hardwareMap.get("leftRear");
        rightRear = (DcMotor) hardwareMap.get("rightRear");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.FORWARD);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

    }

    public void teleOp() {
        if (gamepad1.a) {
            imu.resetYaw();
        }

        double leftY = -gamepad1.left_stick_y; // Remember, this is reversed!
        double leftX = gamepad1.left_stick_x; // Counteract imperfect strafing
        double rightY = -gamepad1.right_stick_y;
        double rightX = -gamepad1.right_stick_x;

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double x;
        double y;
        if (FIELD_CENTRIC_DRIVING) {
            // Rotate the movement direction counter to the bot's rotation
            Vector2d rotated2d = new Vector2d(leftX, leftY).rotated(botHeading);
            x = rotated2d.getX();
            y = rotated2d.getY();

            x *= STRAFE_WEIGHT;  // Counteract imperfect strafing
        } else {
            x = leftX;
            y = rightY;
        }

        double turn;
        if (FIELD_CENTRIC_TURNING) {
            Vector2d turn2d = new Vector2d(rightX, rightY);
            double stickHeading = turn2d
                    .rotated(Math.PI / 2) // Rotate by Pi / 2 Radians (90 degrees)
                    .angle() - Math.PI; // Subtract Pi radians (180 degrees) from final angle

            // Find the lowest of the 3 coterminal angles
            double defaultHeadingError = (botHeading - stickHeading); // Default angle
            double lowHeadingError = defaultHeadingError - (Math.PI * 2); // Low coterminal angle (360 degrees off from original)
            double highHeadingError = defaultHeadingError + (Math.PI * 2); // High coterminal angle (360 degrees off from original)

            // Create map of the absolute value of each value to the original value
            double headingError = Arrays.stream(new Double[]{
                            defaultHeadingError,
                            lowHeadingError,
                            highHeadingError
                    })
                    .min(Comparator.comparingDouble(Math::abs)).get();

            double distance = turn2d.distTo(new Vector2d(0,0));


            double headingP = MIN_HEADING_P + ((MAX_HEADING_P - MIN_HEADING_P) * distance);
            turn = headingError * -headingP;

            if (distance < STICK_THRESHOLD) turn = 0;

            telemetry.addData("heading error", Math.toDegrees(botHeading - stickHeading));
            telemetry.addData("theta", Math.toDegrees(stickHeading));

            telemetry.addData("low heading error", lowHeadingError);
            telemetry.addData("default heading error", defaultHeadingError);
            telemetry.addData("high heading error", highHeadingError);
        } else {
            turn = rightX;

            if (rightX < STICK_THRESHOLD) turn = 0;
        }

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(turn), 1);
        double frontLeftPower = (y + x + turn) / denominator;
        double backLeftPower = (y - x + turn) / denominator;
        double frontRightPower = (y - x - turn) / denominator;
        double backRightPower = (y + x - turn) / denominator;


        leftFront.setPower(frontLeftPower);
        leftRear.setPower(backLeftPower);
        rightFront.setPower(frontRightPower);
        rightRear.setPower(backRightPower);

        telemetry.addData("heading", Math.toDegrees(botHeading));

    }

    public void setMotorPowers(double leftFront, double leftRear, double rightRear, double rightFront) {
        this.leftFront.setPower(leftFront);
        this.leftRear.setPower(leftRear);
        this.rightRear.setPower(rightRear);
        this.rightFront.setPower(rightFront);
    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches) {
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftRearTarget;
        int newRightRearTarget;

        // Ensure that the OpMode is still active

        // Determine new target position, and pass to motor controller
        newLeftFrontTarget = leftFront.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
        newRightFrontTarget = rightFront.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
        newLeftRearTarget = leftRear.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
        newRightRearTarget = rightRear.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
        leftFront.setTargetPosition(newLeftFrontTarget);
        rightFront.setTargetPosition(newRightFrontTarget);
        leftRear.setTargetPosition(newLeftRearTarget);
        rightRear.setTargetPosition(newRightRearTarget);

        // Turn On RUN_TO_POSITION
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(Math.abs(speed));
        rightFront.setPower(Math.abs(speed));
        leftRear.setPower(Math.abs(speed));
        rightRear.setPower(Math.abs(speed));

        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.
        while (leftFront.isBusy() && rightFront.isBusy() && leftRear.isBusy() && rightRear.isBusy()) {

            // Display it for the driver.
            telemetry.addData("Running to", " %7d :%7d", newLeftFrontTarget, newRightFrontTarget);
            telemetry.addData("Currently at", " at %7d :%7d",
                    leftFront.getCurrentPosition(), rightFront.getCurrentPosition());
            telemetry.update();
        }

        // Stop all motion;
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);

        // Turn off RUN_TO_POSITION
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ElapsedTime sleeptimer = new ElapsedTime();
        while (sleeptimer.seconds() < 0.25){
            telemetry.addLine("waiting");
        }
    }
}

