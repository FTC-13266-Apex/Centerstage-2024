package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Drive;


@Autonomous
public class Main   {

    @Override
    public void runOpMode () {
        Drive drive = new Drive(this);
        public void onStart () {
            double power = .3;
            forward(power);

            forward(0);
        }
        @Override
        public void run() {

        }
        private void forward (double power) {
            drive.setMotorPowers(power,power,power,power);
        }
    }
}

