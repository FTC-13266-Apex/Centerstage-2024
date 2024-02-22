package org.firstinspires.ftc.teamcode.opmode.auto.inside.blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.trajectorysequence.container.Forward;
import org.firstinspires.ftc.teamcode.trajectorysequence.container.Pose2dContainer;
import org.firstinspires.ftc.teamcode.trajectorysequence.container.SplineTo;

@Autonomous(name = "Inside Blue Auto")
public class Auto {
    public static class Constants {
        public static Speed speed;
        public static class Speed {
            public static double baseVel = 25; // value
            public static double baseAccel = 25; // value
            public static double turnVel = Math.toRadians(90); // value
            public static double turnAccel = Math.toRadians(90); // value
            public static double slowVel = baseVel * 0.8; // value
            public static double slowAccel = baseAccel * 0.8; // value
            public static double slowTurnVel = turnVel * 0.8; // value
            public static double slowTurnAccel = turnAccel * 0.8; // value
        }

        public static WaitSeconds waitSeconds;
        public static class WaitSeconds {
            public static double dropDriveWait = 0.5;
            public static double dropLiftWait = 1.3 ;
            public static double pickupLiftWait = 1.0;
            public static double coneFlipperYeetWait = 2.0;
            public static double coneFlipperLiftWait = 0.1;
        }

        public static Pose2dContainer startPose = new Pose2dContainer(12, 64, 90);
        public static SplineTo a = new SplineTo(22.5, 32, 90);
        public static Forward b = new Forward(12);
        public static Tur

}
