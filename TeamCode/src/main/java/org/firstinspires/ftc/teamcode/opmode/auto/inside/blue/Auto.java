package org.firstinspires.ftc.teamcode.opmode.auto.inside.blue;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystem.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.Slides;
import org.firstinspires.ftc.teamcode.subsystem.Vision;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.container.Back;
import org.firstinspires.ftc.teamcode.trajectorysequence.container.Forward;
import org.firstinspires.ftc.teamcode.trajectorysequence.container.Pose2dContainer;
import org.firstinspires.ftc.teamcode.trajectorysequence.container.SetReversed;
import org.firstinspires.ftc.teamcode.trajectorysequence.container.SplineTo;
import org.firstinspires.ftc.teamcode.trajectorysequence.container.StrafeRight;
import org.firstinspires.ftc.teamcode.trajectorysequence.container.TrajectorySequenceConstraints;
import org.firstinspires.ftc.teamcode.trajectorysequence.container.TrajectorySequenceContainer;
import org.firstinspires.ftc.teamcode.trajectorysequence.container.Turn;

import static org.firstinspires.ftc.teamcode.opmode.auto.inside.blue.Auto.Constants.*;

@Autonomous(name = "Inside Blue Auto")
public class Auto extends LinearOpMode {
    public static Constants constants;
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

        public static PreLoadLeft preLoadLeft;
        public static class PreLoadLeft {
            public static Pose2dContainer startPose = new Pose2dContainer(12, 64, 90);

            public static SetReversed a = new SetReversed(true);
            public static SplineTo b = new SplineTo(22.5, 32, -90);
            private static final TrajectorySequenceContainer traj = new TrajectorySequenceContainer(a, b);
        }
    }
    public static Constants.PreLoadLeft yellowLeft;
    public static class YellowLeft {

        public static SetReversed e = new SetReversed(true);
        public static Forward a = new Forward(12);
        public static Turn b = new Turn(90);
        public static SplineTo c = new SplineTo(42, 48, 0);
       // public static Back d = new Back(8);

        private static final TrajectorySequenceContainer traj = new TrajectorySequenceContainer(e, a, b, c);
    }

    private enum State {
        PRE_LOAD_A,
        PRE_LOAD_B,
        DROP,
        PARK,
    }

        @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Vision vision = new Vision(hardwareMap, telemetry);
        Slides slides = new Slides(this);

        TrajectorySequenceConstraints constraints = new TrajectorySequenceConstraints(
                Speed.baseVel,
                Speed.baseAccel,
                Speed.turnVel,
                Speed.turnAccel
        );

        // We should build all of the trajectories in advance so that this doesn't happen during the middle of
        // Pathing
        ElapsedTime timer = new ElapsedTime();
        TrajectorySequence preLoadLeft = PreLoadLeft.traj.build(PreLoadLeft.startPose.getPose(), constraints);
        TrajectorySequence yellowLeft = YellowLeft.traj.build(preLoadLeft.end(), constraints);

        TrajectorySequence preLoadMid = preLoadLeft; // TODO: There should be two more traj sequences here

        TrajectorySequence preLoadRight = preLoadLeft;

        TrajectorySequence preLoad = preLoadLeft; // This one comes from vision
        TrajectorySequence yellow = yellowLeft;

        State state = State.PRE_LOAD_A;

        drive.setPoseEstimate(PreLoadLeft.startPose.getPose());


        while (!isStarted()) {
            vision.setPosition(vision.getPosition());
            telemetry.addData("Vision", vision.getFinalPosition());
            telemetry.update();
        }
        waitForStart();



        Vision.Position position = vision.getFinalPosition();

        switch (position) {
            case LEFT:
                preLoad = preLoadLeft;
                yellow = yellowLeft;
            case RIGHT:
                preLoad = preLoadMid;
                yellow = yellowLeft;
            case MIDDLE:
                preLoad = preLoadRight;
                yellow = yellowLeft;
        }

        while (!isStopRequested()) {
            switch (state) {
                case PRE_LOAD_A:
                    if (drive.isBusy()) break;
                    drive.followTrajectorySequenceAsync(preLoad);
                    state = State.PRE_LOAD_B;
                    timer.reset();
                    break;
                case PRE_LOAD_B:
                    if (drive.isBusy()) break;
                    drive.followTrajectorySequenceAsync(yellow);
                    state = State.DROP;
                    // TODO: you need to implement something like this to allow slides to move in auto
                    // Currently, there are no public methods that would let you move the slides
//                    slides.lift();
                    break;
                case DROP:
                    state = State.PARK;
                    break;
                case PARK:
                    break;
            }
            drive.update();
        }


    }

}
