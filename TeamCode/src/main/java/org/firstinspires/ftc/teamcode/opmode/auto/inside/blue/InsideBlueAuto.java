package org.firstinspires.ftc.teamcode.opmode.auto.inside.blue;

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
import org.firstinspires.ftc.teamcode.trajectorysequence.container.SplineToConstantHeading;
import org.firstinspires.ftc.teamcode.trajectorysequence.container.TrajectorySequenceConstraints;
import org.firstinspires.ftc.teamcode.trajectorysequence.container.TrajectorySequenceContainer;
import org.firstinspires.ftc.teamcode.trajectorysequence.container.Turn;

import static org.firstinspires.ftc.teamcode.opmode.auto.inside.blue.InsideBlueAuto.Constants.*;

@Autonomous(name = "Inside Blue Auto")
public class InsideBlueAuto extends LinearOpMode {
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
            public static SplineTo b = new SplineTo(24, 32, -90);
            private static final TrajectorySequenceContainer traj = new TrajectorySequenceContainer(a, b);
        }

        public static PreLoadRight preLoadRight;
        public static class PreLoadRight {
            public static Pose2dContainer startPose = new Pose2dContainer(12, 64, 90);

            public static SetReversed a = new SetReversed(true);
            public static SplineTo b = new SplineTo(0, 40, -135);
            private static final TrajectorySequenceContainer traj = new TrajectorySequenceContainer(a, b);
        }

        public static PreLoadMid preLoadMid;
        public static class PreLoadMid {
            public static Pose2dContainer startPose = new Pose2dContainer(12, 64, 90);

            public static SetReversed a = new SetReversed(true);
            public static Back b = new Back(38);
            private static final TrajectorySequenceContainer traj = new TrajectorySequenceContainer(a, b);
        }

    }
    public static Constants.PreLoadLeft yellowLeft;
    public static class YellowLeft {

        public static SetReversed e = new SetReversed(true);
        public static Forward a = new Forward(12);
        public static Turn b = new Turn(90);
        public static SplineTo c = new SplineTo(50, 43, 0);
        private static final TrajectorySequenceContainer traj = new TrajectorySequenceContainer(e, a, b, c);
    }

    public static Constants.PreLoadRight yellowRight;
    public static class YellowRight {

        public static SetReversed a = new SetReversed(true);
        public static Forward b = new Forward(12);
        public static Turn c = new Turn(90);
        public static SplineTo d = new SplineTo(50, 30, 0);

        private static final TrajectorySequenceContainer traj = new TrajectorySequenceContainer(a, b, c, d);
    }

    public static Constants.PreLoadMid yellowMid;
    public static class YellowMid {

        public static SetReversed a = new SetReversed(true);
        public static Forward b = new Forward(12);
        public static Turn c = new Turn(90);
        public static SplineTo d = new SplineTo(50, 36, 0);

        private static final TrajectorySequenceContainer traj = new TrajectorySequenceContainer(a, b, c, d);
    }

    public static Constants.PreLoadLeft parkLeft;
    public static class ParkLeft {

        public static SetReversed a = new SetReversed(false);
        public static SplineToConstantHeading b = new SplineToConstantHeading(52, 65, 0);
        public static Back c = new Back(10);

        private static final TrajectorySequenceContainer traj = new TrajectorySequenceContainer(a,b, c);
    }

    public static Constants.PreLoadRight parkRight;
    public static class ParkRight {

        public static SetReversed a = new SetReversed(false);
        public static Forward b = new Forward(8);
        public static SplineToConstantHeading c = new SplineToConstantHeading(52, 68, 0);
        public static Back d = new Back(10);

        private static final TrajectorySequenceContainer traj = new TrajectorySequenceContainer(a, b, c, d);
    }

    public static Constants.PreLoadMid parkMid;
    public static class ParkMid {

        public static SetReversed a = new SetReversed(false);
        public static Forward b = new Forward(8);
        public static SplineToConstantHeading c = new SplineToConstantHeading(52, 68, 0);
        public static Back d = new Back(10);

        private static final TrajectorySequenceContainer traj = new TrajectorySequenceContainer(a, b, c, d);
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
        TrajectorySequence parkLeft = ParkLeft.traj.build(YellowLeft.traj.end(), constraints);

        TrajectorySequence preLoadMid = PreLoadMid.traj.build(PreLoadMid.startPose.getPose(), constraints);
        TrajectorySequence yellowMid = YellowMid.traj.build(preLoadMid.end(), constraints);
        TrajectorySequence parkMid = ParkMid.traj.build(YellowMid.traj.end(), constraints);

        TrajectorySequence preLoadRight = PreLoadRight.traj.build(PreLoadRight.startPose.getPose(), constraints);
        TrajectorySequence yellowRight = YellowRight.traj.build(preLoadRight.end(), constraints);
        TrajectorySequence parkRight = ParkRight.traj.build(YellowRight.traj.end(), constraints);

        TrajectorySequence preLoad = preLoadLeft; // This one comes from vision
        TrajectorySequence yellow = yellowLeft;
        TrajectorySequence park = parkLeft;

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
                park = parkLeft;
            case RIGHT:
                preLoad = preLoadRight;
                yellow = yellowRight;
                park = parkRight;
            case MIDDLE:
                preLoad = preLoadMid;
                yellow = yellowLeft;
                park = parkLeft;
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
                    if (drive.isBusy()) break;
                    drive.followTrajectorySequenceAsync(park);
                    state = State.PARK;
                    break;
                case PARK:
                    break;
            }
            drive.update();
        }


    }

}
