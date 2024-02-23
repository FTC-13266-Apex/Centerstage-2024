package org.firstinspires.ftc.teamcode.opmode.auto.inside.blue;

import static org.firstinspires.ftc.teamcode.opmode.auto.inside.blue.InsideBlueAuto.Constants.PreLoadLeft;
import static org.firstinspires.ftc.teamcode.opmode.auto.inside.blue.InsideBlueAuto.Constants.PreLoadMid;
import static org.firstinspires.ftc.teamcode.opmode.auto.inside.blue.InsideBlueAuto.Constants.PreLoadRight;
import static org.firstinspires.ftc.teamcode.opmode.auto.inside.blue.InsideBlueAuto.Constants.Speed;

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

import java.util.Set;

@Autonomous(name = "Outside Blue Auto")
public class OutsideBlueAuto extends LinearOpMode {
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
            public static Pose2dContainer startPose = new Pose2dContainer(-36, 64, 90);

            public static SetReversed a = new SetReversed(true);
            public static SplineTo b = new SplineTo(-40, 34, 0);
            public static Back c = new Back(15);

            private static final TrajectorySequenceContainer traj = new TrajectorySequenceContainer(a, b, c);
        }

        public static PreLoadRight preLoadRight;
        public static class PreLoadRight {
            public static Pose2dContainer startPose = new Pose2dContainer(-36, 64, 90);

            public static SetReversed a = new SetReversed(true);
            public static Back b = new Back(12);
            public static SplineTo c = new SplineTo(-46, 30, -90);

            private static final TrajectorySequenceContainer traj = new TrajectorySequenceContainer(a, b, c);
        }

        public static PreLoadMid preLoadMid;
        public static class PreLoadMid {
            public static Pose2dContainer startPose = new Pose2dContainer(-36, 64, 90);

            public static SetReversed a = new SetReversed(true);
            public static SplineTo b = new SplineTo(-50, 30, -90);
            public static SplineTo c = new SplineTo(-35, 25, 0);

            private static final TrajectorySequenceContainer traj = new TrajectorySequenceContainer(a, b, c);
        }

    }
    public static Constants.PreLoadLeft yellowLeft;
    public static class YellowLeft {

        public static SetReversed a = new SetReversed(true);
        public static Forward b = new Forward(12);
        public static SplineToConstantHeading c = new SplineToConstantHeading(-30, 12, 0);
        public static SplineTo d = new SplineTo(12, 12, 0);
        public static SplineTo e = new SplineTo(42, 40, 0);

        private static final TrajectorySequenceContainer traj = new TrajectorySequenceContainer(a, b, c, d, e);
    }

    public static Constants.PreLoadRight yellowRight;
    public static class YellowRight {

        public static SetReversed a = new SetReversed(false);
        public static SplineTo b = new SplineTo(-36, 52, 90);
        public static SetReversed c = new SetReversed(true);
        public static SplineTo d = new SplineTo(-24, 11, 0);
        public static Back e = new Back(24);
        public static SplineTo f = new SplineTo(42, 28, 0);

        private static final TrajectorySequenceContainer traj = new TrajectorySequenceContainer(a, b, c, d, e, f);
    }

    public static Constants.PreLoadMid yellowMid;
    public static class YellowMid {

        public static SetReversed a = new SetReversed(true);
        public static Forward b = new Forward(12);
        public static SplineToConstantHeading c = new SplineToConstantHeading(-46, 12, 0);
        public static SplineTo d = new SplineTo(12, 12, 0);
        public static SplineTo e = new SplineTo(42, 36, 0);

        private static final TrajectorySequenceContainer traj = new TrajectorySequenceContainer(a, b, c, d, e);
    }

    public static Constants.PreLoadLeft parkLeft;
    public static class ParkLeft {

        public static SetReversed a = new SetReversed(false);
        public static Forward b = new Forward(8);
        public static SplineToConstantHeading c = new SplineToConstantHeading(52, 12, 0);
        public static Forward d = new Forward(8);

        private static final TrajectorySequenceContainer traj = new TrajectorySequenceContainer(a, b, c, d);
    }

    public static Constants.PreLoadRight parkRight;
    public static class ParkRight {

        public static SetReversed a = new SetReversed(false);
        public static Forward b = new Forward(8);
        public static SplineToConstantHeading c = new SplineToConstantHeading(52, 12, 0);
        public static Forward d = new Forward(8);

        private static final TrajectorySequenceContainer traj = new TrajectorySequenceContainer(a, b, c, d);
    }

    public static Constants.PreLoadMid parkMid;
    public static class ParkMid {

        public static SetReversed a = new SetReversed(false);
        public static Forward b = new Forward(8);
        public static SplineToConstantHeading c = new SplineToConstantHeading(52, 12, 0);
        public static Forward d = new Forward(8);

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
        TrajectorySequence preLoadLeft = Constants.PreLoadLeft.traj.build(Constants.PreLoadLeft.startPose.getPose(), constraints);
        TrajectorySequence yellowLeft = YellowLeft.traj.build(preLoadLeft.end(), constraints);
        TrajectorySequence parkLeft = ParkLeft.traj.build(YellowLeft.traj.end(), constraints);

        TrajectorySequence preLoadMid = Constants.PreLoadMid.traj.build(Constants.PreLoadLeft.startPose.getPose(), constraints);
        TrajectorySequence yellowMid = YellowMid.traj.build(preLoadMid.end(), constraints);
        TrajectorySequence parkMid = ParkMid.traj.build(YellowMid.traj.end(), constraints);

        TrajectorySequence preLoadRight = Constants.PreLoadRight.traj.build(Constants.PreLoadRight.startPose.getPose(), constraints);
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
                yellow = yellowMid;
                park = parkMid;
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
