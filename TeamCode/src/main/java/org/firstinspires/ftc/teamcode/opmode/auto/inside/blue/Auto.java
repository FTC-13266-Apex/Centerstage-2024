package org.firstinspires.ftc.teamcode.opmode.auto.inside.blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystem.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.Slides;
import org.firstinspires.ftc.teamcode.subsystem.Vision;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.container.Forward;
import org.firstinspires.ftc.teamcode.trajectorysequence.container.Pose2dContainer;
import org.firstinspires.ftc.teamcode.trajectorysequence.container.SplineTo;
import org.firstinspires.ftc.teamcode.trajectorysequence.container.TrajectorySequenceConstraints;
import org.firstinspires.ftc.teamcode.trajectorysequence.container.TrajectorySequenceContainer;
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
            public static SplineTo a = new SplineTo(22.5, 32, 90);
            public static Forward b = new Forward(12);
            private static final TrajectorySequenceContainer traj = new TrajectorySequenceContainer(a, b);
        }
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
        TrajectorySequence preLoadMid = preLoadLeft; // TODO: There should be two more traj sequences here
        TrajectorySequence preLoadRight = preLoadLeft;
        TrajectorySequence preLoad = preLoadLeft; // This one comes from vision
        State state = State.PRE_LOAD_A;


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
            case RIGHT:
                preLoad = preLoadMid;
            case MIDDLE:
                preLoad = preLoadRight;
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
                    if (timer.seconds() < 1.0) break;
                    // TODO: you need to implement something like this to allow slides to move in auto
                    // Currently, there are no public methods that would let you move the slides
//                    slides.lift();
                    break;
                case DROP:
                    break;
                case PARK:
                    break;
            }
            drive.update();
        }


    }

}
