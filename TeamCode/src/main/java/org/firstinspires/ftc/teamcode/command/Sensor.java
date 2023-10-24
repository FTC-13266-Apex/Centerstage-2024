package org.firstinspires.ftc.teamcode.command;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystem.Gripper;
import org.firstinspires.ftc.teamcode.subsystem.Arm;
@Deprecated
public class Sensor extends Command {
    private enum SensorState {
        DISABLED,
        GRAB,
        LIFT
    }

    public static class Constants {
        public static double LIFT_WAIT_SECONDS = 0.3;
    }

    private final Gripper gripper;
    private final Arm arm;
    private SensorState sensorState = SensorState.GRAB;
    private final ElapsedTime waitTimer = new ElapsedTime();


    public Sensor(@NonNull LinearOpMode opMode, Gripper gripper, Arm arm) {
        super(opMode);
        this.gripper = gripper;
        this.arm = arm;
    }

    @Override
    protected void run() {
        switch (sensorState) {
            case DISABLED:
                if (gripper.ignoreSensor()) break;
                sensorState = SensorState.GRAB;
                break;
            case GRAB:
                if (!gripper.isInRange()) break;
                if (!arm.isDown()) break;
                gripper.close();
                waitTimer.reset();
                sensorState = SensorState.LIFT;
            case LIFT:
                if (!(waitTimer.seconds() > Constants.LIFT_WAIT_SECONDS)) break;
//                arm.moveGroundJunction();
                sensorState = SensorState.DISABLED;
                break;
        }
    }
}
