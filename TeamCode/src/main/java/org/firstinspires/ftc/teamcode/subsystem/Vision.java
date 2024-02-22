package org.firstinspires.ftc.teamcode.subsystem;


import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;


public class Vision {
    public enum Position {
        LEFT,
        MIDDLE,
        RIGHT
    }
    private final Telemetry telemetry;
    private Position finalPos;
    private OpenCvCamera camera;
    private final HardwareMap hardwareMap;
    private MarkerPipeline pipeline;

    public static int CAMERA_WIDTH = 432, CAMERA_HEIGHT = 240;
    public static OpenCvCameraRotation ORIENTATION = OpenCvCameraRotation.UPRIGHT;
    public Vision(HardwareMap hw, Telemetry tl) {
        hardwareMap = hw;
        telemetry = tl;
        init();

        //Camera Grid
//y-
//y+
        //x- <---> x+
        setLeftRectangle(0.009, 0.55);
        setCenterRectangle(.5, 0.55);
        setRightRectangle(0.92, 0.55);
        setRectangleSize(30,30); //Use this to change Box Size
    }

    public void init() {
        //This will instantiate an OpenCvCamera object for the camera we'll be using
        if (true) {
            int cameraMonitorViewId = hardwareMap
                    .appContext.getResources()
                    .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            camera = OpenCvCameraFactory.getInstance()
                    .createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        } else {
            int cameraMonitorViewId = hardwareMap
                    .appContext.getResources()
                    .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            camera = OpenCvCameraFactory.getInstance()
                    .createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        }

        //Set the pipeline the camera should use and start streaming
        camera.setPipeline(pipeline = new MarkerPipeline());
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, ORIENTATION);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
    }


    public void periodic() {
//        currentPos = duckDetector.getPosition();
//        telemetry.addData("Position", duckDetector.getPosition());

        telemetry.addData("Current Position", getPosition());
        telemetry.addData("Final Position", getFinalPosition());

    }

    public void setPosition(Position position) {
        finalPos = position;
    }
    public Position getFinalPosition() {
        return finalPos;
    }

    public void setLeftRectangle(double x, double y) {
        pipeline.setLeftRectHeightPercentage(y);
        pipeline.setLeftRectWidthPercentage(x);
    }
    public void setCenterRectangle(double x, double y) {
        pipeline.setCenterRectHeightPercentage(y);
        pipeline.setCenterRectWidthPercentage(x);
    }
    public void setRightRectangle(double x, double y) {
        pipeline.setRightRectHeightPercentage(y);
        pipeline.setRightRectWidthPercentage(x);
    }
    public void setRectangleSize(int w, int h) {
        pipeline.setRectangleHeight(h);
        pipeline.setRectangleWidth(w);
    }

    public Position getPosition() {
//        Util.logger(this, Level.INFO, "Left Avg: ", getLeftAverage());

        if(pipeline.getLeftAverage() > pipeline.getCenterAverage() && pipeline.getLeftAverage() > pipeline.getRightAverage()){
            return Position.LEFT;
        }
        else if(pipeline.getCenterAverage() > pipeline.getLeftAverage() && pipeline.getCenterAverage() > pipeline.getRightAverage()){
            return Position.MIDDLE;
        }
        else if(pipeline.getRightAverage() > pipeline.getLeftAverage() && pipeline.getRightAverage() > pipeline.getCenterAverage()){
            return Position.RIGHT;
        }
        else{
            return Position.LEFT;
        }
    }
    public void stopFFVision() {
        camera.closeCameraDevice();
    }

}