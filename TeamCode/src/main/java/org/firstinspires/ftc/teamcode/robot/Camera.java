package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.Subsystem;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Config
public class Camera extends Subsystem {
    // store the parameters of the camera on dashboard for ease of use
    public static class WebcamParams {
        public int resolutionWidth = 320, resolutionHeight = 240, maxFPS = 90;
    }
    public static WebcamParams webcamParams = new WebcamParams();
    private final AprilTagProcessor aprilTagProcessor; // this is the pipeline responsible for detecting april tags
    private VisionPortal visionPortal; // the camera


    public Camera(Hardware hardware, Telemetry telemetry) {
        super(hardware, telemetry);

        // creating the april tag pipeline
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
                .build();
    }

    @Override
    public void declareHardware() {
        // create camera (visionPortal)
//        visionPortal = new VisionPortal.Builder()
//                .addProcessor(aprilTagProcessor)
//                .setCamera(hardware.getCameraName())
//                .setCameraResolution(new Size(webcamParams.resolutionWidth, webcamParams.resolutionHeight))
//                .build();
    }

    @Override
    public void updateState() {

    }

    public void streamToFTCDashboard() {
        FtcDashboard.getInstance().startCameraStream(visionPortal, webcamParams.maxFPS);
    }

    public VisionPortal getVP() {
        return visionPortal;
    }
    public void enableAprilTagPipeline(boolean enabled) {
        visionPortal.setProcessorEnabled(aprilTagProcessor, enabled);
    }

    // gets the relative position of the closest april tag
    public int getNumDetectedAprilTags() {
        return aprilTagProcessor.getDetections().size();
    }
    public AprilTagDetection getClosestAprilTag() {
        if (aprilTagProcessor.getDetections().isEmpty())
            return null;
        return aprilTagProcessor.getDetections().get(0);
    }
}