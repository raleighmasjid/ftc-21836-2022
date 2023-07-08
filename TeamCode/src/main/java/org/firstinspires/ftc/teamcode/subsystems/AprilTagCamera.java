package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.control.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Config
public class AprilTagCamera {

    public static double
            TAG_SIZE = 0.166,
            CAMERA_FX = 578.272,
            CAMERA_FY = 578.272,
            CAMERA_CX = 402.145,
            CAMERA_CY = 221.506;

    public AprilTagDetection detectedTag = null;

    private final OpenCvCamera camera;

    private final AprilTagDetectionPipeline pipeline = new AprilTagDetectionPipeline(
            TAG_SIZE,
            CAMERA_FX,
            CAMERA_FY,
            CAMERA_CX,
            CAMERA_CY
    );

    private final MultipleTelemetry myTelemetry;

    private final int[] tagIdsToLookFor;

    public AprilTagCamera(HardwareMap hardwareMap, MultipleTelemetry myTelemetry, int[] tagIdsToLookFor, OpenCvCameraRotation cameraRotation) {
        camera = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"),
                hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName())
        );
        camera.setPipeline(pipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, cameraRotation);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
        this.myTelemetry = myTelemetry;
        this.tagIdsToLookFor = tagIdsToLookFor;
    }

    public void initLoop() {
        ArrayList<AprilTagDetection> currentDetections = pipeline.getLatestDetections();

        if (currentDetections.size() != 0) {
            boolean tagFound = false;

            iterateTags:
            for (AprilTagDetection detection : currentDetections) {
                for (int tagId : tagIdsToLookFor) {
                    if (detection.id == tagId) {
                        detectedTag = detection;
                        tagFound = true;
                        break iterateTags;
                    }
                }
            }
            if (tagFound) {
                myTelemetry.addLine("Tag of interest is in sight!");
                tagToTelemetry(detectedTag);
            } else printNoTagFound();
        } else printNoTagFound();

        myTelemetry.update();
    }

    public void printOutput() {
        camera.stopStreaming();
        camera.closeCameraDevice();

        if (detectedTag != null) {
            myTelemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(detectedTag);
            myTelemetry.update();
        } else {
            myTelemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            myTelemetry.update();
        }
    }

    private void printNoTagFound() {
        myTelemetry.addLine("Don't see tag of interest :(");
        if (detectedTag == null) myTelemetry.addLine("(The tag has never been seen)");
        else {
            myTelemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
            tagToTelemetry(detectedTag);
        }
    }

    private void tagToTelemetry(AprilTagDetection detection) {
        myTelemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
    }
}
