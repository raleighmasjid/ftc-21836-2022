package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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

    /**
     * @param hardwareMap     {@link HardwareMap} passed in from the opmode
     * @param myTelemetry     {@link MultipleTelemetry} telemetry to print output to
     * @param tagIdsToLookFor integer IDs of April Tags to look for
     * @param cameraRotation  physical orientation of camera
     */
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

    /**
     * To be run in a "while (!isStarted() && !isStopRequested())" loop at the end of the init sequence
     */
    public void initLoop() {
        if (checkIfTagDetected(pipeline.getLatestDetections())) {
            myTelemetry.addLine("Tag of interest is in sight!");
            printDetectedTag();
        } else printNoTagVisible();

        myTelemetry.update();
    }

    /**
     * To be run once after the opmode is started (after init)
     */
    public void printOutput() {
        camera.stopStreaming();
        camera.closeCameraDevice();

        if (detectedTag != null) printDetectedTag();
        else myTelemetry.addLine("No tag was detected during the init loop :(");

        myTelemetry.update();
    }

    /**
     * @param detections {@link AprilTagDetection} ArrayList grabbed from an {@link AprilTagDetectionPipeline}
     * @return Whether or not a {@link #detectedTag}'s id is one of the {@link #tagIdsToLookFor}
     */
    private boolean checkIfTagDetected(ArrayList<AprilTagDetection> detections) {
        if (detections.size() == 0) return false;
        for (AprilTagDetection detection : detections) {
            for (int tagId : tagIdsToLookFor) {
                if (detection.id == tagId) {
                    detectedTag = detection;
                    return true;
                }
            }
        }
        return false;
    }

    /**
     * Prints telemetry if no tag is visible at the moment <p>
     * However, if a tag was previously detected, it's id will be printed
     */
    private void printNoTagVisible() {
        myTelemetry.addLine("No tag visible");
        if (detectedTag == null) myTelemetry.addLine("(A tag has never been detected)");
        else {
            myTelemetry.addLine("\nBut a tag WAS detected:");
            printDetectedTag();
        }
    }

    /**
     * Prints formatted id of the most recent {@link #detectedTag}
     */
    private void printDetectedTag() {
        myTelemetry.addLine(String.format("\nDetected tag ID=%d", detectedTag.id));
    }
}
