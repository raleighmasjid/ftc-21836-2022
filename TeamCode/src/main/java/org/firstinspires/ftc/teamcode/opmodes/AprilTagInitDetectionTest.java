/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.opmodes;


import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.autonomous.AutonConfig;
import org.firstinspires.ftc.teamcode.control.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name = "Camera Test", group = "21836 Backup")
public class AprilTagInitDetectionTest extends LinearOpMode {

    MultipleTelemetry myTelemetry;

    @Override
    public void runOpMode() throws InterruptedException {

        // Lens intrinsics
        // UNITS ARE PIXELS
        // NOTE: this calibration is for the C920 webcam at 800x448.
        // You will need to do your own calibration for other configurations!
        int ONE = 1;
        int TWO = 2;
        int THREE = 3;

        AprilTagDetection tagOfInterest = null;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        AprilTagDetectionPipeline pipeline = new AprilTagDetectionPipeline(
                AutonConfig.TAG_SIZE,
                AutonConfig.CAMERA_FX,
                AutonConfig.CAMERA_FY,
                AutonConfig.CAMERA_CX,
                AutonConfig.CAMERA_CY
        );

        camera.setPipeline(pipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        //  Initialize telemetry and dashboard
        myTelemetry = new MultipleTelemetry(telemetry);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */

        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = pipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == ONE || tag.id == TWO || tag.id == THREE) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if (tagFound) {
                    myTelemetry.addLine("Tag of interest is in sight!");
                    tagToTelemetry(tagOfInterest);
                } else {
                    myTelemetry.addLine("Don't see tag of interest :(");

                    if (tagOfInterest == null) myTelemetry.addLine("(The tag has never been seen)");
                    else {
                        myTelemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            } else {
                myTelemetry.addLine("Don't see tag of interest :(");

                if (tagOfInterest == null) myTelemetry.addLine("(The tag has never been seen)");
                else {
                    myTelemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);

                }

            }

            myTelemetry.update();
        }

        //START IS HERE//

        camera.stopStreaming();
        camera.closeCameraDevice();

        if (tagOfInterest != null) {
            myTelemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            myTelemetry.update();
        } else {
            myTelemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            myTelemetry.update();
        }
    }

    void tagToTelemetry(AprilTagDetection detection) {
        myTelemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
    }
}