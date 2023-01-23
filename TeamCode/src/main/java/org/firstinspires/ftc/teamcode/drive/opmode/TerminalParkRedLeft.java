package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.PowerplayScorer;
import org.firstinspires.ftc.teamcode.auton.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.drive.AutonMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;


@Autonomous(name= "Red Alliance - Left - Terminal & park", group = "21836 Autonomous")
public class TerminalParkRedLeft extends LinearOpMode {

    OpenCvCamera camera;
    AprilTagDetectionPipeline signalSleeveDetectionPipeline;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    static ElapsedTime autonTimer = new ElapsedTime();

    boolean hasParked = false;

    AprilTagDetection tagOfInterest = null;

    PowerplayScorer scorer = new PowerplayScorer();

    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        signalSleeveDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(signalSleeveDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        AutonMecanumDrive drive = new AutonMecanumDrive(hardwareMap);
        scorer.init(hardwareMap);

        //  Initialize telemetry and dashboard
        MultipleTelemetry mytelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        FtcDashboard dashboard = FtcDashboard.getInstance();

        //  initializes code:


        Pose2d startPose = new Pose2d(-35, -62.5, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(startPose)
                .splineToConstantHeading(new Vector2d(-40, -61), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-55, -61), Math.toRadians(180))
                .waitSeconds(0.05)
                .setTangent(0)
                .lineTo(new Vector2d(-35, -61))
                .waitSeconds(0.05)
                .lineTo(new Vector2d(-35, -12.5))
                .waitSeconds(0.05)
                .build()
                ;

        TrajectorySequence parkLeft = drive.trajectorySequenceBuilder(traj1.end())
                .lineToConstantHeading(new Vector2d(-57, -12.5))
                .build()
                ;

        TrajectorySequence parkRight = drive.trajectorySequenceBuilder(traj1.end())
                .lineToConstantHeading(new Vector2d(-12.5, -12.5))
                .build()
                ;

        TrajectorySequence parkMiddle = drive.trajectorySequenceBuilder(traj1.end())
                .lineToConstantHeading(new Vector2d(-34.5, -12.5))
                .build()
                ;



        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */

        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = signalSleeveDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);

                }

            }

            telemetry.update();
            sleep(20);
        }

        //START IS HERE//

        camera.stopStreaming();
        camera.closeCameraDevice();

        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }


        drive.followTrajectorySequenceAsync(traj1);
        autonTimer.reset();

        while(opModeIsActive()) {

            drive.update();


            if((autonTimer.seconds() >= 3) && !drive.isBusy() && !hasParked) {
                if(tagOfInterest.id == LEFT) {
                    drive.followTrajectorySequenceAsync(parkLeft);

                } else if(tagOfInterest.id == RIGHT) {
                    drive.followTrajectorySequenceAsync(parkRight);

                } else {
                    drive.followTrajectorySequenceAsync(parkMiddle);
                }

                hasParked = true;
            }

        }
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
    }
}
