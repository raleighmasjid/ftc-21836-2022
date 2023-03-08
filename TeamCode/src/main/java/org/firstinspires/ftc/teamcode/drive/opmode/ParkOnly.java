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
import org.firstinspires.ftc.teamcode.TeleOpConfig;
import org.firstinspires.ftc.teamcode.auton.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.drive.AutonMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;


@Autonomous(name= "Parking Only", group = "21836 Backup")
public class ParkOnly extends LinearOpMode {

    OpenCvCamera camera;
    AprilTagDetectionPipeline signalSleeveDetectionPipeline;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = TeleOpConfig.fx;
    double fy = TeleOpConfig.fy;
    double cx = TeleOpConfig.cx;
    double cy = TeleOpConfig.cy;

    // UNITS ARE METERS
    double tagSize = 0.166;

    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    static ElapsedTime autonomousTimer = new ElapsedTime();

    boolean hasParked = false;

    AprilTagDetection tagOfInterest = null;

    PowerplayScorer scorer = new PowerplayScorer();

    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        signalSleeveDetectionPipeline = new AprilTagDetectionPipeline(tagSize, fx, fy, cx, cy);

        camera.setPipeline(signalSleeveDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.SIDEWAYS_RIGHT);
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
        MultipleTelemetry myTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        FtcDashboard dashboard = FtcDashboard.getInstance();

        Vector2d turnPos = new Vector2d(47, -13);

        Vector2d parkingZone1 = new Vector2d(12.5, -12.5);
        Vector2d parkingZone2 = new Vector2d(35, -12.5);
        Vector2d parkingZone3 = new Vector2d(57, -12.5);

        double facingRight = Math.toRadians(0);
        double facingForward = Math.toRadians(90);
        double facingLeft = Math.toRadians(180);

        Pose2d startPose = new Pose2d(35, -62.5, facingForward);
        drive.setPoseEstimate(startPose);

        TrajectorySequence trajectory1 = drive.trajectorySequenceBuilder(startPose)
                .splineTo(parkingZone2, facingForward)
                .build()
                ;

        TrajectorySequence parkLeft = drive.trajectorySequenceBuilder(trajectory1.end())
                .setTangent(facingLeft)
                .splineTo(parkingZone1, facingLeft)
                .build()
                ;

        TrajectorySequence parkRight = drive.trajectorySequenceBuilder(trajectory1.end())
                .setTangent(facingRight)
                .splineTo(turnPos, facingRight)
                .splineTo(parkingZone3, facingRight)
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
        autonomousTimer.reset();

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


        drive.followTrajectorySequenceAsync(trajectory1);

        while(opModeIsActive()) {

            drive.update();


            // parking statement
            if(
                    (autonomousTimer.seconds() >= 3) && //at least 3 seconds into autonomous
                    !drive.isBusy() &&                  //bot is not driving
                    (tagOfInterest != null) &&          //camera HAS detected any tag
                    !hasParked                          //bot has not yet parked in zone
            ) {

                if (tagOfInterest.id == LEFT) {
                    drive.followTrajectorySequenceAsync(parkLeft);
                } else if (tagOfInterest.id == RIGHT) {
                    drive.followTrajectorySequenceAsync(parkRight);
                }

                hasParked = true;
            }



            //everything below is telemetry
            if (scorer.limitSwitch.getState()) {
                myTelemetry.addData("Limit switch", "is not triggered");
            } else {
                myTelemetry.addData("Limit switch", "is triggered");
            }

            if (scorer.clawIsFlipping) {
                myTelemetry.addData("Claw is", "passing through");
            } else if (scorer.clawIsOpen){
                myTelemetry.addData("Claw is", "open");
            } else {
                myTelemetry.addData("Claw is", "closed");
            }

            myTelemetry.addData("Lift position:", scorer.getTargetLiftPositionName());
            myTelemetry.addData("Lift encoder raw output:", scorer.getCurrentLiftPosition());
            myTelemetry.addData("Lift target pos:", scorer.getTargetLiftPosition());
            myTelemetry.addData("Lift motors output", scorer.liftVelocity);

            myTelemetry.addData("Passthrough status", scorer.getCurrentPassthroughState());

            myTelemetry.update();

        }
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
    }
}
