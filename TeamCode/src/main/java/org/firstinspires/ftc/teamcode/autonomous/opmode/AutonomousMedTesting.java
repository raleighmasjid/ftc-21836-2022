package org.firstinspires.ftc.teamcode.autonomous.opmode;


import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.autonomous.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.control.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.robot.AutonMecanumDrive;
import org.firstinspires.ftc.teamcode.robot.PowerplayScorer;
import org.firstinspires.ftc.teamcode.robot.RobotConfig;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.List;


@Autonomous(name= "Medium Testing", group = "21836 Backup")
public class AutonomousMedTesting extends LinearOpMode {

    OpenCvCamera camera;
    AprilTagDetectionPipeline signalSleeveDetectionPipeline;
    List<LynxModule> hubs;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double
            fx = RobotConfig.camera_fx,
            fy = RobotConfig.camera_fy,
            cx = RobotConfig.camera_cx,
            cy = RobotConfig.camera_cy,
            tagSize = 0.166;
    int
            LEFT = 1,
            MIDDLE = 2,
            RIGHT = 3;

    ElapsedTime autonomousTimer = new ElapsedTime();

    boolean hasParked = false;

    AprilTagDetection tagOfInterest = null;

    PowerplayScorer scorer = new PowerplayScorer();
    AutonMecanumDrive drivetrain;

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

        drivetrain = new AutonMecanumDrive(hardwareMap);
        scorer.init(hardwareMap);

        //  Initialize telemetry and dashboard
        MultipleTelemetry myTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        FtcDashboard dashboard = FtcDashboard.getInstance();
        dashboard.startCameraStream(camera,0);

        boolean isRight = true;
        double side = isRight? 1: -1;

        double centerPathX = side*35;

        double facingRight = Math.toRadians(0);
        double facingForward = Math.toRadians(90);
        double facingLeft = Math.toRadians(180);
        double liftAndDropTime = 0.1;
        double clawToFlipTime = 0.1;

        Vector2d stackPos = new Vector2d(side*59, -12.5);
        Vector2d sideTurnPos = new Vector2d(side*46, -12.5);
        Pose2d medScoringPos = new Pose2d(side*31, -17.5, Math.toRadians(isRight? 35: 180-35));
        Pose2d centerTallScoringPos = new Pose2d(medScoringPos.getX()-side*24, medScoringPos.getY(), medScoringPos.getHeading());
        Vector2d centerTurnPos = new Vector2d(sideTurnPos.getX()-side*24, sideTurnPos.getY());

        Pose2d parkingZone1 = new Pose2d(side*(isRight? 12.5:57), -12.5, isRight? facingRight: facingLeft);
        Pose2d parkingZone2 = new Pose2d(centerPathX, -12.5, parkingZone1.getHeading());
        Pose2d parkingZone3 = new Pose2d(side*(isRight? 57:12.5), -12.5, parkingZone1.getHeading());

        Pose2d startPose = new Pose2d(centerPathX, -62.5, facingForward);
        drivetrain.setPoseEstimate(startPose);

        TrajectorySequence trajectory1 = drivetrain.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .addTemporalMarker(() -> scorer.raiseClaw())
                .UNSTABLE_addTemporalMarkerOffset(clawToFlipTime, () -> scorer.triggerPassThru())
                .lineTo(parkingZone2.vec())
                .lineToSplineHeading(medScoringPos)
                .UNSTABLE_addTemporalMarkerOffset(-RobotConfig.TIME_LIFT_MEDIUM, () -> scorer.setTargetLiftPos(PowerplayScorer.liftPos.MED))
                .addTemporalMarker(() -> scorer.dropClaw(PowerplayScorer.liftPos.FIVE))
                .waitSeconds(liftAndDropTime)
                .UNSTABLE_addTemporalMarkerOffset(clawToFlipTime, () -> scorer.triggerPassThru())
                .setReversed(false)
                .splineTo(sideTurnPos, isRight? facingRight: facingLeft)
                .splineTo(stackPos, isRight? facingRight: facingLeft)
                .addTemporalMarker(() -> scorer.liftClaw())
                .waitSeconds(liftAndDropTime)
                .UNSTABLE_addTemporalMarkerOffset(clawToFlipTime, () -> scorer.triggerPassThru())
                // loop below
                .setReversed(true)
                .splineTo(sideTurnPos, isRight? facingLeft: facingRight)
                .splineToSplineHeading(medScoringPos, medScoringPos.getHeading()-facingLeft)
                .UNSTABLE_addTemporalMarkerOffset(-RobotConfig.TIME_LIFT_MEDIUM, () -> scorer.setTargetLiftPos(PowerplayScorer.liftPos.MED))
                .addTemporalMarker(() -> scorer.dropClaw(PowerplayScorer.liftPos.FOUR))
                .waitSeconds(liftAndDropTime)
                .UNSTABLE_addTemporalMarkerOffset(clawToFlipTime, () -> scorer.triggerPassThru())
                .setReversed(false)
                .splineTo(sideTurnPos, isRight? facingRight: facingLeft)
                .splineTo(stackPos, isRight? facingRight: facingLeft)
                .addTemporalMarker(() -> scorer.liftClaw())
                .waitSeconds(liftAndDropTime)
                .UNSTABLE_addTemporalMarkerOffset(clawToFlipTime, () -> scorer.triggerPassThru())
                .build()
                ;

        TrajectorySequence parkLeft = isRight?
                drivetrain.trajectorySequenceBuilder(trajectory1.end())
                        .setReversed(true)
                        .splineTo(centerTurnPos, isRight? facingLeft: facingRight)
                        .splineToSplineHeading(centerTallScoringPos, medScoringPos.getHeading()-facingLeft)
                        .UNSTABLE_addTemporalMarkerOffset(-RobotConfig.TIME_LIFT_MEDIUM, () -> scorer.setTargetLiftPos(PowerplayScorer.liftPos.MED))
                        .addTemporalMarker(() -> scorer.dropClaw())
                        .waitSeconds(liftAndDropTime)
                        .UNSTABLE_addTemporalMarkerOffset(clawToFlipTime, () -> scorer.triggerPassThru())
                        .lineToSplineHeading(parkingZone1)
                        .build():
                drivetrain.trajectorySequenceBuilder(trajectory1.end())
                        .setReversed(true)
                        .splineTo(sideTurnPos, isRight? facingLeft: facingRight)
                        .splineToSplineHeading(medScoringPos, medScoringPos.getHeading()-facingLeft)
                        .UNSTABLE_addTemporalMarkerOffset(-RobotConfig.TIME_LIFT_MEDIUM, () -> scorer.setTargetLiftPos(PowerplayScorer.liftPos.MED))
                        .addTemporalMarker(() -> scorer.dropClaw())
                        .waitSeconds(liftAndDropTime)
                        .UNSTABLE_addTemporalMarkerOffset(clawToFlipTime, () -> scorer.triggerPassThru())
                        .setReversed(false)
                        .splineTo(sideTurnPos, isRight? facingRight: facingLeft)
                        .splineTo(parkingZone1.vec(), parkingZone1.getHeading())
                        .build()
                ;

        TrajectorySequence parkMiddle = drivetrain.trajectorySequenceBuilder(trajectory1.end())
                .setReversed(true)
                .splineTo(sideTurnPos, isRight? facingLeft: facingRight)
                .splineToSplineHeading(medScoringPos, medScoringPos.getHeading()-facingLeft)
                .UNSTABLE_addTemporalMarkerOffset(-RobotConfig.TIME_LIFT_MEDIUM, () -> scorer.setTargetLiftPos(PowerplayScorer.liftPos.MED))
                .addTemporalMarker(() -> scorer.dropClaw())
                .waitSeconds(liftAndDropTime)
                .UNSTABLE_addTemporalMarkerOffset(clawToFlipTime, () -> scorer.triggerPassThru())
                .lineToSplineHeading(parkingZone2)
                .build()
                ;

        TrajectorySequence parkRight = isRight?
                drivetrain.trajectorySequenceBuilder(trajectory1.end())
                        .setReversed(true)
                        .splineTo(sideTurnPos, isRight? facingLeft: facingRight)
                        .splineToSplineHeading(medScoringPos, medScoringPos.getHeading()-facingLeft)
                        .UNSTABLE_addTemporalMarkerOffset(-RobotConfig.TIME_LIFT_MEDIUM, () -> scorer.setTargetLiftPos(PowerplayScorer.liftPos.MED))
                        .addTemporalMarker(() -> scorer.dropClaw())
                        .waitSeconds(liftAndDropTime)
                        .UNSTABLE_addTemporalMarkerOffset(clawToFlipTime, () -> scorer.triggerPassThru())
                        .setReversed(false)
                        .splineTo(sideTurnPos, isRight? facingRight: facingLeft)
                        .splineTo(parkingZone3.vec(), parkingZone3.getHeading())
                        .build():
                drivetrain.trajectorySequenceBuilder(trajectory1.end())
                        .setReversed(true)
                        .splineTo(centerTurnPos, isRight? facingLeft: facingRight)
                        .splineToSplineHeading(centerTallScoringPos, medScoringPos.getHeading()-facingLeft)
                        .UNSTABLE_addTemporalMarkerOffset(-RobotConfig.TIME_LIFT_MEDIUM, () -> scorer.setTargetLiftPos(PowerplayScorer.liftPos.MED))
                        .addTemporalMarker(() -> scorer.dropClaw())
                        .waitSeconds(liftAndDropTime)
                        .UNSTABLE_addTemporalMarkerOffset(clawToFlipTime, () -> scorer.triggerPassThru())
                        .lineToSplineHeading(parkingZone3)
                        .build()
                ;

        drivetrain.followTrajectorySequenceAsync(trajectory1);

        hubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */

        scorer.setClawOpen(false);
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
//            sleep(20);
            scorer.runClaw();
        }

        //START IS HERE//

        dashboard.stopCameraStream();
        camera.stopStreaming();
        camera.closeCameraDevice();

        autonomousTimer.reset();

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


        while(opModeIsActive()) {

            for (LynxModule hub : hubs) {
                hub.clearBulkCache();
            }

            if(!hasParked && !drivetrain.isBusy() && (autonomousTimer.seconds() >= 3)) {

                if (tagOfInterest != null) {
                    if (tagOfInterest.id == LEFT) {
                        drivetrain.followTrajectorySequenceAsync(parkLeft);
                    } else if (tagOfInterest.id == RIGHT) {
                        drivetrain.followTrajectorySequenceAsync(parkRight);
                    } else {
                        drivetrain.followTrajectorySequenceAsync(parkMiddle);
                    }
                } else {
                    drivetrain.followTrajectorySequenceAsync(parkMiddle);
                }

                hasParked = true;
            }

            scorer.readLiftPos();
            scorer.runLiftToPos();
            scorer.runPassThruStates();
            scorer.runClaw();
            scorer.runPivot();
            scorer.runPassThruServos();
            drivetrain.update();

            //everything below is telemetry
            scorer.printTelemetry(myTelemetry);
            myTelemetry.update();

        }
    }

    @SuppressLint("DefaultLocale")
    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
    }
}
