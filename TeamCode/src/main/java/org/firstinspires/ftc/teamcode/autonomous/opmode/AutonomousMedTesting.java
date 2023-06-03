package org.firstinspires.ftc.teamcode.autonomous.opmode;


import android.annotation.SuppressLint;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.autonomous.AutonConfig;
import org.firstinspires.ftc.teamcode.autonomous.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.autonomous.utility.AutonMecanumDrive;
import org.firstinspires.ftc.teamcode.control.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.control.HeadingHolder;
import org.firstinspires.ftc.teamcode.robot.PowerplayScorer;
import org.firstinspires.ftc.teamcode.robot.RobotConfig;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.List;


@Autonomous(name = "Medium Testing", group = "21836 Backup")
public class AutonomousMedTesting extends LinearOpMode {

    MultipleTelemetry myTelemetry;
    AutonMecanumDrive drivetrain;
    PowerplayScorer scorer;
    List<LynxModule> hubs;

    @Override
    public void runOpMode() throws InterruptedException {
        // Lens intrinsics
        // UNITS ARE PIXELS
        // NOTE: this calibration is for the C920 webcam at 800x448.
        // You will need to do your own calibration for other configurations!
        double
                fx = AutonConfig.CAMERA_FX,
                fy = AutonConfig.CAMERA_FY,
                cx = AutonConfig.CAMERA_CX,
                cy = AutonConfig.CAMERA_CY,
                tagSize = 0.166;
        int
                LEFT = 1,
                MIDDLE = 2,
                RIGHT = 3;

        ElapsedTime autonomousTimer = new ElapsedTime();

        boolean hasParked = false;

        AprilTagDetection tagOfInterest = null;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        AprilTagDetectionPipeline signalSleeveDetectionPipeline = new AprilTagDetectionPipeline(tagSize, fx, fy, cx, cy);

        camera.setPipeline(signalSleeveDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.SIDEWAYS_RIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        drivetrain = new AutonMecanumDrive(hardwareMap);
        scorer = new PowerplayScorer(hardwareMap);

        //  Initialize telemetry and dashboard
        myTelemetry = new MultipleTelemetry(telemetry);

        boolean isRight = true;
        double side = isRight ? 1 : -1;

        double centerPathX = side * AutonConfig.ZONE_2_X;

        double facingRight = Math.toRadians(0);
        double facingForward = Math.toRadians(90);
        double facingLeft = Math.toRadians(180);

        Vector2d stackPos = new Vector2d(side * AutonConfig.STACK_X, AutonConfig.MAIN_Y);
        Vector2d sideTurnPos = new Vector2d(side * AutonConfig.TURN_POS_X, AutonConfig.MAIN_Y);
        Pose2d medScoringPos = new Pose2d(side * AutonConfig.MED_X, AutonConfig.MED_Y, Math.toRadians(isRight ? AutonConfig.MED_ANGLE : 180 - AutonConfig.MED_ANGLE));
        Pose2d centerTallScoringPos = new Pose2d(medScoringPos.getX() - side * AutonConfig.ONE_TILE, medScoringPos.getY(), medScoringPos.getHeading());
        Vector2d centerTurnPos = new Vector2d(sideTurnPos.getX() - side * AutonConfig.ONE_TILE, sideTurnPos.getY());

        Pose2d parkingZone1 = new Pose2d(side * (isRight ? AutonConfig.ZONE_1_X : AutonConfig.ZONE_3_X), AutonConfig.MAIN_Y, isRight ? facingRight : facingLeft);
        Pose2d parkingZone2 = new Pose2d(centerPathX, AutonConfig.MAIN_Y, parkingZone1.getHeading());
        Pose2d parkingZone3 = new Pose2d(side * (isRight ? AutonConfig.ZONE_3_X : AutonConfig.ZONE_1_X), AutonConfig.MAIN_Y, parkingZone1.getHeading());

        HeadingHolder.setHeading(isRight ? 270.0 : 90.0);

        Pose2d startPose = new Pose2d(centerPathX, AutonConfig.STARTING_Y, facingForward);
        drivetrain.setPoseEstimate(startPose);

        TrajectorySequence trajectory1 = drivetrain.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .addTemporalMarker(() -> scorer.liftClaw())
                .UNSTABLE_addTemporalMarkerOffset(AutonConfig.TIME_FIRST_FLIP, () -> scorer.triggerPassThru())
                .lineTo(parkingZone2.vec())
                .lineToSplineHeading(medScoringPos)
                .UNSTABLE_addTemporalMarkerOffset(-RobotConfig.TIME_LIFT_MEDIUM, () -> scorer.setTargetLiftPos(PowerplayScorer.LiftPos.MED))
                .addTemporalMarker(() -> scorer.dropCone(PowerplayScorer.LiftPos.FIVE))
                .waitSeconds(AutonConfig.TIME_DROP)
                .UNSTABLE_addTemporalMarkerOffset(AutonConfig.TIME_POST_DROP, () -> scorer.triggerPassThru())
                .setReversed(false)
                .splineTo(sideTurnPos, isRight ? facingRight : facingLeft)
                .splineTo(stackPos, isRight ? facingRight : facingLeft)
                .addTemporalMarker(() -> scorer.grabCone())
                .waitSeconds(AutonConfig.TIME_GRAB)
                .UNSTABLE_addTemporalMarkerOffset(AutonConfig.TIME_POST_GRAB, () -> scorer.triggerPassThru())
                // loop below
                .setReversed(true)
                .splineTo(sideTurnPos, isRight ? facingLeft : facingRight)
                .splineToSplineHeading(medScoringPos, medScoringPos.getHeading() - facingLeft)
                .UNSTABLE_addTemporalMarkerOffset(-RobotConfig.TIME_LIFT_MEDIUM, () -> scorer.setTargetLiftPos(PowerplayScorer.LiftPos.MED))
                .addTemporalMarker(() -> scorer.dropCone(PowerplayScorer.LiftPos.FOUR))
                .waitSeconds(AutonConfig.TIME_DROP)
                .UNSTABLE_addTemporalMarkerOffset(AutonConfig.TIME_POST_DROP, () -> scorer.triggerPassThru())
                .setReversed(false)
                .splineTo(sideTurnPos, isRight ? facingRight : facingLeft)
                .splineTo(stackPos, isRight ? facingRight : facingLeft)
                .addTemporalMarker(() -> scorer.grabCone())
                .waitSeconds(AutonConfig.TIME_GRAB)
                .UNSTABLE_addTemporalMarkerOffset(AutonConfig.TIME_POST_GRAB, () -> scorer.triggerPassThru())
                .build();

        TrajectorySequence parkLeft = isRight ?
                drivetrain.trajectorySequenceBuilder(trajectory1.end())
                        .setReversed(true)
                        .splineTo(centerTurnPos, isRight ? facingLeft : facingRight)
                        .splineToSplineHeading(centerTallScoringPos, medScoringPos.getHeading() - facingLeft)
                        .UNSTABLE_addTemporalMarkerOffset(-RobotConfig.TIME_LIFT_MEDIUM, () -> scorer.setTargetLiftPos(PowerplayScorer.LiftPos.TALL))
                        .addTemporalMarker(() -> scorer.dropCone())
                        .waitSeconds(AutonConfig.TIME_DROP)
                        .UNSTABLE_addTemporalMarkerOffset(AutonConfig.TIME_POST_DROP, () -> scorer.triggerPassThru())
                        .lineToSplineHeading(parkingZone1)
                        .build() :
                drivetrain.trajectorySequenceBuilder(trajectory1.end())
                        .setReversed(true)
                        .splineTo(sideTurnPos, isRight ? facingLeft : facingRight)
                        .splineToSplineHeading(medScoringPos, medScoringPos.getHeading() - facingLeft)
                        .UNSTABLE_addTemporalMarkerOffset(-RobotConfig.TIME_LIFT_MEDIUM, () -> scorer.setTargetLiftPos(PowerplayScorer.LiftPos.MED))
                        .addTemporalMarker(() -> scorer.dropCone())
                        .waitSeconds(AutonConfig.TIME_DROP)
                        .UNSTABLE_addTemporalMarkerOffset(AutonConfig.TIME_POST_DROP, () -> scorer.triggerPassThru())
                        .setReversed(false)
                        .splineTo(sideTurnPos, isRight ? facingRight : facingLeft)
                        .splineTo(parkingZone1.vec(), parkingZone1.getHeading())
                        .build();

        TrajectorySequence parkMiddle = drivetrain.trajectorySequenceBuilder(trajectory1.end())
                .setReversed(true)
                .splineTo(sideTurnPos, isRight ? facingLeft : facingRight)
                .splineToSplineHeading(medScoringPos, medScoringPos.getHeading() - facingLeft)
                .UNSTABLE_addTemporalMarkerOffset(-RobotConfig.TIME_LIFT_MEDIUM, () -> scorer.setTargetLiftPos(PowerplayScorer.LiftPos.MED))
                .addTemporalMarker(() -> scorer.dropCone())
                .waitSeconds(AutonConfig.TIME_DROP)
                .UNSTABLE_addTemporalMarkerOffset(AutonConfig.TIME_POST_DROP, () -> scorer.triggerPassThru())
                .lineToSplineHeading(parkingZone2)
                .build();

        TrajectorySequence parkRight = isRight ?
                drivetrain.trajectorySequenceBuilder(trajectory1.end())
                        .setReversed(true)
                        .splineTo(sideTurnPos, isRight ? facingLeft : facingRight)
                        .splineToSplineHeading(medScoringPos, medScoringPos.getHeading() - facingLeft)
                        .UNSTABLE_addTemporalMarkerOffset(-RobotConfig.TIME_LIFT_MEDIUM, () -> scorer.setTargetLiftPos(PowerplayScorer.LiftPos.MED))
                        .addTemporalMarker(() -> scorer.dropCone())
                        .waitSeconds(AutonConfig.TIME_DROP)
                        .UNSTABLE_addTemporalMarkerOffset(AutonConfig.TIME_POST_DROP, () -> scorer.triggerPassThru())
                        .setReversed(false)
                        .splineTo(sideTurnPos, isRight ? facingRight : facingLeft)
                        .splineTo(parkingZone3.vec(), parkingZone3.getHeading())
                        .build() :
                drivetrain.trajectorySequenceBuilder(trajectory1.end())
                        .setReversed(true)
                        .splineTo(centerTurnPos, isRight ? facingLeft : facingRight)
                        .splineToSplineHeading(centerTallScoringPos, medScoringPos.getHeading() - facingLeft)
                        .UNSTABLE_addTemporalMarkerOffset(-RobotConfig.TIME_LIFT_MEDIUM, () -> scorer.setTargetLiftPos(PowerplayScorer.LiftPos.TALL))
                        .addTemporalMarker(() -> scorer.dropCone())
                        .waitSeconds(AutonConfig.TIME_DROP)
                        .UNSTABLE_addTemporalMarkerOffset(AutonConfig.TIME_POST_DROP, () -> scorer.triggerPassThru())
                        .lineToSplineHeading(parkingZone3)
                        .build();

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

            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT) {
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

                    if (tagOfInterest == null) {
                        myTelemetry.addLine("(The tag has never been seen)");
                    } else {
                        myTelemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            } else {
                myTelemetry.addLine("Don't see tag of interest :(");

                if (tagOfInterest == null) {
                    myTelemetry.addLine("(The tag has never been seen)");
                } else {
                    myTelemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);

                }

            }

            myTelemetry.update();
//            sleep(20);
            scorer.runClaw();
        }

        //START IS HERE//

        camera.stopStreaming();
        camera.closeCameraDevice();

        autonomousTimer.reset();

        if (tagOfInterest != null) {
            myTelemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            myTelemetry.update();
        } else {
            myTelemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            myTelemetry.update();
        }


        while (opModeIsActive()) {

            for (LynxModule hub : hubs) {
                hub.clearBulkCache();
            }

            if (!hasParked && !drivetrain.isBusy() && (autonomousTimer.seconds() >= 3)) {

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
            drivetrain.update();
            scorer.runLiftToPos();
            scorer.runPassThru();
            scorer.runPivot();
            scorer.runClaw();
            scorer.runConeArms(0, 0);

            // everything below is telemetry
            scorer.printTelemetry(myTelemetry);
            myTelemetry.update();

        }
    }

    @SuppressLint("DefaultLocale")
    void tagToTelemetry(@NonNull AprilTagDetection detection) {
        myTelemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
    }
}
