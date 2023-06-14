package org.firstinspires.ftc.teamcode.autonomous.opmode;


import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
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


public abstract class BaseAuton extends LinearOpMode {

    MultipleTelemetry myTelemetry;
    AutonMecanumDrive drivetrain;
    PowerplayScorer scorer;
    List<LynxModule> hubs;

    public enum Side {
        LEFT, RIGHT
    }

    public void runOpMode(PowerplayScorer.LiftPos pole, Side sideEnum) throws InterruptedException {
        drivetrain = new AutonMecanumDrive(hardwareMap);
        scorer = new PowerplayScorer(hardwareMap);

        boolean isRight = sideEnum == Side.RIGHT;
        double side = isRight ? 1 : -1;

        double centerPathX = side * AutonConfig.ZONE_2_X;

        double facingRight = Math.toRadians(0);
        double facingForward = Math.toRadians(90);
        double facingLeft = Math.toRadians(180);
        double stack = side * Math.toRadians(AutonConfig.STACK_ANGLE_OFFSET);

        Vector2d stackPos = new Vector2d(side * AutonConfig.STACK_X, AutonConfig.STACK_Y);
        Vector2d sideTurnPos = new Vector2d(side * AutonConfig.TURN_POS_X, AutonConfig.MAIN_Y);
        Pose2d tallScoringPos = new Pose2d(side * AutonConfig.TALL_X, AutonConfig.TALL_Y, Math.toRadians(isRight ? AutonConfig.TALL_ANGLE : 180 - AutonConfig.TALL_ANGLE));
        Pose2d medScoringPos = new Pose2d(side * AutonConfig.MED_X, AutonConfig.MED_Y, Math.toRadians(isRight ? AutonConfig.MED_ANGLE : 180 - AutonConfig.MED_ANGLE));
        Vector2d centerTurnPos = new Vector2d(sideTurnPos.getX() - side * AutonConfig.ONE_TILE, sideTurnPos.getY());
        Pose2d centerTallScoringPos = new Pose2d(medScoringPos.getX() - side * AutonConfig.ONE_TILE, medScoringPos.getY(), medScoringPos.getHeading());

        Pose2d parkingZone1, parkingZone2, parkingZone3;

        Pose2d startPose = new Pose2d(centerPathX, AutonConfig.STARTING_Y, facingForward);

        TrajectorySequence scoringTrajectory, parkInZone1, parkInZone2, parkInZone3;

        drivetrain.setPoseEstimate(startPose);

        switch (pole) {
            case TALL:
                parkingZone1 = new Pose2d(side * (isRight ? AutonConfig.ZONE_1_X : AutonConfig.ZONE_3_X), AutonConfig.MAIN_Y, !isRight ? facingRight : facingLeft);
                parkingZone2 = new Pose2d(centerPathX, AutonConfig.MAIN_Y, parkingZone1.getHeading());
                parkingZone3 = new Pose2d(side * (isRight ? AutonConfig.ZONE_3_X : AutonConfig.ZONE_1_X), AutonConfig.MAIN_Y, parkingZone1.getHeading());

                scoringTrajectory = drivetrain.trajectorySequenceBuilder(startPose)
                        .setReversed(false)
                        .addTemporalMarker(() -> scorer.liftClaw())
                        .splineTo(new Vector2d(centerPathX, -25), facingForward)
                        .splineToSplineHeading(tallScoringPos, tallScoringPos.getHeading())
                        .UNSTABLE_addTemporalMarkerOffset(-RobotConfig.TIME_LIFT_TALL, () -> scorer.setTargetLiftPos(PowerplayScorer.LiftPos.TALL))
                        .addTemporalMarker(() -> scorer.dropCone(PowerplayScorer.LiftPos.FIVE))
                        .waitSeconds(AutonConfig.TIME_DROP)
                        .UNSTABLE_addTemporalMarkerOffset(AutonConfig.TIME_POST_DROP, () -> scorer.passthrough.trigger())
                        .setReversed(true)
                        .setTangent(tallScoringPos.getHeading() + facingLeft)
                        .splineTo(sideTurnPos, stack + (isRight ? facingRight : facingLeft))
                        .splineTo(stackPos, isRight ? facingRight : facingLeft)
                        .addTemporalMarker(() -> scorer.grabCone())
                        .waitSeconds(AutonConfig.TIME_GRAB)
                        .UNSTABLE_addTemporalMarkerOffset(AutonConfig.TIME_POST_GRAB, () -> scorer.passthrough.trigger())
                        // loop below
                        .setReversed(false)
                        .splineTo(sideTurnPos, stack + (isRight ? facingLeft : facingRight))
                        .splineToSplineHeading(tallScoringPos, tallScoringPos.getHeading())
                        .UNSTABLE_addTemporalMarkerOffset(-RobotConfig.TIME_LIFT_TALL, () -> scorer.setTargetLiftPos(PowerplayScorer.LiftPos.TALL))
                        .addTemporalMarker(() -> scorer.dropCone(PowerplayScorer.LiftPos.FOUR))
                        .waitSeconds(AutonConfig.TIME_DROP)
                        .UNSTABLE_addTemporalMarkerOffset(AutonConfig.TIME_POST_DROP, () -> scorer.passthrough.trigger())
                        .setReversed(true)
                        .setTangent(tallScoringPos.getHeading() + facingLeft)
                        .splineTo(sideTurnPos, stack + (isRight ? facingRight : facingLeft))
                        .splineTo(stackPos, isRight ? facingRight : facingLeft)
                        .addTemporalMarker(() -> scorer.grabCone())
                        .waitSeconds(AutonConfig.TIME_GRAB)
                        .UNSTABLE_addTemporalMarkerOffset(AutonConfig.TIME_POST_GRAB, () -> scorer.passthrough.trigger())
                        .build();

                parkInZone1 = isRight ?
                        // right side of field
                        drivetrain.trajectorySequenceBuilder(scoringTrajectory.end())
                                .setReversed(false)
                                .splineTo(centerTurnPos, facingLeft)
                                .splineTo(centerTallScoringPos.vec(), centerTallScoringPos.getHeading() + facingLeft)
                                .UNSTABLE_addTemporalMarkerOffset(-RobotConfig.TIME_LIFT_TALL, () -> scorer.setTargetLiftPos(PowerplayScorer.LiftPos.TALL))
                                .addTemporalMarker(() -> scorer.dropCone())
                                .waitSeconds(AutonConfig.TIME_DROP)
                                .UNSTABLE_addTemporalMarkerOffset(AutonConfig.TIME_POST_DROP, () -> scorer.passthrough.trigger())
                                .lineToSplineHeading(parkingZone1)
                                .build() :
                        // left side of field
                        drivetrain.trajectorySequenceBuilder(scoringTrajectory.end())
                                .setReversed(false)
                                .splineTo(sideTurnPos, stack + facingRight)
                                .splineToSplineHeading(tallScoringPos, tallScoringPos.getHeading())
                                .UNSTABLE_addTemporalMarkerOffset(-RobotConfig.TIME_LIFT_TALL, () -> scorer.setTargetLiftPos(PowerplayScorer.LiftPos.TALL))
                                .addTemporalMarker(() -> scorer.dropCone())
                                .waitSeconds(AutonConfig.TIME_DROP)
                                .UNSTABLE_addTemporalMarkerOffset(AutonConfig.TIME_POST_DROP, () -> scorer.passthrough.trigger())
                                .setReversed(true)
                                .splineTo(sideTurnPos, facingLeft)
                                .splineTo(parkingZone1.vec(), facingLeft)
                                .build();

                parkInZone2 = drivetrain.trajectorySequenceBuilder(scoringTrajectory.end())
                        .setReversed(false)
                        .splineTo(sideTurnPos, stack + (isRight ? facingLeft : facingRight))
                        .splineToSplineHeading(tallScoringPos, tallScoringPos.getHeading())
                        .UNSTABLE_addTemporalMarkerOffset(-RobotConfig.TIME_LIFT_TALL, () -> scorer.setTargetLiftPos(PowerplayScorer.LiftPos.TALL))
                        .addTemporalMarker(() -> scorer.dropCone())
                        .waitSeconds(AutonConfig.TIME_DROP)
                        .UNSTABLE_addTemporalMarkerOffset(AutonConfig.TIME_POST_DROP, () -> scorer.passthrough.trigger())
                        .lineToSplineHeading(parkingZone2)
                        .build();

                parkInZone3 = isRight ?
                        // right side of field
                        drivetrain.trajectorySequenceBuilder(scoringTrajectory.end())
                                .setReversed(false)
                                .splineTo(sideTurnPos, stack + facingLeft)
                                .splineToSplineHeading(tallScoringPos, tallScoringPos.getHeading())
                                .UNSTABLE_addTemporalMarkerOffset(-RobotConfig.TIME_LIFT_TALL, () -> scorer.setTargetLiftPos(PowerplayScorer.LiftPos.TALL))
                                .addTemporalMarker(() -> scorer.dropCone())
                                .waitSeconds(AutonConfig.TIME_DROP)
                                .UNSTABLE_addTemporalMarkerOffset(AutonConfig.TIME_POST_DROP, () -> scorer.passthrough.trigger())
                                .setReversed(true)
                                .splineTo(sideTurnPos, facingRight)
                                .splineTo(parkingZone3.vec(), facingRight)
                                .build() :
                        // left side of field
                        drivetrain.trajectorySequenceBuilder(scoringTrajectory.end())
                                .setReversed(false)
                                .splineTo(centerTurnPos, facingRight)
                                .splineTo(centerTallScoringPos.vec(), centerTallScoringPos.getHeading() + facingLeft)
                                .UNSTABLE_addTemporalMarkerOffset(-RobotConfig.TIME_LIFT_TALL, () -> scorer.setTargetLiftPos(PowerplayScorer.LiftPos.TALL))
                                .addTemporalMarker(() -> scorer.dropCone())
                                .waitSeconds(AutonConfig.TIME_DROP)
                                .UNSTABLE_addTemporalMarkerOffset(AutonConfig.TIME_POST_DROP, () -> scorer.passthrough.trigger())
                                .lineToSplineHeading(parkingZone3)
                                .build();

                HeadingHolder.setHeading(isRight ? 90.0 : 270.0);
                break;
            default:
            case MED:
                parkingZone1 = new Pose2d(side * (isRight ? AutonConfig.ZONE_1_X : AutonConfig.ZONE_3_X), AutonConfig.MAIN_Y, isRight ? facingRight : facingLeft);
                parkingZone2 = new Pose2d(centerPathX, AutonConfig.MAIN_Y, parkingZone1.getHeading());
                parkingZone3 = new Pose2d(side * (isRight ? AutonConfig.ZONE_3_X : AutonConfig.ZONE_1_X), AutonConfig.MAIN_Y, parkingZone1.getHeading());

                scoringTrajectory = drivetrain.trajectorySequenceBuilder(startPose)
                        .setReversed(true)
                        .addTemporalMarker(() -> scorer.liftClaw())
                        .UNSTABLE_addTemporalMarkerOffset(AutonConfig.TIME_FIRST_FLIP, () -> scorer.passthrough.trigger())
                        .lineTo(parkingZone2.vec())
                        .lineToSplineHeading(medScoringPos)
                        .UNSTABLE_addTemporalMarkerOffset(-RobotConfig.TIME_LIFT_MEDIUM, () -> scorer.setTargetLiftPos(PowerplayScorer.LiftPos.MED))
                        .addTemporalMarker(() -> scorer.dropCone(PowerplayScorer.LiftPos.FIVE))
                        .waitSeconds(AutonConfig.TIME_DROP)
                        .UNSTABLE_addTemporalMarkerOffset(AutonConfig.TIME_POST_DROP, () -> scorer.passthrough.trigger())
                        .setReversed(false)
                        .splineTo(sideTurnPos, stack + (isRight ? facingRight : facingLeft))
                        .splineTo(stackPos, isRight ? facingRight : facingLeft)
                        .addTemporalMarker(() -> scorer.grabCone())
                        .waitSeconds(AutonConfig.TIME_GRAB)
                        .UNSTABLE_addTemporalMarkerOffset(AutonConfig.TIME_POST_GRAB, () -> scorer.passthrough.trigger())
                        // loop below
                        .setReversed(true)
                        .splineTo(sideTurnPos, stack + (isRight ? facingLeft : facingRight))
                        .splineToSplineHeading(medScoringPos, medScoringPos.getHeading() - facingLeft)
                        .UNSTABLE_addTemporalMarkerOffset(-RobotConfig.TIME_LIFT_MEDIUM, () -> scorer.setTargetLiftPos(PowerplayScorer.LiftPos.MED))
                        .addTemporalMarker(() -> scorer.dropCone(PowerplayScorer.LiftPos.FOUR))
                        .waitSeconds(AutonConfig.TIME_DROP)
                        .UNSTABLE_addTemporalMarkerOffset(AutonConfig.TIME_POST_DROP, () -> scorer.passthrough.trigger())
                        .setReversed(false)
                        .splineTo(sideTurnPos, stack + (isRight ? facingRight : facingLeft))
                        .splineTo(stackPos, isRight ? facingRight : facingLeft)
                        .addTemporalMarker(() -> scorer.grabCone())
                        .waitSeconds(AutonConfig.TIME_GRAB)
                        .UNSTABLE_addTemporalMarkerOffset(AutonConfig.TIME_POST_GRAB, () -> scorer.passthrough.trigger())
                        .build();

                parkInZone1 = isRight ?
                        drivetrain.trajectorySequenceBuilder(scoringTrajectory.end())
                                .setReversed(true)
                                .splineTo(centerTurnPos, facingLeft)
                                .splineToSplineHeading(centerTallScoringPos, medScoringPos.getHeading() - facingLeft)
                                .UNSTABLE_addTemporalMarkerOffset(-RobotConfig.TIME_LIFT_MEDIUM, () -> scorer.setTargetLiftPos(PowerplayScorer.LiftPos.TALL))
                                .addTemporalMarker(() -> scorer.dropCone())
                                .waitSeconds(AutonConfig.TIME_DROP)
                                .UNSTABLE_addTemporalMarkerOffset(AutonConfig.TIME_POST_DROP, () -> scorer.passthrough.trigger())
                                .lineToSplineHeading(parkingZone1)
                                .build() :
                        drivetrain.trajectorySequenceBuilder(scoringTrajectory.end())
                                .setReversed(true)
                                .splineTo(sideTurnPos, stack + facingRight)
                                .splineToSplineHeading(medScoringPos, medScoringPos.getHeading() - facingLeft)
                                .UNSTABLE_addTemporalMarkerOffset(-RobotConfig.TIME_LIFT_MEDIUM, () -> scorer.setTargetLiftPos(PowerplayScorer.LiftPos.MED))
                                .addTemporalMarker(() -> scorer.dropCone())
                                .waitSeconds(AutonConfig.TIME_DROP)
                                .UNSTABLE_addTemporalMarkerOffset(AutonConfig.TIME_POST_DROP, () -> scorer.passthrough.trigger())
                                .setReversed(false)
                                .splineTo(sideTurnPos, facingLeft)
                                .splineTo(parkingZone1.vec(), parkingZone1.getHeading())
                                .build();

                parkInZone2 = drivetrain.trajectorySequenceBuilder(scoringTrajectory.end())
                        .setReversed(true)
                        .splineTo(sideTurnPos, stack + (isRight ? facingLeft : facingRight))
                        .splineToSplineHeading(medScoringPos, medScoringPos.getHeading() - facingLeft)
                        .UNSTABLE_addTemporalMarkerOffset(-RobotConfig.TIME_LIFT_MEDIUM, () -> scorer.setTargetLiftPos(PowerplayScorer.LiftPos.MED))
                        .addTemporalMarker(() -> scorer.dropCone())
                        .waitSeconds(AutonConfig.TIME_DROP)
                        .UNSTABLE_addTemporalMarkerOffset(AutonConfig.TIME_POST_DROP, () -> scorer.passthrough.trigger())
                        .lineToSplineHeading(parkingZone2)
                        .build();

                parkInZone3 = isRight ?
                        drivetrain.trajectorySequenceBuilder(scoringTrajectory.end())
                                .setReversed(true)
                                .splineTo(sideTurnPos, stack + facingLeft)
                                .splineToSplineHeading(medScoringPos, medScoringPos.getHeading() - facingLeft)
                                .UNSTABLE_addTemporalMarkerOffset(-RobotConfig.TIME_LIFT_MEDIUM, () -> scorer.setTargetLiftPos(PowerplayScorer.LiftPos.MED))
                                .addTemporalMarker(() -> scorer.dropCone())
                                .waitSeconds(AutonConfig.TIME_DROP)
                                .UNSTABLE_addTemporalMarkerOffset(AutonConfig.TIME_POST_DROP, () -> scorer.passthrough.trigger())
                                .setReversed(false)
                                .splineTo(sideTurnPos, facingRight)
                                .splineTo(parkingZone3.vec(), parkingZone3.getHeading())
                                .build() :
                        drivetrain.trajectorySequenceBuilder(scoringTrajectory.end())
                                .setReversed(true)
                                .splineTo(centerTurnPos, facingRight)
                                .splineToSplineHeading(centerTallScoringPos, medScoringPos.getHeading() - facingLeft)
                                .UNSTABLE_addTemporalMarkerOffset(-RobotConfig.TIME_LIFT_MEDIUM, () -> scorer.setTargetLiftPos(PowerplayScorer.LiftPos.TALL))
                                .addTemporalMarker(() -> scorer.dropCone())
                                .waitSeconds(AutonConfig.TIME_DROP)
                                .UNSTABLE_addTemporalMarkerOffset(AutonConfig.TIME_POST_DROP, () -> scorer.passthrough.trigger())
                                .lineToSplineHeading(parkingZone3)
                                .build();

                HeadingHolder.setHeading(isRight ? 270.0 : 90.0);
                break;
        }

        // Lens intrinsics
        // UNITS ARE PIXELS
        // NOTE: this calibration is for the C920 webcam at 800x448.
        // You will need to do your own calibration for other configurations!
        int LEFT = 1;
        int MIDDLE = 2;
        int RIGHT = 3;

        ElapsedTime autonomousTimer = new ElapsedTime();

        boolean hasParked = false;

        AprilTagDetection tagOfInterest = null;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        AprilTagDetectionPipeline signalSleeveDetectionPipeline = new AprilTagDetectionPipeline(
                AutonConfig.TAG_SIZE,
                AutonConfig.CAMERA_FX,
                AutonConfig.CAMERA_FY,
                AutonConfig.CAMERA_CX,
                AutonConfig.CAMERA_CY
        );

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

        //  Initialize telemetry and dashboard
        myTelemetry = new MultipleTelemetry(telemetry);

        drivetrain.followTrajectorySequenceAsync(scoringTrajectory);

        hubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */

        scorer.passthrough.setClawOpen(false);
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
//            sleep(20);
            scorer.passthrough.runClaw();
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

            for (LynxModule hub : hubs) hub.clearBulkCache();

            if (!hasParked && !drivetrain.isBusy() && (autonomousTimer.seconds() >= 3)) {

                drivetrain.followTrajectorySequenceAsync(
                        tagOfInterest == null ? parkInZone2 :
                                tagOfInterest.id == LEFT ? parkInZone1 :
                                        tagOfInterest.id == RIGHT ? parkInZone3 :
                                                parkInZone2
                );

                hasParked = true;
            }

            scorer.updateLiftGains();
            scorer.lift.readPosition();
            drivetrain.update();
            scorer.lift.runToPosition();
            scorer.passthrough.run();
            scorer.passthrough.runPivot();
            scorer.passthrough.runClaw();
            scorer.run(0, 0);

            // everything below is telemetry
            scorer.printTelemetry(myTelemetry);
            myTelemetry.update();

        }
    }

    void tagToTelemetry(AprilTagDetection detection) {
        myTelemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
    }
}
