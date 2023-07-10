package org.firstinspires.ftc.teamcode.opmodes;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.control.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.control.HeadingHolder;
import org.firstinspires.ftc.teamcode.control.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.robot.PowerplayLift;
import org.firstinspires.ftc.teamcode.robot.PowerplayScorer;
import org.firstinspires.ftc.teamcode.subsystems.AprilTagCamera;
import org.firstinspires.ftc.teamcode.subsystems.AutonMecanumDrivetrain;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.List;

@Config
public abstract class BaseAuton extends LinearOpMode {

    private MultipleTelemetry myTelemetry;
    private AutonMecanumDrivetrain drivetrain;
    private PowerplayScorer scorer;
    private List<LynxModule> hubs;

    public static double
            MED_ANGLE = 35,
            MED_X = 31,
            MED_Y = -17.5,
            TALL_ANGLE = 135,
            TALL_X = 31,
            TALL_Y = -7.5,
            STACK_X = 59,
            STACK_Y = -12,
            STACK_ANGLE_OFFSET = 5,
            TURN_POS_X = 46,
            ONE_TILE = 24,
            TIME_PRE_GRAB = 1,
            TIME_GRAB = 1,
            TIME_POST_GRAB = 1,
            TIME_PRE_DROP = 1,
            TIME_DROP = 1,
            TIME_POST_DROP = 1,
            TIME_FIRST_FLIP = 1,
            TIME_LIFT_MEDIUM = 0.8,
            TIME_LIFT_TALL = 1.0,
            MAIN_Y = -12.5,
            ZONE_1_X = 12.5,
            ZONE_2_X = 35,
            ZONE_3_X = 57,
            STARTING_Y = -62.5;

    public enum Side {
        LEFT, RIGHT
    }

    public void runOpMode(PowerplayLift.Position pole, Side sideEnum) throws InterruptedException {
        drivetrain = new AutonMecanumDrivetrain(hardwareMap);
        scorer = new PowerplayScorer(hardwareMap);

        boolean isRight = sideEnum == Side.RIGHT;
        double side = isRight ? 1 : -1;

        double centerPathX = side * ZONE_2_X;

        double facingRight = Math.toRadians(0);
        double facingForward = Math.toRadians(90);
        double facingLeft = Math.toRadians(180);
        double stack = side * Math.toRadians(STACK_ANGLE_OFFSET);

        Vector2d stackPos = new Vector2d(side * STACK_X, STACK_Y);
        Vector2d sideTurnPos = new Vector2d(side * TURN_POS_X, MAIN_Y);
        Pose2d tallScoringPos = new Pose2d(side * TALL_X, TALL_Y, Math.toRadians(isRight ? TALL_ANGLE : 180 - TALL_ANGLE));
        Pose2d medScoringPos = new Pose2d(side * MED_X, MED_Y, Math.toRadians(isRight ? MED_ANGLE : 180 - MED_ANGLE));
        Vector2d centerTurnPos = new Vector2d(sideTurnPos.getX() - side * ONE_TILE, sideTurnPos.getY());
        Pose2d centerTallScoringPos = new Pose2d(medScoringPos.getX() - side * ONE_TILE, medScoringPos.getY(), medScoringPos.getHeading());

        Pose2d parkingZone1, parkingZone2, parkingZone3;

        Pose2d startPose = new Pose2d(centerPathX, STARTING_Y, facingForward);

        TrajectorySequence firstConeTrajectory, parkInZone1, parkInZone2, parkInZone3;

        drivetrain.setPoseEstimate(startPose);

        switch (pole) {
            case TALL:
                parkingZone1 = new Pose2d(side * (isRight ? ZONE_1_X : ZONE_3_X), MAIN_Y, !isRight ? facingRight : facingLeft);
                parkingZone2 = new Pose2d(centerPathX, MAIN_Y, parkingZone1.getHeading());
                parkingZone3 = new Pose2d(side * (isRight ? ZONE_3_X : ZONE_1_X), MAIN_Y, parkingZone1.getHeading());

                firstConeTrajectory = drivetrain.trajectorySequenceBuilder(startPose)
                        .setReversed(false)
                        .addTemporalMarker(() -> scorer.liftClaw())
                        .splineTo(new Vector2d(centerPathX, -25), facingForward)
                        .splineToSplineHeading(tallScoringPos, tallScoringPos.getHeading())
                        .UNSTABLE_addTemporalMarkerOffset(-TIME_LIFT_TALL, () -> scorer.setTargetLiftPos(PowerplayLift.Position.TALL))
                        .addTemporalMarker(() -> scorer.dropCone(PowerplayLift.Position.FIVE))
                        .waitSeconds(TIME_DROP)
                        .UNSTABLE_addTemporalMarkerOffset(TIME_POST_DROP, () -> scorer.passthrough.trigger())
                        .setReversed(true)
                        .setTangent(tallScoringPos.getHeading() + facingLeft)
                        .splineTo(sideTurnPos, stack + (isRight ? facingRight : facingLeft))
                        .splineTo(stackPos, isRight ? facingRight : facingLeft)
                        .addTemporalMarker(() -> scorer.grabCone())
                        .waitSeconds(TIME_GRAB)
                        .UNSTABLE_addTemporalMarkerOffset(TIME_POST_GRAB, () -> scorer.passthrough.trigger())
                        // loop below
                        .setReversed(false)
                        .splineTo(sideTurnPos, stack + (isRight ? facingLeft : facingRight))
                        .splineToSplineHeading(tallScoringPos, tallScoringPos.getHeading())
                        .UNSTABLE_addTemporalMarkerOffset(-TIME_LIFT_TALL, () -> scorer.setTargetLiftPos(PowerplayLift.Position.TALL))
                        .addTemporalMarker(() -> scorer.dropCone(PowerplayLift.Position.FOUR))
                        .waitSeconds(TIME_DROP)
                        .UNSTABLE_addTemporalMarkerOffset(TIME_POST_DROP, () -> scorer.passthrough.trigger())
                        .setReversed(true)
                        .setTangent(tallScoringPos.getHeading() + facingLeft)
                        .splineTo(sideTurnPos, stack + (isRight ? facingRight : facingLeft))
                        .splineTo(stackPos, isRight ? facingRight : facingLeft)
                        .addTemporalMarker(() -> scorer.grabCone())
                        .waitSeconds(TIME_GRAB)
                        .UNSTABLE_addTemporalMarkerOffset(TIME_POST_GRAB, () -> scorer.passthrough.trigger())
                        .build();

                parkInZone1 = isRight ?
                        // right side of field
                        drivetrain.trajectorySequenceBuilder(firstConeTrajectory.end())
                                .setReversed(false)
                                .splineTo(centerTurnPos, facingLeft)
                                .splineTo(centerTallScoringPos.vec(), centerTallScoringPos.getHeading() + facingLeft)
                                .UNSTABLE_addTemporalMarkerOffset(-TIME_LIFT_TALL, () -> scorer.setTargetLiftPos(PowerplayLift.Position.TALL))
                                .addTemporalMarker(() -> scorer.dropCone())
                                .waitSeconds(TIME_DROP)
                                .UNSTABLE_addTemporalMarkerOffset(TIME_POST_DROP, () -> scorer.passthrough.trigger())
                                .lineToSplineHeading(parkingZone1)
                                .build() :
                        // left side of field
                        drivetrain.trajectorySequenceBuilder(firstConeTrajectory.end())
                                .setReversed(false)
                                .splineTo(sideTurnPos, stack + facingRight)
                                .splineToSplineHeading(tallScoringPos, tallScoringPos.getHeading())
                                .UNSTABLE_addTemporalMarkerOffset(-TIME_LIFT_TALL, () -> scorer.setTargetLiftPos(PowerplayLift.Position.TALL))
                                .addTemporalMarker(() -> scorer.dropCone())
                                .waitSeconds(TIME_DROP)
                                .UNSTABLE_addTemporalMarkerOffset(TIME_POST_DROP, () -> scorer.passthrough.trigger())
                                .setReversed(true)
                                .splineTo(sideTurnPos, facingLeft)
                                .splineTo(parkingZone1.vec(), facingLeft)
                                .build();

                parkInZone2 = drivetrain.trajectorySequenceBuilder(firstConeTrajectory.end())
                        .setReversed(false)
                        .splineTo(sideTurnPos, stack + (isRight ? facingLeft : facingRight))
                        .splineToSplineHeading(tallScoringPos, tallScoringPos.getHeading())
                        .UNSTABLE_addTemporalMarkerOffset(-TIME_LIFT_TALL, () -> scorer.setTargetLiftPos(PowerplayLift.Position.TALL))
                        .addTemporalMarker(() -> scorer.dropCone())
                        .waitSeconds(TIME_DROP)
                        .UNSTABLE_addTemporalMarkerOffset(TIME_POST_DROP, () -> scorer.passthrough.trigger())
                        .lineToSplineHeading(parkingZone2)
                        .build();

                parkInZone3 = isRight ?
                        // right side of field
                        drivetrain.trajectorySequenceBuilder(firstConeTrajectory.end())
                                .setReversed(false)
                                .splineTo(sideTurnPos, stack + facingLeft)
                                .splineToSplineHeading(tallScoringPos, tallScoringPos.getHeading())
                                .UNSTABLE_addTemporalMarkerOffset(-TIME_LIFT_TALL, () -> scorer.setTargetLiftPos(PowerplayLift.Position.TALL))
                                .addTemporalMarker(() -> scorer.dropCone())
                                .waitSeconds(TIME_DROP)
                                .UNSTABLE_addTemporalMarkerOffset(TIME_POST_DROP, () -> scorer.passthrough.trigger())
                                .setReversed(true)
                                .splineTo(sideTurnPos, facingRight)
                                .splineTo(parkingZone3.vec(), facingRight)
                                .build() :
                        // left side of field
                        drivetrain.trajectorySequenceBuilder(firstConeTrajectory.end())
                                .setReversed(false)
                                .splineTo(centerTurnPos, facingRight)
                                .splineTo(centerTallScoringPos.vec(), centerTallScoringPos.getHeading() + facingLeft)
                                .UNSTABLE_addTemporalMarkerOffset(-TIME_LIFT_TALL, () -> scorer.setTargetLiftPos(PowerplayLift.Position.TALL))
                                .addTemporalMarker(() -> scorer.dropCone())
                                .waitSeconds(TIME_DROP)
                                .UNSTABLE_addTemporalMarkerOffset(TIME_POST_DROP, () -> scorer.passthrough.trigger())
                                .lineToSplineHeading(parkingZone3)
                                .build();

                HeadingHolder.setHeading(isRight ? 90.0 : 270.0);
                break;
            default:
            case MED:
                parkingZone1 = new Pose2d(side * (isRight ? ZONE_1_X : ZONE_3_X), MAIN_Y, isRight ? facingRight : facingLeft);
                parkingZone2 = new Pose2d(centerPathX, MAIN_Y, parkingZone1.getHeading());
                parkingZone3 = new Pose2d(side * (isRight ? ZONE_3_X : ZONE_1_X), MAIN_Y, parkingZone1.getHeading());

                firstConeTrajectory = drivetrain.trajectorySequenceBuilder(startPose)
                        .setReversed(true)
                        .addTemporalMarker(() -> scorer.liftClaw())
                        .lineTo(parkingZone2.vec())
                        .lineToSplineHeading(medScoringPos)
                        .UNSTABLE_addTemporalMarkerOffset(-TIME_FIRST_FLIP, () -> scorer.passthrough.trigger())
                        .UNSTABLE_addTemporalMarkerOffset(-TIME_LIFT_MEDIUM, () -> scorer.setTargetLiftPos(PowerplayLift.Position.MED))
//                        .waitSeconds(TIME_PRE_DROP)
//                        .addTemporalMarker(() -> scorer.dropCone(PowerplayLift.Position.FIVE))
//                        .waitSeconds(TIME_DROP)
//                        .UNSTABLE_addTemporalMarkerOffset(TIME_POST_DROP, () -> scorer.passthrough.trigger())
//                        .setReversed(false)
//                        .splineTo(sideTurnPos, stack + (isRight ? facingRight : facingLeft))
//                        .splineTo(stackPos, isRight ? facingRight : facingLeft)
//                        .addTemporalMarker(() -> scorer.grabCone())
//                        .waitSeconds(TIME_GRAB)
//                        .UNSTABLE_addTemporalMarkerOffset(TIME_POST_GRAB, () -> scorer.passthrough.trigger())
//                        // loop below
//                        .setReversed(true)
//                        .splineTo(sideTurnPos, stack + (isRight ? facingLeft : facingRight))
//                        .splineToSplineHeading(medScoringPos, medScoringPos.getHeading() - facingLeft)
//                        .UNSTABLE_addTemporalMarkerOffset(-TIME_LIFT_MEDIUM, () -> scorer.setTargetLiftPos(PowerplayLift.Position.MED))
//                        .waitSeconds(TIME_PRE_DROP)
//                        .addTemporalMarker(() -> scorer.dropCone(PowerplayLift.Position.FOUR))
//                        .waitSeconds(TIME_DROP)
//                        .UNSTABLE_addTemporalMarkerOffset(TIME_POST_DROP, () -> scorer.passthrough.trigger())
//                        .setReversed(false)
//                        .splineTo(sideTurnPos, stack + (isRight ? facingRight : facingLeft))
//                        .splineTo(stackPos, isRight ? facingRight : facingLeft)
//                        .addTemporalMarker(() -> scorer.grabCone())
//                        .waitSeconds(TIME_GRAB)
//                        .UNSTABLE_addTemporalMarkerOffset(TIME_POST_GRAB, () -> scorer.passthrough.trigger())
                        .build();

                parkInZone1 = isRight ?
                        drivetrain.trajectorySequenceBuilder(firstConeTrajectory.end())
                                .setReversed(true)
                                .splineTo(centerTurnPos, facingLeft)
                                .splineToSplineHeading(centerTallScoringPos, medScoringPos.getHeading() - facingLeft)
                                .UNSTABLE_addTemporalMarkerOffset(-TIME_LIFT_TALL, () -> scorer.setTargetLiftPos(PowerplayLift.Position.TALL))
                                .waitSeconds(TIME_PRE_DROP)
                                .addTemporalMarker(() -> scorer.dropCone())
                                .waitSeconds(TIME_DROP)
                                .UNSTABLE_addTemporalMarkerOffset(TIME_POST_DROP, () -> scorer.passthrough.trigger())
                                .lineToSplineHeading(parkingZone1)
                                .build() :
                        drivetrain.trajectorySequenceBuilder(firstConeTrajectory.end())
                                .setReversed(true)
                                .splineTo(sideTurnPos, stack + facingRight)
                                .splineToSplineHeading(medScoringPos, medScoringPos.getHeading() - facingLeft)
                                .UNSTABLE_addTemporalMarkerOffset(-TIME_LIFT_MEDIUM, () -> scorer.setTargetLiftPos(PowerplayLift.Position.MED))
                                .waitSeconds(TIME_PRE_DROP)
                                .addTemporalMarker(() -> scorer.dropCone())
                                .waitSeconds(TIME_DROP)
                                .UNSTABLE_addTemporalMarkerOffset(TIME_POST_DROP, () -> scorer.passthrough.trigger())
                                .setReversed(false)
                                .splineTo(sideTurnPos, facingLeft)
                                .splineTo(parkingZone1.vec(), parkingZone1.getHeading())
                                .build();

                parkInZone2 = drivetrain.trajectorySequenceBuilder(firstConeTrajectory.end())
                        .setReversed(true)
                        .splineTo(sideTurnPos, stack + (isRight ? facingLeft : facingRight))
                        .splineToSplineHeading(medScoringPos, medScoringPos.getHeading() - facingLeft)
                        .UNSTABLE_addTemporalMarkerOffset(-TIME_LIFT_MEDIUM, () -> scorer.setTargetLiftPos(PowerplayLift.Position.MED))
                        .waitSeconds(TIME_PRE_DROP)
                        .addTemporalMarker(() -> scorer.dropCone())
                        .waitSeconds(TIME_DROP)
                        .UNSTABLE_addTemporalMarkerOffset(TIME_POST_DROP, () -> scorer.passthrough.trigger())
                        .lineToSplineHeading(parkingZone2)
                        .build();

                parkInZone3 = isRight ?
                        drivetrain.trajectorySequenceBuilder(firstConeTrajectory.end())
                                .setReversed(true)
                                .splineTo(sideTurnPos, stack + facingLeft)
                                .splineToSplineHeading(medScoringPos, medScoringPos.getHeading() - facingLeft)
                                .UNSTABLE_addTemporalMarkerOffset(-TIME_LIFT_MEDIUM, () -> scorer.setTargetLiftPos(PowerplayLift.Position.MED))
                                .waitSeconds(TIME_PRE_DROP)
                                .addTemporalMarker(() -> scorer.dropCone())
                                .waitSeconds(TIME_DROP)
                                .UNSTABLE_addTemporalMarkerOffset(TIME_POST_DROP, () -> scorer.passthrough.trigger())
                                .setReversed(false)
                                .splineTo(sideTurnPos, facingRight)
                                .splineTo(parkingZone3.vec(), parkingZone3.getHeading())
                                .build() :
                        drivetrain.trajectorySequenceBuilder(firstConeTrajectory.end())
                                .setReversed(true)
                                .splineTo(centerTurnPos, facingRight)
                                .splineToSplineHeading(centerTallScoringPos, medScoringPos.getHeading() - facingLeft)
                                .UNSTABLE_addTemporalMarkerOffset(-TIME_LIFT_TALL, () -> scorer.setTargetLiftPos(PowerplayLift.Position.TALL))
                                .waitSeconds(TIME_PRE_DROP)
                                .addTemporalMarker(() -> scorer.dropCone())
                                .waitSeconds(TIME_DROP)
                                .UNSTABLE_addTemporalMarkerOffset(TIME_POST_DROP, () -> scorer.passthrough.trigger())
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
                AprilTagCamera.TAG_SIZE,
                AprilTagCamera.CAMERA_FX,
                AprilTagCamera.CAMERA_FY,
                AprilTagCamera.CAMERA_CX,
                AprilTagCamera.CAMERA_CY
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

        drivetrain.followTrajectorySequenceAsync(firstConeTrajectory);

        hubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */

        scorer.passthrough.claw.setActivated(true);
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
            scorer.passthrough.claw.run();
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

//            if (!hasParked && !drivetrain.isBusy() && (autonomousTimer.seconds() >= 3)) {
//
//                drivetrain.followTrajectorySequenceAsync(
//                        tagOfInterest == null ? parkInZone2 :
//                                tagOfInterest.id == LEFT ? parkInZone1 :
//                                        tagOfInterest.id == RIGHT ? parkInZone3 :
//                                                parkInZone2
//                );
//
//                hasParked = true;
//            }

            scorer.lift.readPosition();
            drivetrain.update();
            scorer.lift.runToPosition();
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
