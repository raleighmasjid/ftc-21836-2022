package org.firstinspires.ftc.teamcode.opmodes;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.control.HeadingHolder;
import org.firstinspires.ftc.teamcode.control.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.robot.Lift;
import org.firstinspires.ftc.teamcode.robot.ScoringSystems;
import org.firstinspires.ftc.teamcode.subsystems.AprilTagCamera;
import org.firstinspires.ftc.teamcode.subsystems.AutonMecanumDrivetrain;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.List;

@Config
public abstract class BaseAuton extends LinearOpMode {

    private MultipleTelemetry myTelemetry;
    private AutonMecanumDrivetrain drivetrain;
    private ScoringSystems scorer;
    private List<LynxModule> hubs;
    private AprilTagCamera camera;

    public static double
            MED_ANGLE = 35,
            MED_X = 31,
            MED_Y = -17.5,
            TALL_ANGLE = 135,
            TALL_X = 31,
            TALL_Y = -7.5,
            STACK_X = 59,
            STACK_Y = -12,
            STACK_ANGLE_OFFSET_MED = 5,
            STACK_ANGLE_OFFSET_TALL = -2,
            ONE_TILE = 24,
            ZONE_1_X = 12.5,
            ZONE_2_X = 35,
            ZONE_3_X = 57,
            X_TURN_POS = 46,
            Y_START = -62.5,
            Y_MAIN_PATH = -12.5,
            TIME_PRE_GRAB = 1,
            TIME_GRAB = 1,
            TIME_GRAB_TO_FLIP = 1,
            TIME_PRE_DROP = 1,
            TIME_DROP = 1,
            TIME_DROP_TO_FLIP = 1,
            TIME_FIRST_FLIP = 1,
            TIME_LIFT_MEDIUM = 0.8,
            TIME_LIFT_TALL = 1.0;

    public enum Side {
        LEFT, RIGHT
    }

    public void runOpMode(Lift.Position pole, Side sideEnum) throws InterruptedException {
        drivetrain = new AutonMecanumDrivetrain(hardwareMap);
        scorer = new ScoringSystems(hardwareMap);

        boolean isRight = sideEnum == Side.RIGHT;
        double side = isRight ? 1 : -1;

        double X_START = side * ZONE_2_X;

        double facingRight = Math.toRadians(0);
        double facingForward = Math.toRadians(90);
        double facingLeft = Math.toRadians(180);

        Vector2d stackPos = new Vector2d(side * STACK_X, STACK_Y);
        Vector2d sideTurnPos = new Vector2d(side * X_TURN_POS, Y_MAIN_PATH);
        Pose2d tallScoringPos = new Pose2d(side * TALL_X, TALL_Y, Math.toRadians(isRight ? TALL_ANGLE : 180 - TALL_ANGLE));
        Pose2d medScoringPos = new Pose2d(side * MED_X, MED_Y, Math.toRadians(isRight ? MED_ANGLE : 180 - MED_ANGLE));
        Vector2d centerTurnPos = new Vector2d(sideTurnPos.getX() - side * ONE_TILE, sideTurnPos.getY());
        Pose2d centerTallScoringPos = new Pose2d(medScoringPos.getX() - side * ONE_TILE, medScoringPos.getY(), medScoringPos.getHeading());

        double zone1x = side * (isRight ? ZONE_1_X : ZONE_3_X);
        double zone3x = side * (isRight ? ZONE_3_X : ZONE_1_X);
        Pose2d parkingZone1 = new Pose2d(zone1x, Y_MAIN_PATH, isRight ? facingRight : facingLeft);
        Pose2d parkingZone2 = new Pose2d(X_START, Y_MAIN_PATH, parkingZone1.getHeading());
        Pose2d parkingZone3 = new Pose2d(zone3x, Y_MAIN_PATH, parkingZone1.getHeading());

        Pose2d startPose = new Pose2d(X_START, Y_START, facingForward);

        drivetrain.setPoseEstimate(startPose);

        double TIME_LIFT;
        Pose2d scoringPos;
        double stackOffset;

        switch (pole) {
            case TALL:
                TIME_LIFT = TIME_LIFT_TALL;
                scoringPos = tallScoringPos;
                stackOffset = Math.toRadians(STACK_ANGLE_OFFSET_TALL);
                break;
            default:
            case MED:
                TIME_LIFT = TIME_LIFT_MEDIUM;
                scoringPos = medScoringPos;
                stackOffset = Math.toRadians(STACK_ANGLE_OFFSET_MED);
                break;
        }
        stackOffset *= side;

        TrajectorySequence scoringTrajectory = drivetrain.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(() -> scorer.liftClaw())
                .lineTo(parkingZone2.vec())
                .lineToSplineHeading(scoringPos)
                .UNSTABLE_addTemporalMarkerOffset(-TIME_FIRST_FLIP, () -> scorer.passthrough.trigger())
                .UNSTABLE_addTemporalMarkerOffset(-TIME_LIFT, () -> scorer.setTargetLiftPos(pole))
                .waitSeconds(TIME_PRE_DROP)
                .addTemporalMarker(() -> scorer.dropCone(Lift.Position.FIVE))
                .waitSeconds(TIME_DROP)
                .UNSTABLE_addTemporalMarkerOffset(TIME_DROP_TO_FLIP, () -> scorer.passthrough.trigger())
                .splineTo(sideTurnPos, stackOffset + (isRight ? facingRight : facingLeft))
                .splineTo(stackPos, isRight ? facingRight : facingLeft)
                // loop below
                .waitSeconds(TIME_PRE_GRAB)
                .addTemporalMarker(() -> scorer.grabCone())
                .waitSeconds(TIME_GRAB)
                .UNSTABLE_addTemporalMarkerOffset(TIME_GRAB_TO_FLIP, () -> scorer.passthrough.trigger())
                .setReversed(true)
                .splineTo(sideTurnPos, stackOffset + (isRight ? facingLeft : facingRight))
                .splineToSplineHeading(scoringPos, scoringPos.getHeading() - facingLeft)
                .UNSTABLE_addTemporalMarkerOffset(-TIME_LIFT, () -> scorer.setTargetLiftPos(pole))
                .waitSeconds(TIME_PRE_DROP)
                .addTemporalMarker(() -> scorer.dropCone(Lift.Position.FOUR))
                .waitSeconds(TIME_DROP)
                .UNSTABLE_addTemporalMarkerOffset(TIME_DROP_TO_FLIP, () -> scorer.passthrough.trigger())
                .setReversed(false)
                .splineTo(sideTurnPos, stackOffset + (isRight ? facingRight : facingLeft))
                .splineTo(stackPos, isRight ? facingRight : facingLeft)
                .build();

        TrajectorySequence parkInZone1 = isRight ?
                drivetrain.trajectorySequenceBuilder(scoringTrajectory.end())
                        .setReversed(true)
                        .splineTo(centerTurnPos, facingLeft)
                        .splineToSplineHeading(centerTallScoringPos, medScoringPos.getHeading() - facingLeft)
                        .UNSTABLE_addTemporalMarkerOffset(-TIME_LIFT_TALL, () -> scorer.setTargetLiftPos(Lift.Position.TALL))
                        .waitSeconds(TIME_PRE_DROP)
                        .addTemporalMarker(() -> scorer.dropCone())
                        .waitSeconds(TIME_DROP)
                        .UNSTABLE_addTemporalMarkerOffset(TIME_DROP_TO_FLIP, () -> scorer.passthrough.trigger())
                        .lineToSplineHeading(parkingZone1)
                        .build() :
                drivetrain.trajectorySequenceBuilder(scoringTrajectory.end())
                        .setReversed(true)
                        .splineTo(sideTurnPos, stackOffset + facingRight)
                        .splineToSplineHeading(medScoringPos, medScoringPos.getHeading() - facingLeft)
                        .UNSTABLE_addTemporalMarkerOffset(-TIME_LIFT, () -> scorer.setTargetLiftPos(pole))
                        .waitSeconds(TIME_PRE_DROP)
                        .addTemporalMarker(() -> scorer.dropCone())
                        .waitSeconds(TIME_DROP)
                        .UNSTABLE_addTemporalMarkerOffset(TIME_DROP_TO_FLIP, () -> scorer.passthrough.trigger())
                        .setReversed(false)
                        .splineTo(sideTurnPos, facingLeft)
                        .splineTo(parkingZone1.vec(), parkingZone1.getHeading())
                        .build();

        TrajectorySequence parkInZone2 = drivetrain.trajectorySequenceBuilder(scoringTrajectory.end())
                .setReversed(true)
                .splineTo(sideTurnPos, stackOffset + (isRight ? facingLeft : facingRight))
                .splineToSplineHeading(medScoringPos, medScoringPos.getHeading() - facingLeft)
                .UNSTABLE_addTemporalMarkerOffset(-TIME_LIFT, () -> scorer.setTargetLiftPos(pole))
                .waitSeconds(TIME_PRE_DROP)
                .addTemporalMarker(() -> scorer.dropCone())
                .waitSeconds(TIME_DROP)
                .UNSTABLE_addTemporalMarkerOffset(TIME_DROP_TO_FLIP, () -> scorer.passthrough.trigger())
                .lineToSplineHeading(parkingZone2)
                .build();

        TrajectorySequence parkInZone3 = isRight ?
                drivetrain.trajectorySequenceBuilder(scoringTrajectory.end())
                        .setReversed(true)
                        .splineTo(sideTurnPos, stackOffset + facingLeft)
                        .splineToSplineHeading(medScoringPos, medScoringPos.getHeading() - facingLeft)
                        .UNSTABLE_addTemporalMarkerOffset(-TIME_LIFT, () -> scorer.setTargetLiftPos(pole))
                        .waitSeconds(TIME_PRE_DROP)
                        .addTemporalMarker(() -> scorer.dropCone())
                        .waitSeconds(TIME_DROP)
                        .UNSTABLE_addTemporalMarkerOffset(TIME_DROP_TO_FLIP, () -> scorer.passthrough.trigger())
                        .setReversed(false)
                        .splineTo(sideTurnPos, facingRight)
                        .splineTo(parkingZone3.vec(), parkingZone3.getHeading())
                        .build() :
                drivetrain.trajectorySequenceBuilder(scoringTrajectory.end())
                        .setReversed(true)
                        .splineTo(centerTurnPos, facingRight)
                        .splineToSplineHeading(centerTallScoringPos, medScoringPos.getHeading() - facingLeft)
                        .UNSTABLE_addTemporalMarkerOffset(-TIME_LIFT_TALL, () -> scorer.setTargetLiftPos(Lift.Position.TALL))
                        .waitSeconds(TIME_PRE_DROP)
                        .addTemporalMarker(() -> scorer.dropCone())
                        .waitSeconds(TIME_DROP)
                        .UNSTABLE_addTemporalMarkerOffset(TIME_DROP_TO_FLIP, () -> scorer.passthrough.trigger())
                        .lineToSplineHeading(parkingZone3)
                        .build();

        drivetrain.followTrajectorySequenceAsync(scoringTrajectory);
        HeadingHolder.setHeading(isRight ? 270.0 : 90.0);

        telemetry.setMsTransmissionInterval(50);
        myTelemetry = new MultipleTelemetry(telemetry);

        this.camera = new AprilTagCamera(
                hardwareMap,
                myTelemetry,
                new int[]{1, 2, 3},
                OpenCvCameraRotation.SIDEWAYS_RIGHT
        );

        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */

        scorer.passthrough.claw.setActivated(true);
        while (!isStarted() && !isStopRequested()) {
            camera.initLoop();
            scorer.passthrough.claw.run();
        }

        //START IS HERE//

        camera.printOutput();
        ElapsedTime autonomousTimer = new ElapsedTime();
        boolean hasParked = false;

        while (opModeIsActive()) {

            for (LynxModule hub : hubs) hub.clearBulkCache();

            if (!hasParked && !drivetrain.isBusy() && (autonomousTimer.seconds() >= 3)) {
                drivetrain.followTrajectorySequenceAsync(
                        camera.detectedTag == null ? parkInZone2 :
                                camera.detectedTag.id == 1 ? parkInZone1 :
                                        camera.detectedTag.id == 3 ? parkInZone3 :
                                                parkInZone2
                );

                hasParked = true;
            }

            scorer.lift.readPosition();
            drivetrain.update();
            scorer.lift.runToPosition();
            scorer.run(0, 0);

            // everything below is telemetry
            scorer.printTelemetry(myTelemetry);
            myTelemetry.update();
        }
    }
}
