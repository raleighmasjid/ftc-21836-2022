package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.roadrunner.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.roadrunner.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.roadrunner.DriveConstants.TRACK_WIDTH;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.AprilTagCamera;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.ScoringSystem;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.List;

@Config
public abstract class BaseAuton extends LinearOpMode {

    MultipleTelemetry myTelemetry;
    SampleMecanumDrive drivetrain;
    ScoringSystem scorer;
    List<LynxModule> hubs;
    AprilTagCamera camera;

    public static double
            END_HEADING = 0.0,
            END_HEADING_OFFSET = -90.0,
            END_HEADING_MULTIPLIER = 1,
            START_TURN_Y = -52,
            MED_ANGLE = 40,
            MED_X = 33,
            MED_Y = -18,
            TALL_ANGLE = -35.0,
            TALL_X = 31.0,
            TALL_Y = -7.5,
            STACK_X = 60,
            STACK_Y = -13,
            ONE_TILE = 24.0,
            ZONE_INNER_X = 12.5,
            ZONE_2_X = 35.0,
            ZONE_OUTER_X = 57.0,
            TURN_POS_X = 46.0,
            TURN_ANGLE_OFFSET_MED = 0,
            TURN_ANGLE_OFFSET_TALL = -2.0,
            Y_START = -62.5,
            Y_MAIN_PATH = -13,
            TIME_PRE_GRAB = 1.0,
            TIME_GRAB = 1.0,
            TIME_PRE_DROP = 1.0,
            TIME_DROP = 0,
            TIME_DROP_TO_FLIP = 0,
            TIME_FIRST_FLIP = 3,
            TIME_LIFT_MEDIUM = 1,
            TIME_LIFT_TALL = 1.2,
            TIME_FLIP = 3,
            STACK_VELO = 10,
            STACK_ACCEL = MAX_ACCEL,
            SCORING_VELO = 10,
            SCORING_ACCEL = MAX_ACCEL;

    public static final double
            RIGHT = Math.toRadians(0),
            FORWARD = Math.toRadians(90),
            LEFT = Math.toRadians(180);

    private final TrajectoryVelocityConstraint stackVeloCap = SampleMecanumDrive.getVelocityConstraint(STACK_VELO, MAX_ANG_VEL, TRACK_WIDTH);
    private final TrajectoryAccelerationConstraint stackAccelCap = SampleMecanumDrive.getAccelerationConstraint(STACK_ACCEL);
    private final TrajectoryVelocityConstraint scoringVeloCap = SampleMecanumDrive.getVelocityConstraint(SCORING_VELO, MAX_ANG_VEL, TRACK_WIDTH);
    private final TrajectoryAccelerationConstraint scoringAccelCap = SampleMecanumDrive.getAccelerationConstraint(SCORING_ACCEL);

    public void runOpMode(boolean tallPole, boolean isRight) throws InterruptedException {
        drivetrain = new SampleMecanumDrive(hardwareMap);
        scorer = new ScoringSystem(hardwareMap);

        double side = isRight ? 1 : -1;

        double X_START = side * ZONE_2_X;

        Vector2d stackPos = new Vector2d(side * STACK_X, STACK_Y);
        Vector2d sideTurnPos = new Vector2d(side * TURN_POS_X, Y_MAIN_PATH);
        Vector2d centerTurnPos = new Vector2d(sideTurnPos.getX() - side * ONE_TILE, sideTurnPos.getY());
        Pose2d tallScoringPos = new Pose2d(side * TALL_X, TALL_Y, Math.toRadians(isRight ? TALL_ANGLE : 180 - TALL_ANGLE));
        Pose2d medScoringPos = new Pose2d(side * MED_X, MED_Y, Math.toRadians(isRight ? MED_ANGLE : 180 - MED_ANGLE));
        Pose2d centerTallScoringPos = new Pose2d(medScoringPos.getX() - side * ONE_TILE, medScoringPos.getY(), medScoringPos.getHeading());

        Pose2d parkingZoneInner = new Pose2d(side * ZONE_INNER_X, Y_MAIN_PATH, isRight ? RIGHT : LEFT);
        Pose2d parkingZoneMiddle = new Pose2d(X_START, parkingZoneInner.getY(), parkingZoneInner.getHeading());
        Pose2d parkingZoneOuter = new Pose2d(side * ZONE_OUTER_X, parkingZoneInner.getY(), parkingZoneInner.getHeading());

        Pose2d startPose = new Pose2d(X_START, Y_START, FORWARD);

        Lift.Position pole = tallPole ? Lift.Position.TALL : Lift.Position.MED;
        double TIME_LIFT = tallPole ? TIME_LIFT_TALL : TIME_LIFT_MEDIUM;
        Pose2d scoringPos = tallPole ? tallScoringPos : medScoringPos;
        double TURN_ANGLE_OFFSET = side * Math.toRadians(tallPole ? TURN_ANGLE_OFFSET_TALL : TURN_ANGLE_OFFSET_MED);

        drivetrain.setPoseEstimate(startPose);

        TrajectorySequence scoringTrajectory = drivetrain.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(() -> scorer.liftClaw())
                .lineToSplineHeading(new Pose2d(X_START, START_TURN_Y, RIGHT))
                .lineTo(parkingZoneMiddle.vec())
                .lineToSplineHeading(scoringPos, scoringVeloCap, scoringAccelCap)
                .UNSTABLE_addTemporalMarkerOffset(-TIME_FIRST_FLIP, () -> scorer.passthrough.trigger())
                .UNSTABLE_addTemporalMarkerOffset(-TIME_LIFT, () -> scorer.setTargetLiftPos(pole))
                .waitSeconds(TIME_PRE_DROP)
                .addTemporalMarker(() -> scorer.dropCone(Lift.Position.FIVE))
                .waitSeconds(TIME_DROP)
                .UNSTABLE_addTemporalMarkerOffset(TIME_DROP_TO_FLIP, () -> scorer.passthrough.trigger())
                .splineTo(sideTurnPos, TURN_ANGLE_OFFSET + (isRight ? RIGHT : LEFT))
                .splineTo(stackPos, isRight ? RIGHT : LEFT, stackVeloCap, stackAccelCap)
                // loop below
                .setValues(scorer, sideTurnPos, stackPos, isRight, pole, TIME_LIFT, scoringPos, TURN_ANGLE_OFFSET)
                .addCycle(Lift.Position.FOUR)
                // common parking:
                .waitSeconds(TIME_PRE_GRAB)
                .addTemporalMarker(() -> scorer.grabCone())
                .waitSeconds(TIME_GRAB)
                .build();

        TrajectorySequence parkInner = drivetrain.trajectorySequenceBuilder(scoringTrajectory.end())
                .setReversed(true)
                .splineTo(centerTurnPos, isRight ? LEFT : RIGHT)
                .splineToSplineHeading(centerTallScoringPos, medScoringPos.getHeading() - LEFT, scoringVeloCap, scoringAccelCap)
                .UNSTABLE_addTemporalMarkerOffset(-TIME_LIFT_TALL, () -> scorer.setTargetLiftPos(Lift.Position.TALL))
                .UNSTABLE_addTemporalMarkerOffset(-TIME_FLIP, () -> scorer.passthrough.trigger())
                .waitSeconds(TIME_PRE_DROP)
                .addTemporalMarker(() -> scorer.dropCone())
                .waitSeconds(TIME_DROP)
                .UNSTABLE_addTemporalMarkerOffset(TIME_DROP_TO_FLIP, () -> scorer.passthrough.trigger())
                .lineToSplineHeading(parkingZoneInner)
                .build();

        TrajectorySequence parkOuter = drivetrain.trajectorySequenceBuilder(scoringTrajectory.end())
                .setReversed(true)
                .splineTo(sideTurnPos, TURN_ANGLE_OFFSET + (isRight ? LEFT : RIGHT))
                .splineToSplineHeading(medScoringPos, medScoringPos.getHeading() - LEFT, scoringVeloCap, scoringAccelCap)
                .UNSTABLE_addTemporalMarkerOffset(-TIME_LIFT, () -> scorer.setTargetLiftPos(pole))
                .UNSTABLE_addTemporalMarkerOffset(-TIME_FLIP, () -> scorer.passthrough.trigger())
                .waitSeconds(TIME_PRE_DROP)
                .addTemporalMarker(() -> scorer.dropCone())
                .waitSeconds(TIME_DROP)
                .UNSTABLE_addTemporalMarkerOffset(TIME_DROP_TO_FLIP, () -> scorer.passthrough.trigger())
                .setReversed(false)
                .splineTo(sideTurnPos, (isRight ? RIGHT : LEFT))
                .splineTo(parkingZoneOuter.vec(), parkingZoneOuter.getHeading())
                .build();

        TrajectorySequence parkInZone1 = isRight ? parkInner : parkOuter;

        TrajectorySequence parkInZone2 = drivetrain.trajectorySequenceBuilder(scoringTrajectory.end())
                .setReversed(true)
                .splineTo(sideTurnPos, TURN_ANGLE_OFFSET + (isRight ? LEFT : RIGHT))
                .splineToSplineHeading(medScoringPos, medScoringPos.getHeading() - LEFT, scoringVeloCap, scoringAccelCap)
                .UNSTABLE_addTemporalMarkerOffset(-TIME_LIFT, () -> scorer.setTargetLiftPos(pole))
                .UNSTABLE_addTemporalMarkerOffset(-TIME_FLIP, () -> scorer.passthrough.trigger())
                .waitSeconds(TIME_PRE_DROP)
                .addTemporalMarker(() -> scorer.dropCone())
                .waitSeconds(TIME_DROP)
                .UNSTABLE_addTemporalMarkerOffset(TIME_DROP_TO_FLIP, () -> scorer.passthrough.trigger())
                .lineToSplineHeading(parkingZoneMiddle)
                .build();

        TrajectorySequence parkInZone3 = isRight ? parkOuter : parkInner;

        drivetrain.followTrajectorySequenceAsync(scoringTrajectory);

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
            scorer.passthrough.run();
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
            scorer.passthrough.run();
            scorer.run(0, 0);

            END_HEADING = (drivetrain.getPoseEstimate().getHeading() + END_HEADING_OFFSET) * END_HEADING_MULTIPLIER;

            // everything below is telemetry
            scorer.printTelemetry(myTelemetry);
            myTelemetry.update();
        }
    }
}
