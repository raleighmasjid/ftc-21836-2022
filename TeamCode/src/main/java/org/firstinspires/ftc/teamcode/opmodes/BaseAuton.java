package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.roadrunner.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.roadrunner.DriveConstants.MAX_VEL;
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
            MED_ANGLE = 29.25,
            MED_X = 32,
            MED_Y = -18.5,
            MED_Y_FIRST_OFFSET = 2.5,
            TALL_ANGLE = -35.0,
            TALL_X = 31.0,
            TALL_Y = -7.5,
            STACK_X = 59,
            STACK_Y = -12,
            ONE_TILE = 23.3,
            ZONE_INNER_X = 12.5,
            ZONE_CENTER_X = 35.0,
            ZONE_OUTER_X = 57.0,
            TURN_POS_X = 46.0,
            TURN_ANGLE_OFFSET_MED = 10,
            TURN_ANGLE_OFFSET_TALL = -2.0,
            Y_START = -62.5,
            Y_MAIN_PATH = -13,
            TIME_PRE_GRAB = 0,
            TIME_GRAB = 0.5,
            TIME_PRE_DROP = 0,
            TIME_DROP = 0,
            TIME_DROP_TO_FLIP = 0,
            TIME_FIRST_FLIP = 1.5,
            TIME_LIFT_MEDIUM = 1.3,
            TIME_LIFT_TALL = 1.7,
            TIME_FLIP = 1.75,
            STACK_SHIFT = 0.45,
            STACK_VELO = MAX_VEL,
            STACK_ACCEL = 40,
            SCORING_SHIFT = -0.1,
            SCORING_VELO = MAX_VEL,
            SCORING_ACCEL = 20;

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

        double X_START = side * ZONE_CENTER_X;
        double STACK_SHIFT = side * BaseAuton.STACK_SHIFT;
        double SCORING_SHIFT = side * BaseAuton.SCORING_SHIFT;

        Vector2d stackPos = new Vector2d(side * STACK_X, STACK_Y);
        Vector2d sideTurnPos = new Vector2d(side * TURN_POS_X, Y_MAIN_PATH);
        Vector2d centerTurnPos = new Vector2d(sideTurnPos.getX() - side * ONE_TILE, sideTurnPos.getY());
        Pose2d tallScoringPos = new Pose2d(side * TALL_X, TALL_Y, Math.toRadians(isRight ? TALL_ANGLE : 180 - TALL_ANGLE));
        Pose2d medScoringPos = new Pose2d(side * MED_X, MED_Y, Math.toRadians(isRight ? MED_ANGLE : 180 - MED_ANGLE));
        Pose2d centerTallScoringPos = new Pose2d(medScoringPos.getX() - side * ONE_TILE, medScoringPos.getY(), medScoringPos.getHeading());

        Pose2d innerParkingZone = new Pose2d(side * ZONE_INNER_X, Y_MAIN_PATH, isRight ? RIGHT : LEFT);
        Pose2d centerParkingZone = new Pose2d(X_START, Y_MAIN_PATH, isRight ? RIGHT : LEFT);
        Pose2d outerParkingZone = new Pose2d(side * ZONE_OUTER_X, Y_MAIN_PATH, isRight ? RIGHT : LEFT);

        Pose2d startPose = new Pose2d(X_START, Y_START, FORWARD);

        Lift.Position pole = tallPole ? Lift.Position.TALL : Lift.Position.MED;
        double TIME_LIFT = tallPole ? TIME_LIFT_TALL : TIME_LIFT_MEDIUM;
        Pose2d scoringPos = tallPole ? tallScoringPos : medScoringPos;
        double TURN_ANGLE_OFFSET = side * Math.toRadians(tallPole ? TURN_ANGLE_OFFSET_TALL : TURN_ANGLE_OFFSET_MED);
        double FIRST_Y_OFFSET = tallPole ? 0 : MED_Y_FIRST_OFFSET;

        drivetrain.setPoseEstimate(startPose);

        TrajectorySequence scoringTrajectory = drivetrain.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(() -> scorer.liftClaw())
                .lineToSplineHeading(new Pose2d(X_START, START_TURN_Y, isRight ? RIGHT : LEFT))
                .lineTo(centerParkingZone.vec())
                .lineToSplineHeading(new Pose2d(scoringPos.getX(), scoringPos.getY() + FIRST_Y_OFFSET, scoringPos.getHeading()), scoringVeloCap, scoringAccelCap)
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
                .addCycle(Lift.Position.FOUR, 1, 1)
                .addCycle(Lift.Position.THREE, 2, 2)
                .addCycle(Lift.Position.TWO,3, 3)
                .addCycle(Lift.Position.FLOOR,4, 4)
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
                .lineToSplineHeading(innerParkingZone)
                .build();

        TrajectorySequence parkOuter = drivetrain.trajectorySequenceBuilder(scoringTrajectory.end())
                .setReversed(true)
                .splineTo(sideTurnPos, TURN_ANGLE_OFFSET + (isRight ? LEFT : RIGHT))
                .splineToSplineHeading(new Pose2d(scoringPos.getX() - (5 * SCORING_SHIFT), scoringPos.getY(), scoringPos.getHeading()), scoringPos.getHeading() - LEFT, scoringVeloCap, scoringAccelCap)
                .UNSTABLE_addTemporalMarkerOffset(-TIME_LIFT, () -> scorer.setTargetLiftPos(pole))
                .UNSTABLE_addTemporalMarkerOffset(-TIME_FLIP, () -> scorer.passthrough.trigger())
                .waitSeconds(TIME_PRE_DROP)
                .addTemporalMarker(() -> scorer.dropCone())
                .waitSeconds(TIME_DROP)
                .UNSTABLE_addTemporalMarkerOffset(TIME_DROP_TO_FLIP, () -> scorer.passthrough.trigger())
                .setReversed(false)
                .splineTo(sideTurnPos, (isRight ? RIGHT : LEFT))
                .splineTo(outerParkingZone.vec(), outerParkingZone.getHeading())
                .build();

        TrajectorySequence parkInMiddle = drivetrain.trajectorySequenceBuilder(scoringTrajectory.end())
                .setReversed(true)
                .splineTo(sideTurnPos, TURN_ANGLE_OFFSET + (isRight ? LEFT : RIGHT))
                .splineToSplineHeading(new Pose2d(scoringPos.getX() - (5 * SCORING_SHIFT), scoringPos.getY(), scoringPos.getHeading()), scoringPos.getHeading() - LEFT, scoringVeloCap, scoringAccelCap)
                .UNSTABLE_addTemporalMarkerOffset(-TIME_LIFT, () -> scorer.setTargetLiftPos(pole))
                .UNSTABLE_addTemporalMarkerOffset(-TIME_FLIP, () -> scorer.passthrough.trigger())
                .waitSeconds(TIME_PRE_DROP)
                .addTemporalMarker(() -> scorer.dropCone())
                .waitSeconds(TIME_DROP)
                .UNSTABLE_addTemporalMarkerOffset(TIME_DROP_TO_FLIP, () -> scorer.passthrough.trigger())
                .lineToSplineHeading(centerParkingZone)
                .build();

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

            if (!drivetrain.isBusy() && !hasParked && autonomousTimer.seconds() >= 3) {
                drivetrain.followTrajectorySequenceAsync(
                        camera.detectedTag == null ? parkInMiddle :
                                camera.detectedTag.id == 1 ? isRight ? parkInner : parkOuter :
                                        camera.detectedTag.id == 3 ? isRight ? parkOuter : parkInner :
                                                parkInMiddle
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
