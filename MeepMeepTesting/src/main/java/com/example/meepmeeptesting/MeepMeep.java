package com.example.meepmeeptesting;

import static com.example.meepmeeptesting.DriveConstants.MAX_ANG_VEL;
import static com.example.meepmeeptesting.DriveConstants.MAX_VEL;
import static com.example.meepmeeptesting.DriveConstants.TRACK_WIDTH;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.SampleMecanumDrive;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


public class MeepMeep {

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
            PARKING_INNER_X = 12.5,
            CENTER_X = 35.0,
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
            SCORING_SHIFT = 0.1,
            SCORING_VELO = MAX_VEL,
            SCORING_ACCEL = 20;

    public static final double
            RIGHT = Math.toRadians(0),
            FORWARD = Math.toRadians(90),
            LEFT = Math.toRadians(180);

    private static final TrajectoryVelocityConstraint stackVeloCap = SampleMecanumDrive.getVelocityConstraint(STACK_VELO, MAX_ANG_VEL, TRACK_WIDTH);
    private static final TrajectoryAccelerationConstraint stackAccelCap = SampleMecanumDrive.getAccelerationConstraint(STACK_ACCEL);
    private static final TrajectoryVelocityConstraint scoringVeloCap = SampleMecanumDrive.getVelocityConstraint(SCORING_VELO, MAX_ANG_VEL, TRACK_WIDTH);
    private static final TrajectoryAccelerationConstraint scoringAccelCap = SampleMecanumDrive.getAccelerationConstraint(SCORING_ACCEL);

    public static void main(String[] args) {
        com.noahbres.meepmeep.MeepMeep meepMeep = new com.noahbres.meepmeep.MeepMeep(700);

        boolean isRight = true;
        boolean tallPole = false;

        double side = isRight ? 1 : -1;
        double CENTER_X = side * MeepMeep.CENTER_X;

        Vector2d stackPos = new Vector2d(side * STACK_X, STACK_Y);
        Vector2d sideTurnPos = new Vector2d(side * TURN_POS_X, Y_MAIN_PATH);
        Vector2d centerTurnPos = new Vector2d(sideTurnPos.getX() - side * ONE_TILE, sideTurnPos.getY());
        Pose2d tallScoringPos = new Pose2d(side * TALL_X, TALL_Y, Math.toRadians(isRight ? TALL_ANGLE : 180 - TALL_ANGLE));
        Pose2d medScoringPos = new Pose2d(side * MED_X, MED_Y, Math.toRadians(isRight ? MED_ANGLE : 180 - MED_ANGLE));
        Pose2d centerTallScoringPos = new Pose2d(medScoringPos.getX() - side * ONE_TILE, medScoringPos.getY(), medScoringPos.getHeading());

        Pose2d centerParkingZone = new Pose2d(CENTER_X, Y_MAIN_PATH, isRight ? RIGHT : LEFT);
        Pose2d startPose = new Pose2d(CENTER_X, Y_START, FORWARD);

//        Lift.Position pole = tallPole ? Lift.Position.TALL : Lift.Position.MED;
        double TIME_LIFT = tallPole ? TIME_LIFT_TALL : TIME_LIFT_MEDIUM;
        Pose2d scoringPos = tallPole ? tallScoringPos : medScoringPos;
        double TURN_ANGLE_OFFSET = side * Math.toRadians(tallPole ? TURN_ANGLE_OFFSET_TALL : TURN_ANGLE_OFFSET_MED);
        double FIRST_Y_OFFSET = tallPole ? 0 : MED_Y_FIRST_OFFSET;

        double scoringShifts = 0;
        double stackShifts = 0;

        RoadRunnerBotEntity bot1 = new DefaultBotBuilder(meepMeep)
                .setDimensions(17, 16)
                .setStartPose(startPose)
                .setConstraints(50, 50, Math.toRadians(158.4000248984491), Math.toRadians(190), 16.47)
                .followTrajectorySequence(drivetrain ->
                                drivetrain.trajectorySequenceBuilder(startPose)
//                                        .addTemporalMarker(() -> scorer.liftClaw())
                                        .lineToSplineHeading(new Pose2d(CENTER_X, START_TURN_Y, isRight ? RIGHT : LEFT))
                                        .lineTo(centerParkingZone.vec())
                                        .lineToSplineHeading(new Pose2d(scoringPos.getX(), scoringPos.getY() + FIRST_Y_OFFSET, scoringPos.getHeading()), scoringVeloCap, scoringAccelCap)
//                                        .UNSTABLE_addTemporalMarkerOffset(-TIME_FIRST_FLIP, () -> scorer.passthrough.trigger())
//                                        .UNSTABLE_addTemporalMarkerOffset(-TIME_LIFT, () -> scorer.setTargetLiftPos(pole))
                                        .waitSeconds(TIME_PRE_DROP)
//                                        .addTemporalMarker(() -> scorer.dropCone(Lift.Position.FIVE))
                                        .waitSeconds(TIME_DROP)
//                                        .UNSTABLE_addTemporalMarkerOffset(TIME_DROP_TO_FLIP, () -> scorer.passthrough.trigger())
                                        .splineTo(sideTurnPos, TURN_ANGLE_OFFSET + (isRight ? RIGHT : LEFT))
                                        .splineTo(stackPos, isRight ? RIGHT : LEFT, stackVeloCap, stackAccelCap)
                                        // loop below
                                        .waitSeconds(TIME_PRE_GRAB)
//                                        .addTemporalMarker(() -> scorer.grabCone())
                                        .waitSeconds(TIME_GRAB)
                                        .setReversed(true)
                                        .splineTo(sideTurnPos, TURN_ANGLE_OFFSET + (isRight ? LEFT : RIGHT))
                                        .splineToSplineHeading(new Pose2d(scoringPos.getX() + (scoringShifts * side * SCORING_SHIFT), scoringPos.getY(), scoringPos.getHeading()), scoringPos.getHeading() - LEFT, scoringVeloCap, scoringAccelCap)
//                                        .UNSTABLE_addTemporalMarkerOffset(-TIME_LIFT, () -> scorer.setTargetLiftPos(pole))
//                                        .UNSTABLE_addTemporalMarkerOffset(-TIME_FLIP, () -> scorer.passthrough.trigger())
                                        .waitSeconds(TIME_PRE_DROP)
//                                        .addTemporalMarker(() -> scorer.dropCone(endLiftPosition))
                                        .waitSeconds(TIME_DROP)
//                                        .UNSTABLE_addTemporalMarkerOffset(TIME_DROP_TO_FLIP, () -> scorer.passthrough.trigger())
                                        .setReversed(false)
                                        .splineTo(sideTurnPos, TURN_ANGLE_OFFSET + (isRight ? RIGHT : LEFT))
                                        .splineTo(new Vector2d(stackPos.getX() + (stackShifts * side * STACK_SHIFT), stackPos.getY()), isRight ? RIGHT : LEFT, stackVeloCap, stackAccelCap)
                                        .build()
                );

        meepMeep.setBackground(com.noahbres.meepmeep.MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(false)
                .setBackgroundAlpha(1.0f)
                .addEntity(bot1)
                .start();
    }
}