package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


public class MeepMeep {
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

    public static void main(String[] args) {
        com.noahbres.meepmeep.MeepMeep meepMeep = new com.noahbres.meepmeep.MeepMeep(700);

        boolean isRight = true;
        double side = isRight ? 1 : -1;

        double X_START = side * ZONE_2_X;

        double facingRight = Math.toRadians(0);
        double facingForward = Math.toRadians(90);
        double facingLeft = Math.toRadians(180);
        double stack = side * Math.toRadians(STACK_ANGLE_OFFSET);

        Vector2d stackPos = new Vector2d(side * STACK_X, STACK_Y);
        Vector2d sideTurnPos = new Vector2d(side * X_TURN_POS, Y_MAIN_PATH);
        Pose2d tallScoringPos = new Pose2d(side * TALL_X, TALL_Y, Math.toRadians(isRight ? TALL_ANGLE : 180 - TALL_ANGLE));
        Pose2d medScoringPos = new Pose2d(side * MED_X, MED_Y, Math.toRadians(isRight ? MED_ANGLE : 180 - MED_ANGLE));
        Vector2d centerTurnPos = new Vector2d(sideTurnPos.getX() - side * ONE_TILE, sideTurnPos.getY());
        Pose2d centerTallScoringPos = new Pose2d(medScoringPos.getX() - side * ONE_TILE, medScoringPos.getY(), medScoringPos.getHeading());

        Pose2d parkingZone1 = new Pose2d(side * (isRight ? ZONE_1_X : ZONE_3_X), Y_MAIN_PATH, isRight ? facingRight : facingLeft);
        Pose2d parkingZone2 = new Pose2d(X_START, Y_MAIN_PATH, parkingZone1.getHeading());
        Pose2d parkingZone3 = new Pose2d(side * (isRight ? ZONE_3_X : ZONE_1_X), Y_MAIN_PATH, parkingZone1.getHeading());

        Pose2d startPose = new Pose2d(X_START, Y_START, facingForward);

        double TIME_LIFT = TIME_LIFT_MEDIUM;
        Pose2d scoringPos;

        scoringPos = medScoringPos;

        RoadRunnerBotEntity bot1 = new DefaultBotBuilder(meepMeep)
                .setDimensions(17, 16)
                .setStartPose(startPose)
                .setConstraints(50, 50, Math.toRadians(158.4000248984491), Math.toRadians(190), 16.47)
                .followTrajectorySequence(drivetrain ->
                        drivetrain.trajectorySequenceBuilder(startPose)
                                .addTemporalMarker(() -> {
                                })
                                .lineTo(parkingZone2.vec())
                                .setReversed(true)
                                .lineToSplineHeading(scoringPos)
                                .UNSTABLE_addTemporalMarkerOffset(-TIME_FIRST_FLIP, () -> {
                                })
                                .UNSTABLE_addTemporalMarkerOffset(-TIME_LIFT, () -> {
                                })
                                .build()
                );

        meepMeep.setBackground(com.noahbres.meepmeep.MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(false)
                .setBackgroundAlpha(1.0f)
                .addEntity(bot1)
                .start();
    }
}