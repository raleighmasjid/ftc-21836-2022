package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


public class MeepMeep {
    public static void main(String[] args) {
        com.noahbres.meepmeep.MeepMeep meepMeep = new com.noahbres.meepmeep.MeepMeep(700);

        boolean isRight = true;
        double side = isRight ? 1 : -1;

        double centerPathX = side * AutonConfig.ZONE_2_X;

        double facingRight = Math.toRadians(0);
        double facingForward = Math.toRadians(90);
        double facingLeft = Math.toRadians(180);

        Vector2d stackPos = new Vector2d(side * AutonConfig.STACK_X, AutonConfig.MAIN_Y);
        Vector2d sideTurnPos = new Vector2d(side * AutonConfig.TURN_POS_X, AutonConfig.MAIN_Y);
        Pose2d tallScoringPos = new Pose2d(side * AutonConfig.TALL_X, AutonConfig.TALL_Y, Math.toRadians(isRight ? AutonConfig.TALL_ANGLE : 180 - AutonConfig.TALL_ANGLE));
        Pose2d medScoringPos = new Pose2d(side * AutonConfig.MED_X, AutonConfig.MED_Y, Math.toRadians(isRight ? AutonConfig.MED_ANGLE : 180 - AutonConfig.MED_ANGLE));
        Vector2d centerTurnPos = new Vector2d(sideTurnPos.getX() - side * AutonConfig.ONE_TILE, sideTurnPos.getY());
        Pose2d centerTallScoringPos = new Pose2d(medScoringPos.getX() - side * AutonConfig.ONE_TILE, medScoringPos.getY(), medScoringPos.getHeading());

        Pose2d parkingZone1 = new Pose2d(side * (isRight ? AutonConfig.ZONE_1_X : AutonConfig.ZONE_3_X), AutonConfig.MAIN_Y, !isRight ? facingRight : facingLeft);
        Pose2d parkingZone2 = new Pose2d(centerPathX, AutonConfig.MAIN_Y, parkingZone1.getHeading());
        Pose2d parkingZone3 = new Pose2d(side * (isRight ? AutonConfig.ZONE_3_X : AutonConfig.ZONE_1_X), AutonConfig.MAIN_Y, parkingZone1.getHeading());

        Pose2d startPose = new Pose2d(centerPathX, AutonConfig.STARTING_Y, facingForward);

        RoadRunnerBotEntity bot1 = new DefaultBotBuilder(meepMeep)
                .setDimensions(17, 16)
                .setStartPose(startPose)
                .setConstraints(58, 55, Math.toRadians(140), Math.toRadians(190), 14.25)
                .followTrajectorySequence(drivetrain ->
                                drivetrain.trajectorySequenceBuilder(startPose)
                                        .setReversed(false)
//                                .addTemporalMarker(() -> scorer.liftClaw())
                                        .splineTo(new Vector2d(centerPathX, -25), facingForward)
                                        .splineToSplineHeading(tallScoringPos, tallScoringPos.getHeading())
//                                .UNSTABLE_addTemporalMarkerOffset(-RobotConfig.TIME_LIFT_TALL, () -> scorer.setTargetLiftPos(PowerplayScorer.LiftPos.TALL))
//                                .addTemporalMarker(() -> scorer.dropCone(PowerplayScorer.LiftPos.FIVE))
                                        .waitSeconds(AutonConfig.TIME_DROP)
//                                .UNSTABLE_addTemporalMarkerOffset(AutonConfig.TIME_POST_DROP, () -> scorer.triggerPassThru())
                                        .setReversed(true)
                                        .setTangent(tallScoringPos.getHeading() + facingLeft)
                                        .splineTo(sideTurnPos, isRight ? facingRight : facingLeft)
                                        .splineTo(stackPos, isRight ? facingRight : facingLeft)
//                                .addTemporalMarker(() -> scorer.grabCone())
                                        .waitSeconds(AutonConfig.TIME_GRAB)
//                                .UNSTABLE_addTemporalMarkerOffset(AutonConfig.TIME_POST_GRAB, () -> scorer.triggerPassThru())
                                        .setReversed(false)
                                        .splineTo(sideTurnPos, isRight ? facingLeft : facingRight)
                                        .splineToSplineHeading(tallScoringPos, tallScoringPos.getHeading())
//                                .UNSTABLE_addTemporalMarkerOffset(-RobotConfig.TIME_LIFT_TALL, () -> scorer.setTargetLiftPos(PowerplayScorer.LiftPos.TALL))
//                                .addTemporalMarker(() -> scorer.dropCone(PowerplayScorer.LiftPos.FOUR))
                                        .waitSeconds(AutonConfig.TIME_DROP)
//                                .UNSTABLE_addTemporalMarkerOffset(AutonConfig.TIME_POST_DROP, () -> scorer.triggerPassThru())
                                        .setReversed(true)
                                        .setTangent(tallScoringPos.getHeading() + facingLeft)
                                        .splineTo(sideTurnPos, isRight ? facingRight : facingLeft)
                                        .splineTo(stackPos, isRight ? facingRight : facingLeft)
//                                .addTemporalMarker(() -> scorer.grabCone())
                                        .waitSeconds(AutonConfig.TIME_GRAB)
//                                .UNSTABLE_addTemporalMarkerOffset(AutonConfig.TIME_POST_GRAB, () -> scorer.triggerPassThru())
                                        .setReversed(false)
                                        .splineTo(centerTurnPos, isRight ? facingLeft : facingRight)
                                        .splineTo(centerTallScoringPos.vec(), centerTallScoringPos.getHeading() + facingLeft)
//                                        .UNSTABLE_addTemporalMarkerOffset(-RobotConfig.TIME_LIFT_TALL, () -> scorer.setTargetLiftPos(PowerplayScorer.LiftPos.TALL))
//                                        .addTemporalMarker(() -> scorer.dropCone())
                                        .waitSeconds(AutonConfig.TIME_DROP)
//                                        .UNSTABLE_addTemporalMarkerOffset(AutonConfig.TIME_POST_DROP, () -> scorer.triggerPassThru())
                                        .lineToSplineHeading(parkingZone1)
                                        .build()
                );

        meepMeep.setBackground(com.noahbres.meepmeep.MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(false)
                .setBackgroundAlpha(1.0f)
                .addEntity(bot1)
                .start();
    }
}