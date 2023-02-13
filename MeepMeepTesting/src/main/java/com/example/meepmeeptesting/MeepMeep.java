package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


public class MeepMeep {
    public static void main(String[] args) {
        com.noahbres.meepmeep.MeepMeep meepMeep = new com.noahbres.meepmeep.MeepMeep(700);

        Vector2d stackPos = new Vector2d(59, -12.5);
        Vector2d turnPos = new Vector2d(46, -13);
        Vector2d medScoringPos = new Vector2d(31, -17.5);

        double centerPathX = 35;
        double firstScoringY = -24;

        Vector2d parkingZone1 = new Vector2d(13, -12.5);
        Vector2d parkingZone2 = new Vector2d(centerPathX, -12.5);
        Vector2d parkingZone3 = new Vector2d(57, -12.5);

        double facingRight = Math.toRadians(0);
        double facingForward = Math.toRadians(90);
        double facingLeft = Math.toRadians(180);
        double scoringAngleRight = Math.toRadians(215);
        double turnPosToStack = Math.toRadians(5);
        double turnPosToPole = facingLeft + turnPosToStack;

        double liftTime = -0.8;
        double stackApproachOffset = -0.2;
        double stackWait = 0;
        double mediumApproachOffset = -0.01;
        double postScoringWait = 0;
        double mediumScoringOffset = 0.1;

        double CLAW_CLOSING_TIME = 0.3;
        double AUTON_START_DELAY = 0; //0.16

        Pose2d startPose = new Pose2d(centerPathX, -62.5, facingForward);

        RoadRunnerBotEntity bot1 = new DefaultBotBuilder(meepMeep)
                .setDimensions(17, 16)
                .setStartPose(startPose)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(58, 55, Math.toRadians(140), Math.toRadians(190), 14.25)
                .followTrajectorySequence(drive->
                        drive.trajectorySequenceBuilder(startPose)
                                .addTemporalMarker(() -> {
//                                    scorer.clawIsOpen = false;
                                })
                                .waitSeconds(CLAW_CLOSING_TIME + AUTON_START_DELAY)
                                .addTemporalMarker(() -> {
//                                    scorer.targetLiftPos = scorer.liftController.getSetPoint() + 150;
                                })
                                .splineToSplineHeading(new Pose2d(centerPathX, -53, facingLeft), facingForward)
                                .splineToSplineHeading(new Pose2d(centerPathX, firstScoringY, facingLeft), facingForward)
                                .splineToSplineHeading(new Pose2d(centerPathX, -15, scoringAngleRight), facingForward)
                                .setTangent(Math.toRadians(210))
                                .UNSTABLE_addTemporalMarkerOffset(liftTime, () -> {
//                                    scorer.setLiftPos(PowerplayScorer.liftHeights.MED);
                                })
                                .splineTo(medScoringPos, scoringAngleRight)
                                .addTemporalMarker(() -> {
//                                    scorer.setLiftPos(PowerplayScorer.liftHeights.FIVE);
//                                    scorer.clawIsOpen = true;
                                })
                                .waitSeconds(postScoringWait)
                                .UNSTABLE_addTemporalMarkerOffset(mediumScoringOffset, () ->{
//                                    scorer.togglePassthrough();
                                })
                                .setReversed(true)
                                .splineTo(turnPos, turnPosToStack)
                                .splineTo(
                                        stackPos,
                                        facingRight
                                )
                                .UNSTABLE_addTemporalMarkerOffset(stackApproachOffset, () -> {
//                                    scorer.liftClaw();
                                })
                                .waitSeconds(stackWait)
                                .addTemporalMarker(() ->{
//                                    scorer.togglePassthrough();
                                })
                                .setReversed(false)
                                .splineTo(turnPos, turnPosToPole)
                                .splineTo(medScoringPos, scoringAngleRight)
                                .UNSTABLE_addTemporalMarkerOffset(liftTime, () -> {
//                                    scorer.setLiftPos(PowerplayScorer.liftHeights.MED);
                                })
                                .addTemporalMarker(() -> {
//                                    scorer.setLiftPos(PowerplayScorer.liftHeights.FOUR);
//                                    scorer.clawIsOpen = true;
                                })
                                .waitSeconds(postScoringWait)
                                .setReversed(true)
                                .UNSTABLE_addTemporalMarkerOffset(mediumScoringOffset, () ->{
//                                    scorer.togglePassthrough();
                                })
                                .splineTo(turnPos, turnPosToStack)
                                .splineTo(
                                        stackPos,
                                        facingRight
                                )
                                .UNSTABLE_addTemporalMarkerOffset(stackApproachOffset, () -> {
//                                    scorer.liftClaw();
                                })
                                .waitSeconds(stackWait)
                                .addTemporalMarker(() ->{
//                                    scorer.togglePassthrough();
                                })
                                .setReversed(false)
                                .splineTo(turnPos, turnPosToPole)
                                .splineTo(medScoringPos, scoringAngleRight)
                                .UNSTABLE_addTemporalMarkerOffset(liftTime, () -> {
//                                    scorer.setLiftPos(PowerplayScorer.liftHeights.MED);
                                })
                                .UNSTABLE_addTemporalMarkerOffset(mediumApproachOffset, () -> {
//                                    scorer.setLiftPos(PowerplayScorer.liftHeights.THREE);
//                                    scorer.clawIsOpen = true;
                                })
                                .waitSeconds(postScoringWait)
                                .setReversed(true)
                                .UNSTABLE_addTemporalMarkerOffset(mediumScoringOffset, () ->{
//                                    scorer.togglePassthrough();
                                })
                                .splineTo(turnPos, turnPosToStack)
                                .splineTo(
                                        stackPos,
                                        facingRight
                                )
                                .UNSTABLE_addTemporalMarkerOffset(stackApproachOffset, () -> {
//                                    scorer.liftClaw();
                                })
                                .waitSeconds(stackWait)
                                .addTemporalMarker(() ->{
//                                    scorer.togglePassthrough();
                                })
                                .setReversed(false)
                                .splineTo(turnPos, turnPosToPole)
                                .splineTo(medScoringPos, scoringAngleRight)
                                .UNSTABLE_addTemporalMarkerOffset(liftTime, () -> {
//                                    scorer.setLiftPos(PowerplayScorer.liftHeights.MED);
                                })
                                .UNSTABLE_addTemporalMarkerOffset(mediumApproachOffset, () -> {
//                                    scorer.setLiftPos(PowerplayScorer.liftHeights.TWO);
//                                    scorer.clawIsOpen = true;
                                })
                                .waitSeconds(postScoringWait)
                                .setReversed(true)
                                .UNSTABLE_addTemporalMarkerOffset(mediumScoringOffset, () ->{
//                                    scorer.togglePassthrough();
                                })
                                .splineTo(turnPos, turnPosToStack)
                                .splineTo(
                                        stackPos,
                                        facingRight
                                )
                                .UNSTABLE_addTemporalMarkerOffset(stackApproachOffset, () -> {
//                                    scorer.liftClaw();
                                })
                                .waitSeconds(stackWait)
                                .addTemporalMarker(() ->{
//                                    scorer.togglePassthrough();
                                })
                                .setReversed(false)
                                .splineTo(turnPos, turnPosToPole)
                                .splineTo(medScoringPos, scoringAngleRight)
                                .UNSTABLE_addTemporalMarkerOffset(liftTime, () -> {
//                                    scorer.setLiftPos(PowerplayScorer.liftHeights.MED);
                                })
                                .UNSTABLE_addTemporalMarkerOffset(mediumApproachOffset, () -> {
//                                    scorer.dropClaw();
                                })
                                .waitSeconds(postScoringWait)
                                .setReversed(true)
                                .UNSTABLE_addTemporalMarkerOffset(mediumScoringOffset, () ->{
//                                    scorer.togglePassthrough();
                                })
                                .splineTo(turnPos, turnPosToStack)
                                .splineTo(
                                        stackPos,
                                        facingRight
                                )
                                .UNSTABLE_addTemporalMarkerOffset(stackApproachOffset, () -> {
//                                    scorer.liftClaw();
                                })
                                .waitSeconds(stackWait)
                                .addTemporalMarker(() ->{
//                                    scorer.togglePassthrough();
                                })
                                .setReversed(false)
                                .splineTo(turnPos, turnPosToPole)
                                .splineTo(medScoringPos, scoringAngleRight)
                                .UNSTABLE_addTemporalMarkerOffset(liftTime, () -> {
//                                    scorer.setLiftPos(PowerplayScorer.liftHeights.MED);
                                })
                                .UNSTABLE_addTemporalMarkerOffset(mediumApproachOffset, () -> {
//                                    scorer.dropClaw();
                                })
                                .waitSeconds(postScoringWait)
                                .build()
                )
        ;

        meepMeep.setBackground(com.noahbres.meepmeep.MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(false)
                .setBackgroundAlpha(1.0f)
                .addEntity(bot1)
                .start();
    }
}