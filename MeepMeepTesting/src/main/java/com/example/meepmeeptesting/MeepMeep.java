package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


public class MeepMeep {

    public static double CLAW_CLOSING_TIME = 0.3;
    public static double CLAW_OPEN_TO_DROP_TIME = 0.2;

    public static double AUTON_START_DELAY = 0.16;

    public static void main(String[] args) {
        com.noahbres.meepmeep.MeepMeep meepMeep = new com.noahbres.meepmeep.MeepMeep(650);


        Vector2d stackPos = new Vector2d(59, -12.5);
        Vector2d turnPos = new Vector2d(47, -13);
        Vector2d medScoringPos = new Vector2d(30.5, -18);

        double centerPathX = 35;

        Vector2d parkingZone1 = new Vector2d(12.5, -12.5);
        Vector2d parkingZone2 = new Vector2d(centerPathX, -12.5);
        Vector2d parkingZone3 = new Vector2d(57, -12.5);

        double facingRight = Math.toRadians(0);
        double facingForward = Math.toRadians(90);
        double facingLeft = Math.toRadians(180);
        double scoringAngleRight = Math.toRadians(215);

        double mediumScoringOffset = 0.1;
        double stackOffset = 0.5;

        Pose2d startPose = new Pose2d(35, -62.5, facingForward);

        RoadRunnerBotEntity bot1 = new DefaultBotBuilder(meepMeep)
                .setDimensions(17, 16)
                .setStartPose(startPose)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(140), Math.toRadians(190), 14.25)
                .followTrajectorySequence(drive->
                        drive.trajectorySequenceBuilder(startPose)
                                .addTemporalMarker(() -> {
//                                    scorer.clawIsOpen = false;
                                })
                                .waitSeconds(CLAW_CLOSING_TIME + AUTON_START_DELAY)
                                .addTemporalMarker(() -> {
//                                    scorer.setLiftPos(PowerplayScorer.liftHeights.MED);
                                })
                                .splineToSplineHeading(new Pose2d(centerPathX, -53, facingLeft), facingForward)
                                .splineToSplineHeading(new Pose2d(centerPathX, -25, facingLeft), facingForward)
                                .waitSeconds(CLAW_OPEN_TO_DROP_TIME)
                                .lineTo(new Vector2d(31.5, -25))
                                .addTemporalMarker(() -> {
//                                    scorer.setLiftPos(PowerplayScorer.liftHeights.FIVE);
//                                    scorer.clawIsOpen = true;
                                })
                                .lineTo(new Vector2d(centerPathX, -25))
                                .addTemporalMarker(() -> {
//                                    scorer.togglePassthrough();
                                })
                                .setReversed(true)
                                .lineTo(new Vector2d(centerPathX, -12.5))
                                .lineTo(turnPos)
                                .splineTo(
                                        stackPos,
                                        facingRight
                                )
                                .waitSeconds(CLAW_OPEN_TO_DROP_TIME)
                                .addTemporalMarker(() -> {
//                                    scorer.liftClaw();
                                })
                                .waitSeconds(CLAW_CLOSING_TIME)
                                .addTemporalMarker(() ->{
//                                    scorer.togglePassthrough();
//                                    scorer.setLiftPos(PowerplayScorer.liftHeights.MED);
                                })
                                .waitSeconds(stackOffset)
                                .setReversed(false)
                                .splineTo(turnPos, facingLeft)
                                .splineTo(medScoringPos, scoringAngleRight)
                                .addTemporalMarker(() -> {
//                                    scorer.setLiftPos(PowerplayScorer.liftHeights.FOUR);
//                                    scorer.clawIsOpen = true;
                                })
                                .setReversed(true)
                                .UNSTABLE_addTemporalMarkerOffset(mediumScoringOffset, () ->{
//                                    scorer.togglePassthrough();
                                })
                                .splineTo(turnPos, facingRight)
                                .splineTo(
                                        stackPos,
                                        facingRight
                                )
                                .waitSeconds(CLAW_OPEN_TO_DROP_TIME)
                                .addTemporalMarker(() -> {
//                                    scorer.liftClaw();
                                })
                                .waitSeconds(CLAW_CLOSING_TIME)
                                .addTemporalMarker(() ->{
//                                    scorer.togglePassthrough();
//                                    scorer.setLiftPos(PowerplayScorer.liftHeights.MED);
                                })
                                .waitSeconds(stackOffset)
                                .setReversed(false)
                                .splineTo(turnPos, facingLeft)
                                .splineTo(medScoringPos, scoringAngleRight)
                                .addTemporalMarker(() -> {
//                                    scorer.setLiftPos(PowerplayScorer.liftHeights.THREE);
//                                    scorer.clawIsOpen = true;
                                })
                                .setReversed(true)
                                .UNSTABLE_addTemporalMarkerOffset(mediumScoringOffset, () ->{
//                                    scorer.togglePassthrough();
                                })
                                .splineTo(turnPos, facingRight)
                                .splineTo(
                                        stackPos,
                                        facingRight
                                )
                                .waitSeconds(CLAW_OPEN_TO_DROP_TIME)
                                .addTemporalMarker(() -> {
//                                    scorer.liftClaw();
                                })
                                .waitSeconds(CLAW_CLOSING_TIME)
                                .addTemporalMarker(() ->{
//                                    scorer.togglePassthrough();
//                                    scorer.setLiftPos(PowerplayScorer.liftHeights.MED);
                                })
                                .waitSeconds(stackOffset)
                                .setReversed(false)
                                .splineTo(turnPos, facingLeft)
                                .splineTo(medScoringPos, scoringAngleRight)
                                .addTemporalMarker(() -> {
//                                    scorer.setLiftPos(PowerplayScorer.liftHeights.TWO);
//                                    scorer.clawIsOpen = true;
                                })
                                .setReversed(true)
                                .splineTo(turnPos, facingRight)
                                .addTemporalMarker(() -> {
//                                    scorer.setLiftPos(PowerplayScorer.liftHeights.ONE);
                                })
                                .setReversed(false)
                                .splineTo(parkingZone2, facingForward)

                                .setTangent(facingRight)
                                .splineTo(turnPos, facingRight)
                                .splineTo(parkingZone3, facingRight)

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