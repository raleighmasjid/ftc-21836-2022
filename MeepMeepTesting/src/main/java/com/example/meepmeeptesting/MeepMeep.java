package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


public class MeepMeep {

    public static double CLAW_CLOSING_TIME = 0.31;
    public static double CLAW_OPEN_TO_DROP_TIME = 0.2;
    public static double AUTON_START_DELAY = 0.15;

    public static void main(String[] args) {
        com.noahbres.meepmeep.MeepMeep meepMeep = new com.noahbres.meepmeep.MeepMeep(650);

        Vector2d stackPos = new Vector2d(58, -12.5);
        Vector2d turnPos = new Vector2d(49, -12.5);
        Vector2d medScoringPos = new Vector2d(31, -21);

        Vector2d parkingZone1 = new Vector2d(12.5, -12.5);
        Vector2d parkingZone2 = new Vector2d(35, -12.5);
        Vector2d parkingZone3 = new Vector2d(57, -12.5);

        double facingRight = Math.toRadians(0);
        double facingForward = Math.toRadians(90);
        double facingLeft = Math.toRadians(180);
        double scoringAngleRight = Math.toRadians(210);



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
//                                    scorer.setLiftPos(PowerplayScorer.heightVal.MED);
                                })
                                .splineToSplineHeading(new Pose2d(35, -53, Math.toRadians(180)), facingForward)
                                .splineToSplineHeading(new Pose2d(35, -24, Math.toRadians(180)), facingForward)
                                .waitSeconds(CLAW_OPEN_TO_DROP_TIME)
                                .lineTo(new Vector2d(32, -24))
                                .addTemporalMarker(() -> {
//                                    scorer.clawIsOpen = true;
                                })
                                .waitSeconds(CLAW_OPEN_TO_DROP_TIME)
                                .addTemporalMarker(() -> {
//                                    scorer.setLiftPos(PowerplayScorer.heightVal.FIVE);
                                })
                                .lineTo(new Vector2d(35, -24))
                                .addTemporalMarker(() -> {
//                                    scorer.togglePassthrough();
                                })
                                .setReversed(true)
                                .lineTo(new Vector2d(35, -12))
                                .waitSeconds(CLAW_CLOSING_TIME)
                                .lineTo(turnPos)
                                .splineTo(
                                        stackPos,
                                        facingRight
                                )
                                .waitSeconds(CLAW_OPEN_TO_DROP_TIME)
                                .addTemporalMarker(() -> {
//                                    scorer.clawIsOpen = false;
                                })
                                .waitSeconds(CLAW_CLOSING_TIME)
                                .addTemporalMarker(() ->{
//                                    scorer.togglePassthrough();
//                                    scorer.setLiftPos(PowerplayScorer.heightVal.MED);
                                })
                                .setReversed(false)
                                .splineTo(turnPos, facingLeft)
                                .splineTo(medScoringPos, scoringAngleRight)
                                .addTemporalMarker(() -> {
//                                    scorer.clawIsOpen = true;
                                })
                                .waitSeconds(CLAW_OPEN_TO_DROP_TIME)
                                .addTemporalMarker(() -> {
//                                    scorer.setLiftPos(PowerplayScorer.heightVal.FOUR);
                                })
                                .setReversed(true)
                                .splineTo(turnPos, facingRight)
                                .setReversed(false)
                                .splineTo(parkingZone2, facingForward)

                                .lineTo(parkingZone3)
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