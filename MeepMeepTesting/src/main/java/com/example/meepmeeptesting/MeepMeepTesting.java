package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;



public class MeepMeepTesting {

    public static double CLAW_DROP_TIME = 0.7;
    public static double CLAW_LIFT_TIME = 0.7;

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(650);

        Pose2d startPose = new Pose2d(35, -62.5, Math.toRadians(90));

        RoadRunnerBotEntity bot1 = new DefaultBotBuilder(meepMeep)
                .setDimensions(17, 16)
                .setStartPose(startPose)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(65, 65, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive->
                        drive.trajectorySequenceBuilder(startPose)
                                .splineTo(new Vector2d(35, -45), Math.toRadians(90))
                                .addTemporalMarker(() -> {
//                                    scorer.setLiftPos(PowerplayScorer.heightVal.MED);
                                })
                                .splineToSplineHeading(new Pose2d(35, -23.5, Math.toRadians(180)), Math.toRadians(90))
                                .waitSeconds(0.1)
                                .lineTo(new Vector2d(31, -23.5))
                                .addTemporalMarker(() -> {
//                                    scorer.clawIsOpen = false;
//                                    scorer.setLiftPos(PowerplayScorer.heightVal.FIVE);
                                })
                                .waitSeconds(CLAW_DROP_TIME)
                                .lineTo(new Vector2d(35, -23.5))
                                .waitSeconds(0.5)
                                .lineTo(new Vector2d(35, -12.5))
                                .addTemporalMarker(() -> {
//                                    scorer.togglePassthrough();
                                })
                                .waitSeconds(0.3)
                                .setReversed(true)
                                .splineTo(new Vector2d(61, -12.5), Math.toRadians(0))
                                .addTemporalMarker(() -> {
//                                    scorer.liftClaw();
                                })
                                .waitSeconds(CLAW_LIFT_TIME)
                                .addTemporalMarker(() -> {
//                                    scorer.togglePassthrough();
                                })

                                .setReversed(false)
                                .splineTo(new Vector2d(40, -12.5), Math.toRadians(180))
                                .addTemporalMarker(() -> {
//                                    scorer.setLiftPos(PowerplayScorer.heightVal.MED);
                                })
                                .splineTo(new Vector2d(28.5, -19), Math.toRadians(225))
                                .addTemporalMarker(() -> {
//                                    scorer.clawIsOpen = false;
//                                    scorer.setLiftPos(PowerplayScorer.heightVal.FOUR);
                                })
                                .waitSeconds(CLAW_DROP_TIME)
                                .addTemporalMarker(() -> {
//                                    scorer.togglePassthrough();
                                })
                                .setReversed(true)
                                .splineTo(new Vector2d(40, -12.5), Math.toRadians(0))
                                .splineTo(new Vector2d(61, -12.5), Math.toRadians(0))

                                .build()
                )
        ;

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(false)
                .setBackgroundAlpha(1.0f)
                .addEntity(bot1)
                .start();
    }
}