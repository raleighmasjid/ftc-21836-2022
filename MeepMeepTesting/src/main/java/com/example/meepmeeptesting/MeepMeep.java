package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


public class MeepMeep {
    public static void main(String[] args) {
        com.noahbres.meepmeep.MeepMeep meepMeep = new com.noahbres.meepmeep.MeepMeep(700);

        Pose2d startPose = new Pose2d(35, -62.5, Math.toRadians(90));

        RoadRunnerBotEntity bot1 = new DefaultBotBuilder(meepMeep)
                .setDimensions(17, 16)
                .setStartPose(startPose)
                .setConstraints(50, 50, Math.toRadians(158.4000248984491), Math.toRadians(190), 16.47)
                .followTrajectorySequence(drivetrain ->
                        drivetrain.trajectorySequenceBuilder(startPose)
                                .forward(62.5)
                                .build()
                );

        meepMeep.setBackground(com.noahbres.meepmeep.MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(false)
                .setBackgroundAlpha(1.0f)
                .addEntity(bot1)
                .start();
    }
}