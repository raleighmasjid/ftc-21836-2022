package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.ColorScheme;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(650);

        RoadRunnerBotEntity bot1 = new DefaultBotBuilder(meepMeep)
                .setDimensions(16.4, 15.5)
                .setStartPose(new Pose2d(35, -62.5, Math.toRadians(90)))
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(65, 65, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive->
                        drive.trajectorySequenceBuilder(new Pose2d(35, -60, Math.toRadians(90)))
                                .splineTo(new Vector2d(35, -25), Math.toRadians(90))
                                .splineTo(new Vector2d(28, -7), Math.toRadians(120))
                                .waitSeconds(0.1)
                                .setReversed(true)
                                .splineToSplineHeading(new Pose2d(45, -14, Math.toRadians(90)), Math.toRadians(0))
                                .splineToSplineHeading(new Pose2d(60, -12, Math.toRadians(0)), Math.toRadians(0))
                                .waitSeconds(0.1) // liftClaw
                                .splineToSplineHeading(new Pose2d(45, -14, Math.toRadians(90)), Math.toRadians(180))
                                .splineToSplineHeading(new Pose2d(27, -6, Math.toRadians(120)), Math.toRadians(-250))
                                .waitSeconds(0.1) // dropClaw.
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(false)
                .setBackgroundAlpha(1.0f)
                .addEntity(bot1)
//                .addEntity(bot2)
                .start();
    }
}