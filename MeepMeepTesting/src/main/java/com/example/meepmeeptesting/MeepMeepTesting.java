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
//                                .splineTo(new Vector2d(35, -25), Math.toRadians(90))
//                                .splineTo(new Vector2d(27, -6), Math.toRadians(120))
//
//                                .waitSeconds(0.1)
//
//                                .setReversed(true)
//                                .splineToSplineHeading(new Pose2d(45, -14, Math.toRadians(90)), Math.toRadians(0))
//                                .splineToSplineHeading(new Pose2d(61, -12, Math.toRadians(0)), Math.toRadians(0))
//                                .waitSeconds(0.1) // liftClaw
//                                .splineToSplineHeading(new Pose2d(43, -14, Math.toRadians(90)), Math.toRadians(180))
//                                .splineToSplineHeading(new Pose2d(27, -6, Math.toRadians(120)), Math.toRadians(-240))
//                                .waitSeconds(0.1) // dropClaw.
public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(650);

        RoadRunnerBotEntity bot1 = new DefaultBotBuilder(meepMeep)
                .setDimensions(16.4, 15.5)
                .setStartPose(new Pose2d(35, -62.5, Math.toRadians(90)))
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(65, 65, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive->
                        drive.trajectorySequenceBuilder(new Pose2d(35, -62.5, Math.toRadians(90)))
                                .splineToConstantHeading(new Vector2d(40, -61), Math.toRadians(0))
                                .splineToConstantHeading(new Vector2d(55, -61), Math.toRadians(0))
                                .waitSeconds(0.01)
                                .lineToConstantHeading(new Vector2d(35, -60))
                                .lineToConstantHeading(new Vector2d(35, -12))
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