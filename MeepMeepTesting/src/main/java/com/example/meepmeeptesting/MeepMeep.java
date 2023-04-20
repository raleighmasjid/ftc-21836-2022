package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


public class MeepMeep {
    public static void main(String[] args) {
        com.noahbres.meepmeep.MeepMeep meepMeep = new com.noahbres.meepmeep.MeepMeep(700);

        boolean isRight = true;
        double side = isRight? 1: -1;

        double centerPathX = side*35;

        double facingRight = Math.toRadians(0);
        double facingForward = Math.toRadians(90);
        double facingLeft = Math.toRadians(180);
        double liftAndDropTime = 0.1;
        double clawToFlipTime = 0.1;

        Vector2d stackPos = new Vector2d(side*59, -12.5);
        Vector2d sideTurnPos = new Vector2d(side*46, -12.5);
        Pose2d medScoringPos = new Pose2d(side*31, -17.5, Math.toRadians(isRight? 35: 180-35));
        Pose2d tallScoringPos = new Pose2d(side*31, -7.5, Math.toRadians(isRight? 135: 180-135));
        Pose2d centerTallScoringPos = new Pose2d(medScoringPos.getX()-side*24, medScoringPos.getY(), medScoringPos.getHeading());
        Vector2d centerTurnPos = new Vector2d(sideTurnPos.getX()-side*24, sideTurnPos.getY());

        Pose2d parkingZone1 = new Pose2d(side*(isRight? 12.5:57), -12.5, isRight? facingLeft: facingRight);
        Pose2d parkingZone2 = new Pose2d(centerPathX, -12.5, parkingZone1.getHeading());
        Pose2d parkingZone3 = new Pose2d(side*(isRight? 57:12.5), -12.5, parkingZone1.getHeading());

        Pose2d startPose = new Pose2d(centerPathX, -62.5, facingForward);

        RoadRunnerBotEntity bot1 = new DefaultBotBuilder(meepMeep)
                .setDimensions(17, 16)
                .setStartPose(startPose)
                .setConstraints(58, 55, Math.toRadians(140), Math.toRadians(190), 14.25)
                .followTrajectorySequence(drivetrain->
                        drivetrain.trajectorySequenceBuilder(startPose)
                                .setReversed(false)
                                .splineTo(new Vector2d(parkingZone2.getX(), -25), facingForward)
                                .splineToSplineHeading(tallScoringPos, tallScoringPos.getHeading())
//                                .UNSTABLE_addTemporalMarkerOffset(-RobotConfig.TIME_LIFT_TALL, () -> scorer.setTargetLiftPos(PowerplayScorer.liftPos.MED))
//                                .addTemporalMarker(() -> scorer.dropClaw(PowerplayScorer.liftPos.FIVE))
                                .waitSeconds(liftAndDropTime)
//                                .UNSTABLE_addTemporalMarkerOffset(clawToFlipTime, () -> scorer.triggerPassThru())
                                .setReversed(true)
                                .setTangent(tallScoringPos.getHeading() + facingLeft)
                                .splineTo(sideTurnPos, isRight? facingRight: facingLeft)
                                .splineTo(stackPos, isRight? facingRight: facingLeft)
//                                .addTemporalMarker(() -> scorer.liftClaw())
                                .waitSeconds(liftAndDropTime)
//                                .UNSTABLE_addTemporalMarkerOffset(clawToFlipTime, () -> scorer.triggerPassThru())
                                .setReversed(false)
                                .splineTo(sideTurnPos, isRight? facingLeft: facingRight)
                                .splineToSplineHeading(tallScoringPos, tallScoringPos.getHeading())
//                                .UNSTABLE_addTemporalMarkerOffset(-RobotConfig.TIME_LIFT_TALL, () -> scorer.setTargetLiftPos(PowerplayScorer.liftPos.MED))
//                                .addTemporalMarker(() -> scorer.dropClaw(PowerplayScorer.liftPos.FOUR))
                                .waitSeconds(liftAndDropTime)
//                                .UNSTABLE_addTemporalMarkerOffset(clawToFlipTime, () -> scorer.triggerPassThru())
                                .setReversed(true)
                                .setTangent(tallScoringPos.getHeading() + facingLeft)
                                .splineTo(sideTurnPos, isRight? facingRight: facingLeft)
                                .splineTo(stackPos, isRight? facingRight: facingLeft)
//                                .addTemporalMarker(() -> scorer.liftClaw())
                                .waitSeconds(liftAndDropTime)
//                                .UNSTABLE_addTemporalMarkerOffset(clawToFlipTime, () -> scorer.triggerPassThru())

                                .setReversed(false)
                                .splineTo(sideTurnPos, isRight? facingLeft: facingRight)
                                .splineToSplineHeading(tallScoringPos, tallScoringPos.getHeading())
//                                .UNSTABLE_addTemporalMarkerOffset(-RobotConfig.TIME_LIFT_TALL, () -> scorer.setTargetLiftPos(PowerplayScorer.liftPos.MED))
//                                .addTemporalMarker(() -> scorer.dropClaw())
                                .waitSeconds(liftAndDropTime)
//                                .UNSTABLE_addTemporalMarkerOffset(clawToFlipTime, () -> scorer.triggerPassThru())
                                .lineToSplineHeading(parkingZone2)
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