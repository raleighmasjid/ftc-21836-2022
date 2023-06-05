package org.firstinspires.ftc.teamcode.autonomous.opmode;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomous.AutonConfig;
import org.firstinspires.ftc.teamcode.autonomous.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.control.HeadingHolder;
import org.firstinspires.ftc.teamcode.robot.PowerplayScorer;
import org.firstinspires.ftc.teamcode.robot.RobotConfig;


@Autonomous(name = "1+5 Tall", group = "21836 Autonomous")
public class Tall6Right extends BaseAuton {

    @Override
    public void runOpMode() throws InterruptedException {

        parkingZone1 = new Pose2d(side * (isRight ? AutonConfig.ZONE_1_X : AutonConfig.ZONE_3_X), AutonConfig.MAIN_Y, !isRight ? facingRight : facingLeft);

        drivetrain.setPoseEstimate(startPose);

        TrajectorySequence trajectory1 = drivetrain.trajectorySequenceBuilder(startPose)
                .setReversed(false)
                .addTemporalMarker(() -> scorer.liftClaw())
                .splineTo(new Vector2d(centerPathX, -25), facingForward)
                .splineToSplineHeading(tallScoringPos, tallScoringPos.getHeading())
                .UNSTABLE_addTemporalMarkerOffset(-RobotConfig.TIME_LIFT_TALL, () -> scorer.setTargetLiftPos(PowerplayScorer.LiftPos.TALL))
                .addTemporalMarker(() -> scorer.dropCone(PowerplayScorer.LiftPos.FIVE))
                .waitSeconds(AutonConfig.TIME_DROP)
                .UNSTABLE_addTemporalMarkerOffset(AutonConfig.TIME_POST_DROP, () -> scorer.triggerPassThru())
                .setReversed(true)
                .setTangent(tallScoringPos.getHeading() + facingLeft)
                .splineTo(sideTurnPos, stack + (isRight ? facingRight : facingLeft))
                .splineTo(stackPos, isRight ? facingRight : facingLeft)
                .addTemporalMarker(() -> scorer.grabCone())
                .waitSeconds(AutonConfig.TIME_GRAB)
                .UNSTABLE_addTemporalMarkerOffset(AutonConfig.TIME_POST_GRAB, () -> scorer.triggerPassThru())
                // loop below
                .setReversed(false)
                .splineTo(sideTurnPos, stack + (isRight ? facingLeft : facingRight))
                .splineToSplineHeading(tallScoringPos, tallScoringPos.getHeading())
                .UNSTABLE_addTemporalMarkerOffset(-RobotConfig.TIME_LIFT_TALL, () -> scorer.setTargetLiftPos(PowerplayScorer.LiftPos.TALL))
                .addTemporalMarker(() -> scorer.dropCone(PowerplayScorer.LiftPos.FOUR))
                .waitSeconds(AutonConfig.TIME_DROP)
                .UNSTABLE_addTemporalMarkerOffset(AutonConfig.TIME_POST_DROP, () -> scorer.triggerPassThru())
                .setReversed(true)
                .setTangent(tallScoringPos.getHeading() + facingLeft)
                .splineTo(sideTurnPos, stack + (isRight ? facingRight : facingLeft))
                .splineTo(stackPos, isRight ? facingRight : facingLeft)
                .addTemporalMarker(() -> scorer.grabCone())
                .waitSeconds(AutonConfig.TIME_GRAB)
                .UNSTABLE_addTemporalMarkerOffset(AutonConfig.TIME_POST_GRAB, () -> scorer.triggerPassThru())
                .build();

        TrajectorySequence parkLeft = isRight ?
                drivetrain.trajectorySequenceBuilder(trajectory1.end())
                        .setReversed(false)
                        .splineTo(centerTurnPos, isRight ? facingLeft : facingRight)
                        .splineTo(centerTallScoringPos.vec(), centerTallScoringPos.getHeading() + facingLeft)
                        .UNSTABLE_addTemporalMarkerOffset(-RobotConfig.TIME_LIFT_TALL, () -> scorer.setTargetLiftPos(PowerplayScorer.LiftPos.TALL))
                        .addTemporalMarker(() -> scorer.dropCone())
                        .waitSeconds(AutonConfig.TIME_DROP)
                        .UNSTABLE_addTemporalMarkerOffset(AutonConfig.TIME_POST_DROP, () -> scorer.triggerPassThru())
                        .lineToSplineHeading(parkingZone1)
                        .build() :
                drivetrain.trajectorySequenceBuilder(trajectory1.end())
                        .setReversed(false)
                        .splineTo(sideTurnPos, stack + (isRight ? facingLeft : facingRight))
                        .splineToSplineHeading(tallScoringPos, tallScoringPos.getHeading())
                        .UNSTABLE_addTemporalMarkerOffset(-RobotConfig.TIME_LIFT_TALL, () -> scorer.setTargetLiftPos(PowerplayScorer.LiftPos.TALL))
                        .addTemporalMarker(() -> scorer.dropCone())
                        .waitSeconds(AutonConfig.TIME_DROP)
                        .UNSTABLE_addTemporalMarkerOffset(AutonConfig.TIME_POST_DROP, () -> scorer.triggerPassThru())
                        .setReversed(true)
                        .splineTo(sideTurnPos, isRight ? facingRight : facingLeft)
                        .splineTo(parkingZone1.vec(), facingLeft)
                        .build();

        TrajectorySequence parkMiddle = drivetrain.trajectorySequenceBuilder(trajectory1.end())
                .setReversed(false)
                .splineTo(sideTurnPos, stack + (isRight ? facingLeft : facingRight))
                .splineToSplineHeading(tallScoringPos, tallScoringPos.getHeading())
                .UNSTABLE_addTemporalMarkerOffset(-RobotConfig.TIME_LIFT_TALL, () -> scorer.setTargetLiftPos(PowerplayScorer.LiftPos.TALL))
                .addTemporalMarker(() -> scorer.dropCone())
                .waitSeconds(AutonConfig.TIME_DROP)
                .UNSTABLE_addTemporalMarkerOffset(AutonConfig.TIME_POST_DROP, () -> scorer.triggerPassThru())
                .lineToSplineHeading(parkingZone2)
                .build();

        TrajectorySequence parkRight = isRight ?
                drivetrain.trajectorySequenceBuilder(trajectory1.end())
                        .setReversed(false)
                        .splineTo(sideTurnPos, stack + (isRight ? facingLeft : facingRight))
                        .splineToSplineHeading(tallScoringPos, tallScoringPos.getHeading())
                        .UNSTABLE_addTemporalMarkerOffset(-RobotConfig.TIME_LIFT_TALL, () -> scorer.setTargetLiftPos(PowerplayScorer.LiftPos.TALL))
                        .addTemporalMarker(() -> scorer.dropCone())
                        .waitSeconds(AutonConfig.TIME_DROP)
                        .UNSTABLE_addTemporalMarkerOffset(AutonConfig.TIME_POST_DROP, () -> scorer.triggerPassThru())
                        .setReversed(true)
                        .splineTo(sideTurnPos, isRight ? facingRight : facingLeft)
                        .splineTo(parkingZone3.vec(), facingRight)
                        .build() :
                drivetrain.trajectorySequenceBuilder(trajectory1.end())
                        .setReversed(false)
                        .splineTo(centerTurnPos, isRight ? facingLeft : facingRight)
                        .splineTo(centerTallScoringPos.vec(), centerTallScoringPos.getHeading() + facingLeft)
                        .UNSTABLE_addTemporalMarkerOffset(-RobotConfig.TIME_LIFT_TALL, () -> scorer.setTargetLiftPos(PowerplayScorer.LiftPos.TALL))
                        .addTemporalMarker(() -> scorer.dropCone())
                        .waitSeconds(AutonConfig.TIME_DROP)
                        .UNSTABLE_addTemporalMarkerOffset(AutonConfig.TIME_POST_DROP, () -> scorer.triggerPassThru())
                        .lineToSplineHeading(parkingZone3)
                        .build();

        HeadingHolder.setHeading(isRight ? 90.0 : 270.0);

        super.runOpMode(trajectory1, parkLeft, parkMiddle, parkRight);
    }
}