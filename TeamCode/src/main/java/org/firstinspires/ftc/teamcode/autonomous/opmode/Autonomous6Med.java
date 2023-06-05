package org.firstinspires.ftc.teamcode.autonomous.opmode;


import android.annotation.SuppressLint;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomous.AutonConfig;
import org.firstinspires.ftc.teamcode.autonomous.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.control.HeadingHolder;
import org.firstinspires.ftc.teamcode.robot.PowerplayScorer;
import org.firstinspires.ftc.teamcode.robot.RobotConfig;
import org.openftc.apriltag.AprilTagDetection;


@Autonomous(name = "1+5 Medium", group = "21836 Autonomous")
public class Autonomous6Med extends Autonomous6Base {

    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d centerTallScoringPos = new Pose2d(medScoringPos.getX() - side * AutonConfig.ONE_TILE, medScoringPos.getY(), medScoringPos.getHeading());

        TrajectorySequence trajectory1 = drivetrain.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .addTemporalMarker(() -> scorer.liftClaw())
                .UNSTABLE_addTemporalMarkerOffset(AutonConfig.TIME_FIRST_FLIP, () -> scorer.triggerPassThru())
                .lineTo(parkingZone2.vec())
                .lineToSplineHeading(medScoringPos)
                .UNSTABLE_addTemporalMarkerOffset(-RobotConfig.TIME_LIFT_MEDIUM, () -> scorer.setTargetLiftPos(PowerplayScorer.LiftPos.MED))
                .addTemporalMarker(() -> scorer.dropCone(PowerplayScorer.LiftPos.FIVE))
                .waitSeconds(AutonConfig.TIME_DROP)
                .UNSTABLE_addTemporalMarkerOffset(AutonConfig.TIME_POST_DROP, () -> scorer.triggerPassThru())
                .setReversed(false)
                .splineTo(sideTurnPos, isRight ? facingRight : facingLeft)
                .splineTo(stackPos, isRight ? facingRight : facingLeft)
                .addTemporalMarker(() -> scorer.grabCone())
                .waitSeconds(AutonConfig.TIME_GRAB)
                .UNSTABLE_addTemporalMarkerOffset(AutonConfig.TIME_POST_GRAB, () -> scorer.triggerPassThru())
                // loop below
                .setReversed(true)
                .splineTo(sideTurnPos, isRight ? facingLeft : facingRight)
                .splineToSplineHeading(medScoringPos, medScoringPos.getHeading() - facingLeft)
                .UNSTABLE_addTemporalMarkerOffset(-RobotConfig.TIME_LIFT_MEDIUM, () -> scorer.setTargetLiftPos(PowerplayScorer.LiftPos.MED))
                .addTemporalMarker(() -> scorer.dropCone(PowerplayScorer.LiftPos.FOUR))
                .waitSeconds(AutonConfig.TIME_DROP)
                .UNSTABLE_addTemporalMarkerOffset(AutonConfig.TIME_POST_DROP, () -> scorer.triggerPassThru())
                .setReversed(false)
                .splineTo(sideTurnPos, isRight ? facingRight : facingLeft)
                .splineTo(stackPos, isRight ? facingRight : facingLeft)
                .addTemporalMarker(() -> scorer.grabCone())
                .waitSeconds(AutonConfig.TIME_GRAB)
                .UNSTABLE_addTemporalMarkerOffset(AutonConfig.TIME_POST_GRAB, () -> scorer.triggerPassThru())
                .build();

        TrajectorySequence parkLeft = isRight ?
                drivetrain.trajectorySequenceBuilder(trajectory1.end())
                        .setReversed(true)
                        .splineTo(centerTurnPos, isRight ? facingLeft : facingRight)
                        .splineToSplineHeading(centerTallScoringPos, medScoringPos.getHeading() - facingLeft)
                        .UNSTABLE_addTemporalMarkerOffset(-RobotConfig.TIME_LIFT_MEDIUM, () -> scorer.setTargetLiftPos(PowerplayScorer.LiftPos.TALL))
                        .addTemporalMarker(() -> scorer.dropCone())
                        .waitSeconds(AutonConfig.TIME_DROP)
                        .UNSTABLE_addTemporalMarkerOffset(AutonConfig.TIME_POST_DROP, () -> scorer.triggerPassThru())
                        .lineToSplineHeading(parkingZone1)
                        .build() :
                drivetrain.trajectorySequenceBuilder(trajectory1.end())
                        .setReversed(true)
                        .splineTo(sideTurnPos, isRight ? facingLeft : facingRight)
                        .splineToSplineHeading(medScoringPos, medScoringPos.getHeading() - facingLeft)
                        .UNSTABLE_addTemporalMarkerOffset(-RobotConfig.TIME_LIFT_MEDIUM, () -> scorer.setTargetLiftPos(PowerplayScorer.LiftPos.MED))
                        .addTemporalMarker(() -> scorer.dropCone())
                        .waitSeconds(AutonConfig.TIME_DROP)
                        .UNSTABLE_addTemporalMarkerOffset(AutonConfig.TIME_POST_DROP, () -> scorer.triggerPassThru())
                        .setReversed(false)
                        .splineTo(sideTurnPos, isRight ? facingRight : facingLeft)
                        .splineTo(parkingZone1.vec(), parkingZone1.getHeading())
                        .build();

        TrajectorySequence parkMiddle = drivetrain.trajectorySequenceBuilder(trajectory1.end())
                .setReversed(true)
                .splineTo(sideTurnPos, isRight ? facingLeft : facingRight)
                .splineToSplineHeading(medScoringPos, medScoringPos.getHeading() - facingLeft)
                .UNSTABLE_addTemporalMarkerOffset(-RobotConfig.TIME_LIFT_MEDIUM, () -> scorer.setTargetLiftPos(PowerplayScorer.LiftPos.MED))
                .addTemporalMarker(() -> scorer.dropCone())
                .waitSeconds(AutonConfig.TIME_DROP)
                .UNSTABLE_addTemporalMarkerOffset(AutonConfig.TIME_POST_DROP, () -> scorer.triggerPassThru())
                .lineToSplineHeading(parkingZone2)
                .build();

        TrajectorySequence parkRight = isRight ?
                drivetrain.trajectorySequenceBuilder(trajectory1.end())
                        .setReversed(true)
                        .splineTo(sideTurnPos, isRight ? facingLeft : facingRight)
                        .splineToSplineHeading(medScoringPos, medScoringPos.getHeading() - facingLeft)
                        .UNSTABLE_addTemporalMarkerOffset(-RobotConfig.TIME_LIFT_MEDIUM, () -> scorer.setTargetLiftPos(PowerplayScorer.LiftPos.MED))
                        .addTemporalMarker(() -> scorer.dropCone())
                        .waitSeconds(AutonConfig.TIME_DROP)
                        .UNSTABLE_addTemporalMarkerOffset(AutonConfig.TIME_POST_DROP, () -> scorer.triggerPassThru())
                        .setReversed(false)
                        .splineTo(sideTurnPos, isRight ? facingRight : facingLeft)
                        .splineTo(parkingZone3.vec(), parkingZone3.getHeading())
                        .build() :
                drivetrain.trajectorySequenceBuilder(trajectory1.end())
                        .setReversed(true)
                        .splineTo(centerTurnPos, isRight ? facingLeft : facingRight)
                        .splineToSplineHeading(centerTallScoringPos, medScoringPos.getHeading() - facingLeft)
                        .UNSTABLE_addTemporalMarkerOffset(-RobotConfig.TIME_LIFT_MEDIUM, () -> scorer.setTargetLiftPos(PowerplayScorer.LiftPos.TALL))
                        .addTemporalMarker(() -> scorer.dropCone())
                        .waitSeconds(AutonConfig.TIME_DROP)
                        .UNSTABLE_addTemporalMarkerOffset(AutonConfig.TIME_POST_DROP, () -> scorer.triggerPassThru())
                        .lineToSplineHeading(parkingZone3)
                        .build();

        HeadingHolder.setHeading(isRight ? 270.0 : 90.0);

        super.runOpMode(trajectory1, parkLeft, parkMiddle, parkRight);
    }

    @SuppressLint("DefaultLocale")
    void tagToTelemetry(@NonNull AprilTagDetection detection) {
        myTelemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
    }
}