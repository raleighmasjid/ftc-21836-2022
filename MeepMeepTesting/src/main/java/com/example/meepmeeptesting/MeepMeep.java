package com.example.meepmeeptesting;

import static com.example.meepmeeptesting.DriveConstants.MAX_ACCEL;
import static com.example.meepmeeptesting.DriveConstants.MAX_ANG_VEL;
import static com.example.meepmeeptesting.DriveConstants.TRACK_WIDTH;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.SampleMecanumDrive;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


public class MeepMeep {

    public static double
            END_HEADING = 0.0,
            END_HEADING_OFFSET = -90.0,
            END_HEADING_MULTIPLIER = 1,
            START_TURN_Y = -52,
            MED_ANGLE = 40,
            MED_X = 33,
            MED_Y = -18,
            TALL_ANGLE = -35.0,
            TALL_X = 31.0,
            TALL_Y = -7.5,
            STACK_X = 60,
            STACK_Y = -13,
            SIDE_TURN_ANGLE_OFFSET_MED = 0,
            SIDE_TURN_ANGLE_OFFSET_TALL = -2.0,
            ONE_TILE = 24.0,
            ZONE_1_X = 12.5,
            ZONE_2_X = 35.0,
            ZONE_3_X = 57.0,
            X_TURN_POS = 46.0,
            Y_START = -62.5,
            Y_MAIN_PATH = -13,
            TIME_PRE_GRAB = 1.0,
            TIME_GRAB = 1.0,
            TIME_PRE_DROP = 1.0,
            TIME_DROP = 0,
            TIME_DROP_TO_FLIP = 0,
            TIME_FIRST_FLIP = 3,
            TIME_LIFT_MEDIUM = 1,
            TIME_LIFT_TALL = 1.2,
            TIME_FLIP = 3,
            STACK_VELO = 10,
            STACK_ACCEL = MAX_ACCEL,
            SCORING_VELO = 10,
            SCORING_ACCEL = MAX_ACCEL;

    public static final double
            RIGHT = Math.toRadians(0),
            FORWARD = Math.toRadians(90),
            LEFT = Math.toRadians(180);

    private static final TrajectoryVelocityConstraint stackVeloCap = SampleMecanumDrive.getVelocityConstraint(STACK_VELO, MAX_ANG_VEL, TRACK_WIDTH);
    private static final TrajectoryAccelerationConstraint stackAccelCap = SampleMecanumDrive.getAccelerationConstraint(STACK_ACCEL);
    private static final TrajectoryVelocityConstraint scoringVeloCap = SampleMecanumDrive.getVelocityConstraint(SCORING_VELO, MAX_ANG_VEL, TRACK_WIDTH);
    private static final TrajectoryAccelerationConstraint scoringAccelCap = SampleMecanumDrive.getAccelerationConstraint(SCORING_ACCEL);

    public static void main(String[] args) {
        com.noahbres.meepmeep.MeepMeep meepMeep = new com.noahbres.meepmeep.MeepMeep(700);

        boolean isRight = true;
        boolean tallPole = false;

        double side = isRight ? 1 : -1;

        double X_START = side * ZONE_2_X;

        Vector2d stackPos = new Vector2d(side * STACK_X, STACK_Y);
        Vector2d sideTurnPos = new Vector2d(side * X_TURN_POS, Y_MAIN_PATH);
        Vector2d centerTurnPos = new Vector2d(sideTurnPos.getX() - side * ONE_TILE, sideTurnPos.getY());
        Pose2d tallScoringPos = new Pose2d(side * TALL_X, TALL_Y, Math.toRadians(isRight ? TALL_ANGLE : 180 - TALL_ANGLE));
        Pose2d medScoringPos = new Pose2d(side * MED_X, MED_Y, Math.toRadians(isRight ? MED_ANGLE : 180 - MED_ANGLE));
        Pose2d centerTallScoringPos = new Pose2d(medScoringPos.getX() - side * ONE_TILE, medScoringPos.getY(), medScoringPos.getHeading());

        double zone1x = side * (isRight ? ZONE_1_X : ZONE_3_X);
        double zone3x = side * (isRight ? ZONE_3_X : ZONE_1_X);
        Pose2d parkingZone1 = new Pose2d(zone1x, Y_MAIN_PATH, isRight ? RIGHT : LEFT);
        Pose2d parkingZone2 = new Pose2d(X_START, parkingZone1.getY(), parkingZone1.getHeading());
        Pose2d parkingZone3 = new Pose2d(zone3x, parkingZone1.getY(), parkingZone1.getHeading());

        Pose2d startPose = new Pose2d(X_START, Y_START, FORWARD);

        double TIME_LIFT = tallPole ? TIME_LIFT_TALL : TIME_LIFT_MEDIUM;
        Pose2d scoringPos = tallPole ? tallScoringPos : medScoringPos;
        double SIDE_TURN_ANGLE_OFFSET = side * Math.toRadians(tallPole ? SIDE_TURN_ANGLE_OFFSET_TALL : SIDE_TURN_ANGLE_OFFSET_MED);

        RoadRunnerBotEntity bot1 = new DefaultBotBuilder(meepMeep)
                .setDimensions(17, 16)
                .setStartPose(startPose)
                .setConstraints(50, 50, Math.toRadians(158.4000248984491), Math.toRadians(190), 16.47)
                .followTrajectorySequence(drivetrain ->
                        drivetrain.trajectorySequenceBuilder(startPose)
                                .lineToSplineHeading(new Pose2d(X_START, START_TURN_Y, RIGHT))
                                .lineTo(parkingZone2.vec())
                                .lineToSplineHeading(scoringPos, scoringVeloCap, scoringAccelCap)
                                .build()
                );

        meepMeep.setBackground(com.noahbres.meepmeep.MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(false)
                .setBackgroundAlpha(1.0f)
                .addEntity(bot1)
                .start();
    }
}