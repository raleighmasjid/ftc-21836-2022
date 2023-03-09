package org.firstinspires.ftc.teamcode.drive.opmode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.PowerplayScorer;
import org.firstinspires.ftc.teamcode.TeleOpConfig;
import org.firstinspires.ftc.teamcode.auton.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.drive.AutonMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;


@Autonomous(name= "Right - 1+4 medium", group = "21836 Autonomous")
public class AutonomousRight5Med extends LinearOpMode {

    OpenCvCamera camera;
    AprilTagDetectionPipeline signalSleeveDetectionPipeline;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = TeleOpConfig.fx;
    double fy = TeleOpConfig.fy;
    double cx = TeleOpConfig.cx;
    double cy = TeleOpConfig.cy;

    // UNITS ARE METERS
    double tagSize = 0.166;

    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    static ElapsedTime autonomousTimer = new ElapsedTime();

    boolean hasParked = false;

    AprilTagDetection tagOfInterest = null;

    PowerplayScorer scorer = new PowerplayScorer();

    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        signalSleeveDetectionPipeline = new AprilTagDetectionPipeline(tagSize, fx, fy, cx, cy);

        camera.setPipeline(signalSleeveDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.SIDEWAYS_RIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        AutonMecanumDrive drive = new AutonMecanumDrive(hardwareMap);
        scorer.init(hardwareMap);

        //  Initialize telemetry and dashboard
        MultipleTelemetry myTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        FtcDashboard dashboard = FtcDashboard.getInstance();
        dashboard.startCameraStream(camera,0);

        Vector2d stackPos = new Vector2d(59, -12.5);
        Vector2d turnPos = new Vector2d(47, -13);
        Vector2d medScoringPos = new Vector2d(31, -17.5);

        double centerPathX = 35;

        Vector2d parkingZone1 = new Vector2d(13, -12.5);
        Vector2d parkingZone2 = new Vector2d(centerPathX, -12.5);
        Vector2d parkingZone3 = new Vector2d(57, -12.5);

        TrajectoryVelocityConstraint stackVeloCap = AutonMecanumDrive.getVelocityConstraint(16, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH);
        TrajectoryVelocityConstraint scoringVeloCap = AutonMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH);
        TrajectoryAccelerationConstraint accelerationCap = AutonMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL);

        double facingRight = Math.toRadians(0);
        double facingForward = Math.toRadians(90);
        double facingLeft = Math.toRadians(180);
        double scoringAngleRight = Math.toRadians(215);

        double mediumScoringOffset = 0.1;
        double liftTime = -0.8;
        double stackApproachOffset = -0.2;
        double firstScoringY = -24;
        double mediumApproachOffset = -0.003;
        double stackWait = 0.1;

        double CLAW_CLOSING_TIME = 0.3;
        double CLAW_OPEN_TO_DROP_TIME = 0.1;
        double AUTON_START_DELAY = 0.16;

        Pose2d startPose = new Pose2d(centerPathX, -62.5, facingForward);
        drive.setPoseEstimate(startPose);

        TrajectorySequence trajectory1 = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(() -> {
                    scorer.clawIsOpen = false;
                })
                .waitSeconds(CLAW_CLOSING_TIME + AUTON_START_DELAY)
                .addTemporalMarker(() -> {
                    scorer.setTargetLiftPos(scorer.getTargetLiftPos() + 5);
                })
                .splineToSplineHeading(new Pose2d(centerPathX, -53, facingLeft), facingForward, scoringVeloCap, accelerationCap)
                .splineToSplineHeading(new Pose2d(centerPathX, firstScoringY, facingLeft), facingForward, scoringVeloCap, accelerationCap)
                .UNSTABLE_addTemporalMarkerOffset(liftTime, () -> {
                    scorer.setTargetLiftPos(PowerplayScorer.liftPos.MED);
                })
                .lineTo(new Vector2d(31.5, firstScoringY))
                .addTemporalMarker(() -> {
                    scorer.setTargetLiftPos(PowerplayScorer.liftPos.FIVE);
                    scorer.clawIsOpen = true;
                })
                .waitSeconds(CLAW_OPEN_TO_DROP_TIME)
                .lineTo(new Vector2d(centerPathX, firstScoringY))
                .addTemporalMarker(() -> {
                    scorer.triggerPassThru();
                })
                .setReversed(true)
                .lineTo(parkingZone2)
                .setTangent(facingRight)
                .splineTo(turnPos, facingRight)
                .splineTo(
                        stackPos,
                        facingRight,
                        stackVeloCap,
                        accelerationCap
                )
                .UNSTABLE_addTemporalMarkerOffset(stackApproachOffset, () -> {
                    scorer.liftClaw();
                })
                .waitSeconds(stackWait)
                .addTemporalMarker(() ->{
                    scorer.triggerPassThru();
                })
                .setReversed(false)
                .splineTo(turnPos, facingLeft)
                .splineTo(medScoringPos, scoringAngleRight, scoringVeloCap, accelerationCap)
                .UNSTABLE_addTemporalMarkerOffset(liftTime, () -> {
                    scorer.setTargetLiftPos(PowerplayScorer.liftPos.MED);
                })
                .addTemporalMarker(() -> {
                    scorer.setTargetLiftPos(PowerplayScorer.liftPos.FOUR);
                    scorer.clawIsOpen = true;
                })
                .waitSeconds(CLAW_OPEN_TO_DROP_TIME)
                .setReversed(true)
                .UNSTABLE_addTemporalMarkerOffset(mediumScoringOffset, () ->{
                    scorer.triggerPassThru();
                })
                .splineTo(turnPos, facingRight)
                .splineTo(
                        stackPos,
                        facingRight,
                        stackVeloCap,
                        accelerationCap
                )
                .addTemporalMarker(() -> {
                    scorer.liftClaw();
                })
                .waitSeconds(stackWait)
                .addTemporalMarker(() ->{
                    scorer.triggerPassThru();
                })
                .setReversed(false)
                .splineTo(turnPos, facingLeft)
                .splineTo(medScoringPos, scoringAngleRight, scoringVeloCap, accelerationCap)
                .UNSTABLE_addTemporalMarkerOffset(liftTime, () -> {
                    scorer.setTargetLiftPos(PowerplayScorer.liftPos.MED);
                })
                .UNSTABLE_addTemporalMarkerOffset(mediumApproachOffset, () -> {
                    scorer.setTargetLiftPos(PowerplayScorer.liftPos.THREE);
                    scorer.clawIsOpen = true;
                })
                .waitSeconds(CLAW_OPEN_TO_DROP_TIME)
                .setReversed(true)
                .UNSTABLE_addTemporalMarkerOffset(mediumScoringOffset, () ->{
                    scorer.triggerPassThru();
                })
                .splineTo(turnPos, facingRight)
                .splineTo(
                        new Vector2d(60, -12.5),
                        facingRight,
                        stackVeloCap,
                        accelerationCap
                )
                .UNSTABLE_addTemporalMarkerOffset(stackApproachOffset, () -> {
                    scorer.liftClaw();
                })
                .waitSeconds(stackWait)
                .addTemporalMarker(() ->{
                    scorer.triggerPassThru();
                })
                .setReversed(false)
                .splineTo(turnPos, facingLeft)
                .splineTo(medScoringPos, scoringAngleRight, scoringVeloCap, accelerationCap)
                .UNSTABLE_addTemporalMarkerOffset(liftTime, () -> {
                    scorer.setTargetLiftPos(PowerplayScorer.liftPos.MED);
                })
                .UNSTABLE_addTemporalMarkerOffset(mediumApproachOffset, () -> {
                    scorer.setTargetLiftPos(PowerplayScorer.liftPos.TWO);
                    scorer.clawIsOpen = true;
                })
                .waitSeconds(CLAW_OPEN_TO_DROP_TIME)
                .setReversed(true)
                .UNSTABLE_addTemporalMarkerOffset(mediumScoringOffset, () ->{
                    scorer.triggerPassThru();
                })
                .splineTo(turnPos, facingRight)
                .splineTo(
                        new Vector2d(60, -12.5),
                        facingRight,
                        stackVeloCap,
                        accelerationCap
                )
                .UNSTABLE_addTemporalMarkerOffset(stackApproachOffset, () -> {
                    scorer.liftClaw();
                })
                .waitSeconds(stackWait)
                .addTemporalMarker(() ->{
                    scorer.triggerPassThru();
                })
                .setReversed(false)
                .splineTo(turnPos, facingLeft)
                .splineTo(medScoringPos, scoringAngleRight, scoringVeloCap, accelerationCap)
                .UNSTABLE_addTemporalMarkerOffset(liftTime, () -> {
                    scorer.setTargetLiftPos(PowerplayScorer.liftPos.MED);
                })
                .UNSTABLE_addTemporalMarkerOffset(mediumApproachOffset, () -> {
                    scorer.dropClaw();
                })
                .waitSeconds(CLAW_OPEN_TO_DROP_TIME)
                .build()
                ;

        TrajectorySequence parkLeft = drive.trajectorySequenceBuilder(trajectory1.end())
                .setTangent(scoringAngleRight - facingLeft)
                .lineToSplineHeading(new Pose2d(35, -12.5, facingLeft))
                .setTangent(facingLeft)
                .splineTo(new Vector2d(23.5, -12), facingLeft)
                .splineTo(parkingZone1, facingLeft)
                .build()
                ;

        TrajectorySequence parkMiddle = drive.trajectorySequenceBuilder(trajectory1.end())
                .setTangent(scoringAngleRight - facingLeft)
                .lineToSplineHeading(new Pose2d(35, -12.5, facingLeft))
                .build()
                ;

        TrajectorySequence parkRight = drive.trajectorySequenceBuilder(trajectory1.end())
                .setReversed(true)
                .splineTo(turnPos, facingRight)
                .splineTo(parkingZone3, facingRight)
                .build()
                ;



        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */

        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = signalSleeveDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);

                }

            }

            telemetry.update();
            sleep(20);
        }

        //START IS HERE//
        autonomousTimer.reset();

        dashboard.stopCameraStream();
        camera.stopStreaming();
        camera.closeCameraDevice();

        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }


        drive.followTrajectorySequenceAsync(trajectory1);

        while(opModeIsActive()) {

            drive.update();

            scorer.runClaw();
            scorer.runPivot();
            scorer.runPassThruServos();
            scorer.runPassThruStates();
            scorer.runLiftToPos();


            // parking statement
            if(
                    !hasParked &&                       // bot has not yet parked in zone
                            !drive.isBusy() &&                  // bot is not driving
                            (autonomousTimer.seconds() >= 3) && // at least 3 seconds into autonomous
                            (tagOfInterest != null)             // camera has detected any tag
            ) {

                if (tagOfInterest.id == LEFT) {
                    drive.followTrajectorySequenceAsync(parkLeft);
                } else if (tagOfInterest.id == RIGHT) {
                    drive.followTrajectorySequenceAsync(parkRight);
                } else {
                    drive.followTrajectorySequenceAsync(parkMiddle);
                }

                hasParked = true;
            }

            if (tagOfInterest == null) {
                drive.followTrajectorySequenceAsync(parkMiddle);
            }



            //everything below is telemetry
            if (scorer.limitSwitch.getState()) {
                myTelemetry.addData("Limit switch", "is not triggered");
            } else {
                myTelemetry.addData("Limit switch", "is triggered");
            }

            if (scorer.limitSwitch.getState()) {
                myTelemetry.addData("Limit switch", "is not triggered");
            } else {
                myTelemetry.addData("Limit switch", "is triggered");
            }

            if (!scorer.clawIsOpen){
                myTelemetry.addData("Claw is", "closed");
            } else if (scorer.passThruIsMoving) {
                myTelemetry.addData("Claw is", "half-closed");
            } else {
                myTelemetry.addData("Claw is", "open");
            }

            myTelemetry.addData("Lift named target position", scorer.getTargetLiftPosName());
            myTelemetry.addData("Lift current position", scorer.getCurrentLiftPos());
            myTelemetry.addData("Lift target position", scorer.getTargetLiftPos());
            myTelemetry.addData("Lift motor power output", scorer.liftVelocity);

            myTelemetry.addData("Passthrough status", scorer.getCurrentPassThruState());

//            myTelemetry.addData("Current draw lift 1",scorer.lift_motor1.motorEx.getCurrent(CurrentUnit.AMPS));
//            myTelemetry.addData("Current draw lift 2",scorer.lift_motor2.motorEx.getCurrent(CurrentUnit.AMPS));
//            myTelemetry.addData("Current draw lift 3",scorer.lift_motor3.motorEx.getCurrent(CurrentUnit.AMPS));

            myTelemetry.update();

        }
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
    }
}