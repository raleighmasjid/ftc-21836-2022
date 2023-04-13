package org.firstinspires.ftc.teamcode.autonomous.opmode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.control.HeadingHolder;
import org.firstinspires.ftc.teamcode.robot.PowerplayScorer;
import org.firstinspires.ftc.teamcode.robot.RobotConfig;
import org.firstinspires.ftc.teamcode.control.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.autonomous.AutonMecanumDrive;
import org.firstinspires.ftc.teamcode.autonomous.DriveConstants;
import org.firstinspires.ftc.teamcode.autonomous.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.List;


@Autonomous(name= "Active Testing", group = "21836 Backup")
public class AutonomousTesting extends LinearOpMode {

    OpenCvCamera camera;
    AprilTagDetectionPipeline signalSleeveDetectionPipeline;
    List<LynxModule> hubs;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = RobotConfig.fx;
    double fy = RobotConfig.fy;
    double cx = RobotConfig.cx;
    double cy = RobotConfig.cy;

    // UNITS ARE METERS
    double tagSize = 0.166;

    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    static ElapsedTime autonomousTimer = new ElapsedTime();

    boolean hasParked = false;

    AprilTagDetection tagOfInterest = null;

    PowerplayScorer scorer = new PowerplayScorer();
    AutonMecanumDrive drivetrain;

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

        drivetrain = new AutonMecanumDrive(hardwareMap);
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
        HeadingHolder.setHeading(90.0);

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
        drivetrain.setPoseEstimate(startPose);

        TrajectorySequence trajectory1 = drivetrain.trajectorySequenceBuilder(startPose)
//                .addTemporalMarker(() -> {
//                    scorer.clawIsOpen = false;
//                })
//                .waitSeconds(CLAW_CLOSING_TIME + AUTON_START_DELAY)
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
                    scorer.dropClaw(PowerplayScorer.liftPos.FIVE);
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
                    scorer.dropClaw(PowerplayScorer.liftPos.FOUR);
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
                    scorer.dropClaw(PowerplayScorer.liftPos.THREE);
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
                    scorer.dropClaw(PowerplayScorer.liftPos.TWO);
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

        TrajectorySequence parkLeft = drivetrain.trajectorySequenceBuilder(trajectory1.end())
                .setTangent(scoringAngleRight - facingLeft)
                .lineToSplineHeading(new Pose2d(35, -12.5, facingLeft))
                .setTangent(facingLeft)
                .splineTo(new Vector2d(23.5, -12), facingLeft)
                .splineTo(parkingZone1, facingLeft)
                .build()
                ;

        TrajectorySequence parkMiddle = drivetrain.trajectorySequenceBuilder(trajectory1.end())
                .setTangent(scoringAngleRight - facingLeft)
                .lineToSplineHeading(new Pose2d(35, -12.5, facingLeft))
                .build()
                ;

        TrajectorySequence parkRight = drivetrain.trajectorySequenceBuilder(trajectory1.end())
                .setReversed(true)
                .splineTo(turnPos, facingRight)
                .splineTo(parkingZone3, facingRight)
                .build()
                ;


        drivetrain.followTrajectorySequenceAsync(trajectory1);

        hubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */

        scorer.clawIsOpen = false;
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
//            sleep(20);
            scorer.runClaw();
            scorer.runPivot();
            scorer.runPassThruServos();
            scorer.runPassThruStates();
        }

        //START IS HERE//

        dashboard.stopCameraStream();
        camera.stopStreaming();
        camera.closeCameraDevice();

        autonomousTimer.reset();

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


        while(opModeIsActive()) {

            for (LynxModule hub : hubs) {
                hub.clearBulkCache();
            }

            if(!hasParked && !drivetrain.isBusy() && (autonomousTimer.seconds() >= 3)) {

                if (tagOfInterest != null) {
                    if (tagOfInterest.id == LEFT) {
                        drivetrain.followTrajectorySequenceAsync(parkLeft);
                    } else if (tagOfInterest.id == RIGHT) {
                        drivetrain.followTrajectorySequenceAsync(parkRight);
                    } else {
                        drivetrain.followTrajectorySequenceAsync(parkMiddle);
                    }
                } else {
                    drivetrain.followTrajectorySequenceAsync(parkMiddle);
                }

                hasParked = true;
            }

            scorer.readLiftPos();
            scorer.runClaw();
            scorer.runPivot();
            scorer.runPassThruServos();
            scorer.runPassThruStates();
            scorer.runLiftToPos();
            drivetrain.update();

            //everything below is telemetry
            scorer.printTelemetry(myTelemetry);
            myTelemetry.update();

        }
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
    }
}