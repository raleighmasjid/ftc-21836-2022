package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.PowerplayScorer;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


@Autonomous(name="2-Point Backup Auton", group = "21836 Autonomous")
public class TwoPointAuton extends LinearOpMode {
    PowerplayScorer scorer = new PowerplayScorer();

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(35, -62.5, Math.toRadians(90)));

        //  Initialize telemetry and dashboard
        MultipleTelemetry mytelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        FtcDashboard dashboard = FtcDashboard.getInstance();

        //  initializes code:
        scorer.init(hardwareMap);

        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(new Pose2d(35, -62.5, Math.toRadians(90)))
                .splineToConstantHeading(new Vector2d(30, -61), Math.toRadians(-180))
                .splineToConstantHeading(new Vector2d(0, -61), Math.toRadians(-180))
                .build()
                ;


        waitForStart();
        if (isStopRequested()) return;

        drive.followTrajectorySequence(traj1);

    }
}
