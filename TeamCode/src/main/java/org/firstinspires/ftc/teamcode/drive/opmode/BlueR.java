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


@Autonomous(name="Blue Alliance - Right", group = "21836 Autonomous")
public class BlueR extends LinearOpMode {
    PowerplayScorer scorer = new PowerplayScorer();

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(35, -62.5, Math.toRadians(270)));

        //  Initialize telemetry and dashboard
        MultipleTelemetry mytelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        FtcDashboard dashboard = FtcDashboard.getInstance();

        //  initializes code:
        scorer.init(hardwareMap);

        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(new Pose2d(35, -62.5, Math.toRadians(180)))
                .lineTo(new Vector2d(35, -23.5))
                .waitSeconds(0.1)
                .lineTo(new Vector2d(30, -23.5))
                .waitSeconds(0.1)
                .lineTo(new Vector2d(35, -23.5))
                .lineTo(new Vector2d(35, -12.5))

                .setReversed(true)
                .lineTo(new Vector2d(61, -12.5))
                .waitSeconds(0.1)
                .setReversed(false)
                .splineTo(new Vector2d(40, -12.5), Math.toRadians(180))
                .splineTo(new Vector2d(28, -18.5), Math.toRadians(225))
                .waitSeconds(0.1)
                .setReversed(true)
                .splineTo(new Vector2d(40, -12.5), Math.toRadians(0))
                .splineTo(new Vector2d(61, -12.5), Math.toRadians(0))
                .build()
                ;


        waitForStart();
        if (isStopRequested()) return;

        drive.followTrajectorySequence(traj1);

    }
}
