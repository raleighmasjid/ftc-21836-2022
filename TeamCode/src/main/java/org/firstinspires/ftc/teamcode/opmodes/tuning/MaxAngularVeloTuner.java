package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.control.filters.FIRLowPassFilter;
import org.firstinspires.ftc.teamcode.subsystems.AutonMecanumDrivetrain;

import java.util.Objects;

/**
 * This routine is designed to calculate the maximum angular velocity your bot can achieve under load.
 * <p>
 * Upon pressing start, your bot will turn at max power for RUNTIME seconds.
 * <p>
 * Further fine tuning of MAX_ANG_VEL may be desired.
 */

@Config
@Autonomous(group = "drive")
public class MaxAngularVeloTuner extends LinearOpMode {
    public static double RUNTIME = 4.0;

    private ElapsedTime timer, accelTimer;
    private double maxAngVelocity = 0.0, maxAngAcceleration = 0.0;

    private FIRLowPassFilter accelFilter = new FIRLowPassFilter();

    @Override
    public void runOpMode() throws InterruptedException {
        AutonMecanumDrivetrain drive = new AutonMecanumDrivetrain(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addLine("Your bot will turn at full speed for " + RUNTIME + " seconds.");
        telemetry.addLine("Please ensure you have enough space cleared.");
        telemetry.addLine("");
        telemetry.addLine("Press start when ready.");
        telemetry.update();

        waitForStart();

        telemetry.clearAll();
        telemetry.update();

        drive.setDrivePower(new Pose2d(0, 0, 1));
        timer = new ElapsedTime();

        while (!isStopRequested() && timer.seconds() < RUNTIME) {
            drive.updatePoseEstimate();

            Pose2d poseVelo = Objects.requireNonNull(drive.getPoseVelocity(), "poseVelocity() must not be null. Ensure that the getWheelVelocities() method has been overridden in your localizer.");

            double lastMaxAngVelocity = maxAngVelocity;
            maxAngVelocity = Math.max(poseVelo.getHeading(), maxAngVelocity);
            maxAngAcceleration = Math.max(accelFilter.getEstimate((maxAngVelocity - lastMaxAngVelocity) / accelTimer.seconds()), maxAngAcceleration);
            accelTimer.reset();
        }

        drive.setDrivePower(new Pose2d());

        telemetry.addData("Max Angular Velocity (deg)", Math.toDegrees(maxAngVelocity));
        telemetry.addData("Max Recommended Angular Velocity (deg)", Math.toDegrees(maxAngVelocity * 0.9));
        telemetry.addData("Max Angular Acceleration (deg)", Math.toDegrees(maxAngAcceleration));
        telemetry.addData("Max Recommended Angular Acceleration (deg)", Math.toDegrees(maxAngAcceleration * 0.9));
        telemetry.update();

        while (!isStopRequested()) idle();
    }
}
