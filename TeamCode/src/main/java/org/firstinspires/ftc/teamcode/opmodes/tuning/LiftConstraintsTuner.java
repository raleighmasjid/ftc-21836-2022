package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Lift;

import java.util.List;

@TeleOp(group = "21836 Teleop")

public class LiftConstraintsTuner extends LinearOpMode {

    public static double REPEATS = 5;

    MultipleTelemetry myTelemetry;
    Lift lift;
    List<LynxModule> hubs;

    @Override
    public void runOpMode() throws InterruptedException {

        lift = new Lift(hardwareMap);
        myTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        hubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        double[] maxVelocities = new double[(int) REPEATS];
        double[] maxAccelerations = new double[(int) REPEATS];

        waitForStart();

        for (int i = 0; i < REPEATS; i++) {

            while (lift.getCurrentPosition() <= Lift.HEIGHT_1_STAGE * 3) {
                for (LynxModule hub : hubs) hub.clearBulkCache();
                lift.readPosition();
                lift.run(1, true);
            }
            while (lift.getCurrentPosition() >= Lift.TOLERANCE) {
                for (LynxModule hub : hubs) hub.clearBulkCache();
                lift.readPosition();
                lift.run(-0.2, true);
            }

            maxVelocities[i] = lift.getMaxVelocity();
            maxAccelerations[i] = lift.getMaxAcceleration();
            lift.reset();
        }

        double maxVelocity = 0.0;
        for (double velo : maxVelocities) maxVelocity += velo;
        maxVelocity /= REPEATS;

        double maxAcceleration = 0.0;
        for (double accel : maxAccelerations) maxAcceleration += accel;
        maxAcceleration /= REPEATS;

        myTelemetry.addData("Max velocity (in/s)", maxVelocity);
        myTelemetry.addData("Max acceleration (in/s^2)", maxAcceleration);
        myTelemetry.update();
    }
}
