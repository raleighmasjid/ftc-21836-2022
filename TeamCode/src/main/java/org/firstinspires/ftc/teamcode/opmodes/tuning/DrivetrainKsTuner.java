package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.HeadingLockingMecanum;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrivetrain;

import java.util.List;

@Disabled
@TeleOp(name = "Drivetrain kS Tuner", group = "21836 Teleop")

public class DrivetrainKsTuner extends LinearOpMode {

    MecanumDrivetrain drivetrain;
    List<LynxModule> hubs;

    @Override
    public void runOpMode() throws InterruptedException {

        drivetrain = new MecanumDrivetrain(hardwareMap, 537.7, 312);

        hubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        waitForStart();

//      teleop control loop
        while (opModeIsActive()) {
            for (LynxModule hub : hubs) hub.clearBulkCache();
            drivetrain.run(0, 0, HeadingLockingMecanum.kS);
        }
    }
}
