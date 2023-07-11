package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Lift;

import java.util.List;

@TeleOp(name = "Lift kS Tuner", group = "21836 Teleop")

public class LiftKsTuner extends LinearOpMode {

    Lift lift;
    List<LynxModule> hubs;

    @Override
    public void runOpMode() throws InterruptedException {

        lift = new Lift(hardwareMap);

        hubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        waitForStart();

//      teleop control loop
        while (opModeIsActive()) {
            for (LynxModule hub : hubs) hub.clearBulkCache();
            lift.run(Lift.kS, true);
        }
    }
}
