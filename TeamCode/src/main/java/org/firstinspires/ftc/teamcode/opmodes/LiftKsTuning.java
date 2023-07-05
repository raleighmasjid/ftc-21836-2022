package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.PowerplayLift;
import org.firstinspires.ftc.teamcode.robot.PowerplayScorer;

import java.util.List;

@TeleOp(name = "Lift kS Tuner", group = "21836 Teleop")

public class LiftKsTuning extends LinearOpMode {

    PowerplayScorer scorer;
    List<LynxModule> hubs;

    @Override
    public void runOpMode() throws InterruptedException {

        scorer = new PowerplayScorer(hardwareMap);

        hubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        waitForStart();

//      teleop control loop
        while (opModeIsActive()) {
            for (LynxModule hub : hubs) hub.clearBulkCache();
            scorer.lift.run(PowerplayLift.kS, true);
        }
    }
}
