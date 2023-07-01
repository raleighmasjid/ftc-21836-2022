package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.PowerplayScorer;
import org.firstinspires.ftc.teamcode.robot.RobotConfig;

import java.util.List;

@Config
@TeleOp(name = "Lift kS Tuner", group = "21836 Teleop")

public class LiftStaticFeedforwardTuning extends LinearOpMode {

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
            scorer.lift.run(RobotConfig.LIFT_kS, true);
        }
    }
}
