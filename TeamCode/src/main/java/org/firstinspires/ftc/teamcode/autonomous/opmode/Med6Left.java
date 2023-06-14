package org.firstinspires.ftc.teamcode.autonomous.opmode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.robot.PowerplayScorer;

@Autonomous(name = "1+5 Medium Left", group = "21836 Autonomous")
public class Med6Left extends BaseAuton {

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode(PowerplayScorer.LiftPos.MED, Side.LEFT);
    }
}
