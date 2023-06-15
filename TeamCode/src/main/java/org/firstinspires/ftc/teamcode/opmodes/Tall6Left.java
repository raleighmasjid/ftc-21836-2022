package org.firstinspires.ftc.teamcode.opmodes;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.robot.PowerplayScorer;

@Autonomous(name = "1+5 Tall Left", group = "21836 Autonomous")
public class Tall6Left extends BaseAuton {

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode(PowerplayScorer.LiftPos.TALL, Side.LEFT);
    }
}