package org.firstinspires.ftc.teamcode.autonomous.opmode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.robot.PowerplayLift;

@Autonomous(name = "1+5 Tall Left", group = "21836 Autonomous")
public class Tall6Left extends BaseAuton {

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode(PowerplayLift.Position.TALL, Side.LEFT);
    }
}
