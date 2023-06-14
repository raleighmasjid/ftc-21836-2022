package org.firstinspires.ftc.teamcode.autonomous.opmode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.robot.PowerplayLift;

@Autonomous(name = "1+5 Tall Right", group = "21836 Autonomous")
public class Tall6Right extends BaseAuton {

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode(PowerplayLift.Position.TALL, Side.RIGHT);
    }
}
