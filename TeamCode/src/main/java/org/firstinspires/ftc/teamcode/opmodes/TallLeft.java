package org.firstinspires.ftc.teamcode.opmodes;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.robot.Lift;

@Autonomous(group = "21836 Autonomous")
public class TallLeft extends BaseAuton {

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode(Lift.Position.TALL, Side.LEFT);
    }
}
