package org.firstinspires.ftc.teamcode.opmodes;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.robot.PowerplayLift;

@Autonomous(name = "1+5 Medium Left", group = "21836 Autonomous")
public class Med6Left extends BaseAuton {

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode(PowerplayLift.Position.MED, Side.LEFT);
    }
}
