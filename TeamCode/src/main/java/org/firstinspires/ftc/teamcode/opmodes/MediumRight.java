package org.firstinspires.ftc.teamcode.opmodes;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.robot.PowerplayLift;

@Autonomous(name = "1+5 Medium Right", group = "21836 Autonomous")
public class MediumRight extends BaseAuton {

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode(PowerplayLift.Position.MED, Side.RIGHT);
    }
}
