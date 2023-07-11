package org.firstinspires.ftc.teamcode.opmodes;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.robot.Lift;

@Autonomous(group = "21836 Autonomous")
public class MediumRight extends BaseAuton {

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode(Lift.Position.MED, Side.RIGHT);
    }
}
