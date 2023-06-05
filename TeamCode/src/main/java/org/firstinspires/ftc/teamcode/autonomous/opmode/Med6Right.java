package org.firstinspires.ftc.teamcode.autonomous.opmode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "1+5 Medium Right", group = "21836 Autonomous")
public class Med6Right extends BaseAuton {

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode(ScoringPole.MED, Side.RIGHT);
    }
}
