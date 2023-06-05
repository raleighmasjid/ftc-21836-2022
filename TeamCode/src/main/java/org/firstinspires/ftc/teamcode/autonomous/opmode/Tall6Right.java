package org.firstinspires.ftc.teamcode.autonomous.opmode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "1+5 Tall Right", group = "21836 Autonomous")
public class Tall6Right extends BaseAuton {

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode(ScoringPole.TALL, Side.RIGHT);
    }
}
