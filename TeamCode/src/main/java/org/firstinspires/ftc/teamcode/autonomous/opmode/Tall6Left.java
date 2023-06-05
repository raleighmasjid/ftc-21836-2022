package org.firstinspires.ftc.teamcode.autonomous.opmode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "1+5 Tall Left", group = "21836 Autonomous")
public class Tall6Left extends BaseAuton {

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode(ScoringPole.TALL, Side.LEFT);
    }
}
