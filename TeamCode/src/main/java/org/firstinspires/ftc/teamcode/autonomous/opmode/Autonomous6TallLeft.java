package org.firstinspires.ftc.teamcode.autonomous.opmode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name = "1+5 Tall Left", group = "21836 Autonomous")
public class Autonomous6TallLeft extends Autonomous6Tall {

    @Override
    public void runOpMode() throws InterruptedException {
        isRight = false;
        super.runOpMode();
    }
}
