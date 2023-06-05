package org.firstinspires.ftc.teamcode.autonomous.opmode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name = "1+5 Tall Left", group = "21836 Autonomous")
public class Tall6Left extends Tall6Right {

    @Override
    public void runOpMode() throws InterruptedException {
        isRight = false;
        super.runOpMode();
    }
}
