package org.firstinspires.ftc.teamcode.autonomous.opmode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name = "1+5 Medium Left", group = "21836 Autonomous")
public class Med6Left extends Med6Right {

    @Override
    public void runOpMode() throws InterruptedException {
        isRight = false;
        super.runOpMode();
    }
}
