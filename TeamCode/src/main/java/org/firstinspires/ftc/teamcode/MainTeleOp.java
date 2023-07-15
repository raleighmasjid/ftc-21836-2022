package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.List;

@TeleOp(group = "21836 Teleop")

public class MainTeleOp extends LinearOpMode {

    MultipleTelemetry mTelemetry;
    List<LynxModule> hubs;
    GamepadEx Gamepad1, Gamepad2;

    @Override
    public void runOpMode() throws InterruptedException {

        hubs = hardwareMap.getAll(LynxModule.class);
        mTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

//      initializes both gamepads:
        Gamepad1 = new GamepadEx(gamepad1);
        Gamepad2 = new GamepadEx(gamepad2);

        for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        waitForStart();

//      teleop control loop
        while (opModeIsActive()) {
            for (LynxModule hub : hubs) hub.clearBulkCache();

            mTelemetry.addLine("hello");
            mTelemetry.update();
        }
    }
}
