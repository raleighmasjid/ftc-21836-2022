package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrivetrain;

import java.util.List;

@TeleOp(group = "21836 Teleop")

public class MainTeleOp extends LinearOpMode {

    MultipleTelemetry mTelemetry;
    List<LynxModule> hubs;
    GamepadEx Gamepad1, Gamepad2;
    MecanumDrivetrain drivetrain;
    Lift lift;

    @Override
    public void runOpMode() throws InterruptedException {

        hubs = hardwareMap.getAll(LynxModule.class);
        mTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drivetrain = new MecanumDrivetrain(hardwareMap);
        lift = new Lift(hardwareMap);

//      initializes both gamepads:
        Gamepad1 = new GamepadEx(gamepad1);
        Gamepad2 = new GamepadEx(gamepad2);

        ButtonReader
                control2Up = new ButtonReader(Gamepad2, GamepadKeys.Button.DPAD_UP),
                control2Down = new ButtonReader(Gamepad2, GamepadKeys.Button.DPAD_DOWN);

        ButtonReader[] buttonReaders = {control2Up, control2Down};

        for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        waitForStart();

//      teleop control loop
        while (opModeIsActive()) {
            for (LynxModule hub : hubs) hub.clearBulkCache();

            for (ButtonReader button : buttonReaders) button.readValue();

            if (control2Up.wasJustPressed()) lift.setTargetPosition(30);
            if (control2Down.wasJustPressed()) lift.setTargetPosition(0);
            lift.runToPosition();

            drivetrain.drive(Gamepad1.getLeftX(), Gamepad1.getLeftY(), Gamepad1.getRightX());

            mTelemetry.addLine("hello");
            mTelemetry.update();
        }
    }
}
