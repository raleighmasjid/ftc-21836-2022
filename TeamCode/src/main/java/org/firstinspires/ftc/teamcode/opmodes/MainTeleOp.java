package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.control.HeadingHolder;
import org.firstinspires.ftc.teamcode.robot.PowerplayLift;
import org.firstinspires.ftc.teamcode.robot.PowerplayPassthrough;
import org.firstinspires.ftc.teamcode.robot.PowerplayScorer;
import org.firstinspires.ftc.teamcode.subsystems.PositionLockingMecanum;

import java.util.List;

@Config
@TeleOp(name = "Main TeleOp", group = "21836 Teleop")

public class MainTeleOp extends LinearOpMode {

    MultipleTelemetry myTelemetry;
    PowerplayScorer scorer;
    PositionLockingMecanum drivetrain;
    List<LynxModule> hubs;
    GamepadEx Gamepad1, Gamepad2;

    public static double DRIVETRAIN_PRECISION_MODE_SCALE = 0.3;
    public static double PASSTHROUGH_CONTROL_SCALE = 10.0;

    @Override
    public void runOpMode() throws InterruptedException {

//      Initialize telemetry
        myTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        scorer = new PowerplayScorer(hardwareMap);
        drivetrain = new PositionLockingMecanum(hardwareMap, 537.7, 312);

        hubs = hardwareMap.getAll(LynxModule.class);

//      initializes both gamepads:
        Gamepad1 = new GamepadEx(gamepad1);
        Gamepad2 = new GamepadEx(gamepad2);

        ButtonReader
                control2Up = new ButtonReader(Gamepad2, GamepadKeys.Button.DPAD_UP),
                control2Left = new ButtonReader(Gamepad2, GamepadKeys.Button.DPAD_LEFT),
                control2Right = new ButtonReader(Gamepad2, GamepadKeys.Button.DPAD_RIGHT),
                control2Down = new ButtonReader(Gamepad2, GamepadKeys.Button.DPAD_DOWN),

                control2A = new ButtonReader(Gamepad2, GamepadKeys.Button.A), // cone-flipping arms
                control2B = new ButtonReader(Gamepad2, GamepadKeys.Button.B), // claw
                control2X = new ButtonReader(Gamepad2, GamepadKeys.Button.X), // pivot
                control2Y = new ButtonReader(Gamepad2, GamepadKeys.Button.Y), // passthrough

                control2LShoulder = new ButtonReader(Gamepad2, GamepadKeys.Button.LEFT_BUMPER), // stack heights / lift reset
                control2RShoulder = new ButtonReader(Gamepad2, GamepadKeys.Button.RIGHT_BUMPER), // override-automated mode

                control1Up = new ButtonReader(Gamepad1, GamepadKeys.Button.DPAD_UP),
                control1Left = new ButtonReader(Gamepad1, GamepadKeys.Button.DPAD_LEFT),
                control1Right = new ButtonReader(Gamepad1, GamepadKeys.Button.DPAD_RIGHT),
                control1Down = new ButtonReader(Gamepad1, GamepadKeys.Button.DPAD_DOWN),

                control1A = new ButtonReader(Gamepad2, GamepadKeys.Button.A), // 270
                control1X = new ButtonReader(Gamepad2, GamepadKeys.Button.X), // 180
                control1B = new ButtonReader(Gamepad2, GamepadKeys.Button.B), // 90
                control1Y = new ButtonReader(Gamepad2, GamepadKeys.Button.Y); // 0

        ButtonReader[] buttonReaders = {
                control2Up, control2Left, control2Right, control2Down,
                control2A, control2B, control2X, control2Y,
                control2LShoulder, control2RShoulder,
                control1Up, control1Left, control1Right, control1Down,
                control1A, control1X, control1B, control1Y
        };

        boolean overrideMode = false;
        double manualPassthroughAngle = PowerplayPassthrough.ANGLE_PASS_FRONT;

        drivetrain.setCurrentHeading(HeadingHolder.getHeading());

        for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        waitForStart();

//      teleop control loop
        while (opModeIsActive()) {

            for (LynxModule hub : hubs) hub.clearBulkCache();

            // Get button reader states
            for (ButtonReader buttonReader : buttonReaders) buttonReader.readValue();

            scorer.lift.readPosition();
            drivetrain.readIMU();

            // Field-centric resets
            if (control1Up.wasJustPressed()) drivetrain.setCurrentHeading(0.0);
            else if (control1Left.wasJustPressed()) drivetrain.setCurrentHeading(90.0);
            else if (control1Down.wasJustPressed()) drivetrain.setCurrentHeading(180.0);
            else if (control1Right.wasJustPressed()) drivetrain.setCurrentHeading(270.0);

            // Auto-turn
            if (control1Y.wasJustPressed()) drivetrain.setTargetHeading(0.0);
            else if (control1X.wasJustPressed()) drivetrain.setTargetHeading(90.0);
            else if (control1A.wasJustPressed()) drivetrain.setTargetHeading(180.0);
            else if (control1B.wasJustPressed()) drivetrain.setTargetHeading(270.0);

            // Precision mode driving triggers
            double precisionScale = Gamepad1.isDown(GamepadKeys.Button.RIGHT_BUMPER) ?
                    DRIVETRAIN_PRECISION_MODE_SCALE :
                    (DRIVETRAIN_PRECISION_MODE_SCALE - 1) * Gamepad1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) + 1;

            boolean stackHeights = control2LShoulder.isDown();

            if (control2RShoulder.wasJustPressed()) {
                scorer.lift.setTargetPosition(scorer.lift.getCurrentPosition());
                overrideMode = !overrideMode;
            }

            if (overrideMode) {

                if (control2LShoulder.wasJustPressed()) scorer.reset();

                if (control2A.wasJustPressed()) scorer.passthrough.toggleTilt();

                if (control2B.wasJustPressed()) scorer.passthrough.claw.toggle();

                if (control2X.wasJustPressed()) scorer.passthrough.wrist.toggle();

                if (control2Y.wasJustPressed()) scorer.passthrough.toggle();

                scorer.lift.run(Gamepad2.getLeftY(), true);
                manualPassthroughAngle = Range.clip(
                        manualPassthroughAngle + Gamepad2.getRightX() * PASSTHROUGH_CONTROL_SCALE,
                        PowerplayPassthrough.ANGLE_PASS_FRONT,
                        PowerplayPassthrough.ANGLE_PASS_BACK
                );
                scorer.passthrough.run(manualPassthroughAngle);
            } else {

                if (control2Up.wasJustPressed()) {
                    scorer.setTargetLiftPos(stackHeights ?
                            PowerplayLift.Position.FIVE :
                            PowerplayLift.Position.TALL
                    );
                } else if (control2Left.wasJustPressed()) {
                    scorer.setTargetLiftPos(stackHeights ?
                            PowerplayLift.Position.FOUR :
                            PowerplayLift.Position.MED
                    );
                } else if (control2Right.wasJustPressed()) {
                    scorer.setTargetLiftPos(stackHeights ?
                            PowerplayLift.Position.THREE :
                            PowerplayLift.Position.LOW
                    );
                } else if (control2Down.wasJustPressed()) {
                    scorer.setTargetLiftPos(stackHeights ?
                            PowerplayLift.Position.TWO :
                            PowerplayLift.Position.FLOOR
                    );
                }

                if (control2Y.wasJustPressed()) scorer.passthrough.trigger();

                if (control2B.wasJustPressed()) scorer.triggerClaw();

                if (control2X.wasJustPressed()) scorer.toggleFloorPickup();

                scorer.lift.runToPosition();
                scorer.passthrough.run();
            }

            scorer.run(
                    Gamepad2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) * PowerplayScorer.ANGLE_CONE_ARMS_DOWN,
                    Gamepad2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) * PowerplayScorer.ANGLE_CONE_ARMS_DOWN
            );
            drivetrain.run(
                    Gamepad1.getLeftX() * precisionScale,
                    Gamepad1.getLeftY() * precisionScale,
                    Gamepad1.getRightX() * precisionScale
            );

            // everything below is telemetry
            myTelemetry.addData(
                    "Robot is in", overrideMode ?
                            "manual override mode" : // override
                            stackHeights ? // automated
                                    "stack heights mode" :
                                    "junction heights mode"
            );
            myTelemetry.addLine();
            scorer.printTelemetry(myTelemetry);
            myTelemetry.addLine();
            myTelemetry.addLine();
            drivetrain.printNumericalTelemetry(myTelemetry);
            myTelemetry.addLine();
            myTelemetry.addLine();
            scorer.printNumericalTelemetry(myTelemetry);
            myTelemetry.update();
        }
    }
}
