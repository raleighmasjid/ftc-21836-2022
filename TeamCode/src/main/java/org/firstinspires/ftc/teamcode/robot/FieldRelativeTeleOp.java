package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.control.HeadingHolder;

import java.util.List;


@TeleOp(name="Field Relative", group = "21836 Teleop")

public class FieldRelativeTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

//      Initialize telemetry and dashboard
        MultipleTelemetry myTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        FtcDashboard dashboard = FtcDashboard.getInstance();

        PowerplayScorer scorer = new PowerplayScorer();
        TeleOpMecanumDrive drivetrain = new TeleOpMecanumDrive();
        scorer.init(hardwareMap);
        drivetrain.init(hardwareMap);

        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);

//      initializes both gamepads:
        GamepadEx
                Gamepad1 = new GamepadEx(gamepad1),
                Gamepad2 = new GamepadEx(gamepad2);

        ButtonReader
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

                control2Up = new ButtonReader(Gamepad2, GamepadKeys.Button.DPAD_UP),
                control2Left = new ButtonReader(Gamepad2, GamepadKeys.Button.DPAD_LEFT),
                control2Right = new ButtonReader(Gamepad2, GamepadKeys.Button.DPAD_RIGHT),
                control2Down = new ButtonReader(Gamepad2, GamepadKeys.Button.DPAD_DOWN);

        boolean
                overrideMode = false;

        drivetrain.setRotation(HeadingHolder.getHeading());

        for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        waitForStart();

//      teleop control loop
        while (opModeIsActive()) {

            for (LynxModule hub : hubs) hub.clearBulkCache();

            // Get button reader states
            control2A.readValue();
            control2B.readValue();
            control2X.readValue();
            control2Y.readValue();

            control2LShoulder.readValue();
            control2RShoulder.readValue();

            control1Up.readValue();
            control1Left.readValue();
            control1Right.readValue();
            control1Down.readValue();

            control2Up.readValue();
            control2Left.readValue();
            control2Right.readValue();
            control2Down.readValue();

            scorer.readLiftPos();

            // Field-centric resets
            if (control1Up.wasJustPressed()) drivetrain.resetRotation();
            else if (control1Left.wasJustPressed()) drivetrain.setRotation(90.0);
            else if (control1Down.wasJustPressed()) drivetrain.setRotation(180.0);
            else if (control1Right.wasJustPressed()) drivetrain.setRotation(270.0);

            // Precision mode driving triggers
            double precisionScale = Gamepad1.isDown(GamepadKeys.Button.RIGHT_BUMPER)?
                    RobotConfig.PRECISION_MODE_SCALE:
                    (RobotConfig.PRECISION_MODE_SCALE - 1) * Gamepad1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) + 1,

                    control1LeftX = Gamepad1.getLeftX() * precisionScale,
                    control1LeftY = Gamepad1.getLeftY() * precisionScale,
                    control1RightX = Gamepad1.getRightX() * precisionScale;

            if (control2RShoulder.wasJustPressed()) {
                if (overrideMode) scorer.setLiftStateToCurrent();
                overrideMode = !overrideMode;
            }

            boolean stackHeights = control2LShoulder.isDown();

            if (overrideMode) {
                if (control2LShoulder.wasJustPressed()) scorer.resetLift();

                if (control2B.wasJustPressed()) scorer.toggleClaw();

                if (control2X.wasJustPressed()) scorer.togglePivot();

                if (control2Y.wasJustPressed()) scorer.togglePassThru();

                if (control2Up.wasJustPressed()) scorer.toggleClawTilt();

                scorer.runLift(Gamepad2.getLeftY());
            } else {
                if (control2Up.wasJustPressed()) scorer.setTargetLiftPos(stackHeights? PowerplayScorer.liftPos.FIVE: PowerplayScorer.liftPos.TALL);
                else if (control2Left.wasJustPressed()) scorer.setTargetLiftPos(stackHeights? PowerplayScorer.liftPos.FOUR: PowerplayScorer.liftPos.MED);
                else if (control2Right.wasJustPressed()) scorer.setTargetLiftPos(stackHeights? PowerplayScorer.liftPos.THREE: PowerplayScorer.liftPos.LOW);
                else if (control2Down.wasJustPressed()) scorer.setTargetLiftPos(stackHeights? PowerplayScorer.liftPos.TWO: PowerplayScorer.liftPos.FLOOR);

                if (control2Y.wasJustPressed()) scorer.triggerPassThru();

                if (control2B.wasJustPressed()) scorer.triggerClaw();

                scorer.runLiftToPos();
                scorer.runPassThruStates();
            }

            scorer.runClaw();
            scorer.runPivot();
            scorer.runPassThruServos();
            scorer.runConeArms(control2A.isDown());
            drivetrain.driveFieldCentric(control1LeftX, control1LeftY, control1RightX);

            //everything below is telemetry
            myTelemetry.addData(
                    "Robot is in", overrideMode?
                            "manual override mode": // override
                            stackHeights? // automated
                                    "stack heights mode":
                                    "junction heights mode"
            );
            myTelemetry.addLine();
            scorer.printTelemetry(myTelemetry);
            myTelemetry.update();
        }
    }
}
