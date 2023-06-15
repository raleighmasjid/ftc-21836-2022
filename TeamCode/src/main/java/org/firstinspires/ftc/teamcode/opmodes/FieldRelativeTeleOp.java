package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.control.HeadingHolder;
import org.firstinspires.ftc.teamcode.robot.PowerplayScorer;
import org.firstinspires.ftc.teamcode.robot.RobotConfig;
import org.firstinspires.ftc.teamcode.systems.TeleOpMecanumDrive;

import java.util.List;


@TeleOp(name = "Field Relative", group = "21836 Teleop")

public class FieldRelativeTeleOp extends LinearOpMode {

    MultipleTelemetry myTelemetry;
    PowerplayScorer scorer;
    TeleOpMecanumDrive drivetrain;
    List<LynxModule> hubs;
    GamepadEx Gamepad1, Gamepad2;

    @Override
    public void runOpMode() throws InterruptedException {

//      Initialize telemetry
        myTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        scorer = new PowerplayScorer(hardwareMap);
        drivetrain = new TeleOpMecanumDrive(hardwareMap);

        hubs = hardwareMap.getAll(LynxModule.class);

//      initializes both gamepads:
        Gamepad1 = new GamepadEx(gamepad1);
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

        boolean overrideMode = false;

        drivetrain.setHeading(HeadingHolder.getHeading());

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

            scorer.updateValues();
            scorer.lift.readPosition();
            drivetrain.readIMU();

            // Field-centric resets
            if (control1Up.wasJustPressed()) drivetrain.setHeading(0.0);
            else if (control1Left.wasJustPressed()) drivetrain.setHeading(90.0);
            else if (control1Down.wasJustPressed()) drivetrain.setHeading(180.0);
            else if (control1Right.wasJustPressed()) drivetrain.setHeading(270.0);

            // Precision mode driving triggers
            double precisionScale = Gamepad1.isDown(GamepadKeys.Button.RIGHT_BUMPER) ?
                    RobotConfig.SCALE_PRECISION_MODE :
                    (RobotConfig.SCALE_PRECISION_MODE - 1) * Gamepad1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) + 1;

            boolean stackHeights = control2LShoulder.isDown();

            if (control2RShoulder.wasJustPressed()) {
                scorer.lift.setTargetPosition(scorer.lift.getCurrentPosition());
                overrideMode = !overrideMode;
            }

            if (overrideMode) {
                if (control2LShoulder.wasJustPressed()) scorer.resetLift();

                if (control2B.wasJustPressed()) scorer.passthrough.claw.toggle();

                if (control2X.wasJustPressed()) scorer.passthrough.togglePivot();

                if (control2Y.wasJustPressed()) scorer.passthrough.toggle();

                if (control2Up.wasJustPressed()) scorer.passthrough.toggleTilt();

                scorer.lift.run(Gamepad2.getLeftY(), true);
            } else {
                if (control2Up.wasJustPressed())
                    scorer.setTargetLiftPos(stackHeights ? PowerplayScorer.LiftPos.FIVE : PowerplayScorer.LiftPos.TALL);
                else if (control2Left.wasJustPressed())
                    scorer.setTargetLiftPos(stackHeights ? PowerplayScorer.LiftPos.FOUR : PowerplayScorer.LiftPos.MED);
                else if (control2Right.wasJustPressed())
                    scorer.setTargetLiftPos(stackHeights ? PowerplayScorer.LiftPos.THREE : PowerplayScorer.LiftPos.LOW);
                else if (control2Down.wasJustPressed())
                    scorer.setTargetLiftPos(stackHeights ? PowerplayScorer.LiftPos.TWO : PowerplayScorer.LiftPos.FLOOR);

                if (control2Y.wasJustPressed()) scorer.passthrough.trigger();

                if (control2B.wasJustPressed()) scorer.triggerClaw();

                scorer.lift.runToPosition();
            }

            scorer.passthrough.run();
            scorer.passthrough.runPivot();
            scorer.passthrough.claw.run();
            scorer.run(
                    Gamepad2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) * RobotConfig.ANGLE_ARMS_DOWN,
                    Gamepad2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) * RobotConfig.ANGLE_ARMS_DOWN
            );
            drivetrain.driveFieldCentric(
                    Gamepad1.getLeftX() * precisionScale,
                    Gamepad1.getLeftY() * precisionScale,
                    Gamepad1.getRightX() * precisionScale
            );

            //everything below is telemetry
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
            myTelemetry.addData("Robot heading", drivetrain.getHeading());
            myTelemetry.addLine();
            scorer.printNumericalTelemetry(myTelemetry);
            myTelemetry.update();
        }
    }
}
