package org.firstinspires.ftc.teamcode.teleop;

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

import java.util.List;


@TeleOp(name="Field Relative", group = "21836 Teleop")

public class FieldRelativeTeleOp extends LinearOpMode {

    PowerplayScorer scorer = new PowerplayScorer();
    TeleOpMecanumDrive drivetrain = new TeleOpMecanumDrive();
    List<LynxModule> hubs;

    @Override
    public void runOpMode() throws InterruptedException {

//      Initialize telemetry and dashboard
        MultipleTelemetry myTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        FtcDashboard dashboard = FtcDashboard.getInstance();

        scorer.init(hardwareMap);
        drivetrain.init(hardwareMap);

        hubs = hardwareMap.getAll(LynxModule.class);

//      initializes both gamepads:
        GamepadEx Gamepad1 = new GamepadEx(gamepad1);
        GamepadEx Gamepad2 = new GamepadEx(gamepad2);

        ButtonReader control2A = new ButtonReader(Gamepad2, GamepadKeys.Button.A); // cone-flipping arms
        ButtonReader control2B = new ButtonReader(Gamepad2, GamepadKeys.Button.B); // claw
        ButtonReader control2X = new ButtonReader(Gamepad2, GamepadKeys.Button.X); // pivot
        ButtonReader control2Y = new ButtonReader(Gamepad2, GamepadKeys.Button.Y); // passthrough

        ButtonReader control2LShoulder = new ButtonReader(Gamepad2, GamepadKeys.Button.LEFT_BUMPER); // stack heights / lift reset
        ButtonReader control2RShoulder = new ButtonReader(Gamepad2, GamepadKeys.Button.RIGHT_BUMPER); // override-automated mode

        ButtonReader control1Up = new ButtonReader(Gamepad1, GamepadKeys.Button.DPAD_UP);
        ButtonReader control1Left = new ButtonReader(Gamepad1, GamepadKeys.Button.DPAD_LEFT);
        ButtonReader control1Right = new ButtonReader(Gamepad1, GamepadKeys.Button.DPAD_RIGHT);
        ButtonReader control1Down = new ButtonReader(Gamepad1, GamepadKeys.Button.DPAD_DOWN);

        ButtonReader control2Up = new ButtonReader(Gamepad2, GamepadKeys.Button.DPAD_UP);
        ButtonReader control2Left = new ButtonReader(Gamepad2, GamepadKeys.Button.DPAD_LEFT);
        ButtonReader control2Right = new ButtonReader(Gamepad2, GamepadKeys.Button.DPAD_RIGHT);
        ButtonReader control2Down = new ButtonReader(Gamepad2, GamepadKeys.Button.DPAD_DOWN);

        double control1LeftY;
        double control1LeftX;
        double control1RightX;
        double control2LeftY;

        double precisionScale;
        boolean useOverrideMode = false;
        
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

            control1LeftY = Gamepad1.getLeftY();
            control1LeftX = Gamepad1.getLeftX();
            control1RightX = Gamepad1.getRightX();

            control2LeftY = Gamepad2.getLeftY();

            scorer.readLiftPos();

            // Field-centric reset
            if (control1Up.wasJustPressed()) drivetrain.resetRotation();
            else if (control1Left.wasJustPressed()) drivetrain.setRotation(90.0);
            else if (control1Down.wasJustPressed()) drivetrain.setRotation(180.0);
            else if (control1Right.wasJustPressed()) drivetrain.setRotation(270.0);

            // Precision mode driving triggers
            precisionScale = (Gamepad1.isDown(GamepadKeys.Button.RIGHT_BUMPER)) ?
                    RobotConfig.PRECISION_MODE_SCALE :
                    (RobotConfig.PRECISION_MODE_SCALE - 1) * Gamepad1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) + 1
            ;

            control1LeftX *= precisionScale;
            control1LeftY *= precisionScale;
            control1RightX *= precisionScale;

            if (control2RShoulder.wasJustPressed()) {
                if (useOverrideMode) scorer.setTargetLiftPos(scorer.getCurrentLiftPos());
                useOverrideMode = !useOverrideMode;
                scorer.useLiftPIDF = !scorer.useLiftPIDF;
            }

            if (useOverrideMode) {
                if (control2LShoulder.wasJustPressed()) scorer.resetLift();

                if (control2B.wasJustPressed()) scorer.toggleClaw();

                if (control2X.wasJustPressed()) scorer.togglePivot();

                if (control2Y.wasJustPressed()) scorer.togglePassThru();

                scorer.runLift(control2LeftY);
            } else {
                if (control2LShoulder.isDown()) {
                    // Lift stack height triggers
                    if (control2Up.wasJustPressed()) scorer.setTargetLiftPos(PowerplayScorer.liftPos.FIVE);
                    else if (control2Left.wasJustPressed()) scorer.setTargetLiftPos(PowerplayScorer.liftPos.FOUR);
                    else if (control2Right.wasJustPressed()) scorer.setTargetLiftPos(PowerplayScorer.liftPos.THREE);
                    else if (control2Down.wasJustPressed()) scorer.setTargetLiftPos(PowerplayScorer.liftPos.TWO);
                } else {
                    // Lift junction height triggers
                    if (control2Up.wasJustPressed()) scorer.setTargetLiftPos(PowerplayScorer.liftPos.TALL);
                    else if (control2Left.wasJustPressed()) scorer.setTargetLiftPos(PowerplayScorer.liftPos.MED);
                    else if (control2Right.wasJustPressed()) scorer.setTargetLiftPos(PowerplayScorer.liftPos.LOW);
                    else if (control2Down.wasJustPressed()) scorer.setTargetLiftPos(PowerplayScorer.liftPos.FLOOR);
                }

                if (control2Y.wasJustPressed()) scorer.triggerPassThru();

                if (control2B.wasJustPressed()) {
                    if (scorer.clawIsOpen) scorer.liftClaw();
                    else scorer.dropClaw();
                }

                scorer.runLiftToPos();
                scorer.runPassThruStates();
            }

            scorer.runClaw();
            scorer.runPivot();
            scorer.runPassThruServos();
            scorer.runConeArms(control2A.isDown());
            drivetrain.driveFieldCentric(control1LeftX, control1LeftY, control1RightX);

            //everything below is telemetry
            if (useOverrideMode) {
                myTelemetry.addData("Robot is in", "manual override mode");
                scorer.LED1green.setState(false);
                scorer.LED2green.setState(false);
                scorer.LED1red.setState(true);
                scorer.LED2red.setState(true);
            } else {
                if (control2LShoulder.isDown()) myTelemetry.addData("Robot is in", "stack heights mode");
                else                            myTelemetry.addData("Robot is in", "junction heights mode");
                scorer.LED1green.setState(true);
                scorer.LED2green.setState(true);
                scorer.LED1red.setState(false);
                scorer.LED2red.setState(false);
            }
            myTelemetry.addLine();
            scorer.printTelemetry(myTelemetry);
            myTelemetry.update();
        }
    }
}
