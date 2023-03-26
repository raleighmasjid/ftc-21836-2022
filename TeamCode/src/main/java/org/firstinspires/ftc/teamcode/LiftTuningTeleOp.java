package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.List;


@TeleOp(name="Lift Tuning", group = "21836 Testing")

public class LiftTuningTeleOp extends LinearOpMode {

    PowerplayScorer scorer = new PowerplayScorer();
    List<LynxModule> hubs;

    @Override
    public void runOpMode() throws InterruptedException {

//      Initialize telemetry and dashboard
        MultipleTelemetry myTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        FtcDashboard dashboard = FtcDashboard.getInstance();

        scorer.init(hardwareMap);

        hubs = hardwareMap.getAll(LynxModule.class);

//      Initialize gamepad:
        GamepadEx Gamepad2 = new GamepadEx(gamepad2);

        ButtonReader control2X = new ButtonReader(Gamepad2, GamepadKeys.Button.X); // override-automated mode

        ButtonReader control2RShoulder = new ButtonReader(Gamepad2, GamepadKeys.Button.RIGHT_BUMPER);

        ButtonReader control2Up = new ButtonReader(Gamepad2, GamepadKeys.Button.DPAD_UP);
        ButtonReader control2Left = new ButtonReader(Gamepad2, GamepadKeys.Button.DPAD_LEFT);
        ButtonReader control2Right = new ButtonReader(Gamepad2, GamepadKeys.Button.DPAD_RIGHT);
        ButtonReader control2Down = new ButtonReader(Gamepad2, GamepadKeys.Button.DPAD_DOWN);

        double control2LeftY;
        boolean useOverrideMode = false;
        double targetLiftPos = TeleOpConfig.HEIGHT_TALL;
        boolean loop = true;

        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        waitForStart();

//      teleop control loop
        while (opModeIsActive()) {

            for (LynxModule hub : hubs) {
                hub.clearBulkCache();
            }

            // Get button reader states
            control2X.readValue();

            control2RShoulder.readValue();

            control2Up.readValue();
            control2Left.readValue();
            control2Right.readValue();
            control2Down.readValue();

            control2LeftY = Gamepad2.getLeftY();

            scorer.readLiftPos();
            scorer.updateLiftGains();

            if (control2X.wasJustPressed()) {
                if (useOverrideMode) {
                    targetLiftPos = scorer.getCurrentLiftPos();
                }
                useOverrideMode = !useOverrideMode;
                loop = true;
            }

            if (useOverrideMode) {
                if (control2RShoulder.wasJustPressed()) {
                    scorer.resetLiftEncoder();
                }

                scorer.runLift(control2LeftY);
            } else {
                if (control2RShoulder.wasJustPressed()) {
                    loop = !loop;
                }

                if (loop) {
                    if (control2Up.wasJustPressed()) {
                        targetLiftPos = TeleOpConfig.HEIGHT_TALL;
                    } else if (control2Left.wasJustPressed()) {
                        targetLiftPos = TeleOpConfig.HEIGHT_MEDIUM;
                    } else if (control2Right.wasJustPressed()) {
                        targetLiftPos = TeleOpConfig.HEIGHT_LOW;
                    } else if (control2Down.wasJustPressed()) {
                        targetLiftPos = TeleOpConfig.HEIGHT_FLOOR;
                    }

                    if (scorer.getCurrentLiftPos() == TeleOpConfig.HEIGHT_FLOOR) {
                        scorer.setTargetLiftPos(targetLiftPos);
                    } else if (scorer.getCurrentLiftPos() == targetLiftPos) {
                        scorer.setTargetLiftPos(PowerplayScorer.liftPos.FLOOR);
                    }
                } else {
                    if (control2Up.wasJustPressed()) {
                        scorer.setTargetLiftPos(PowerplayScorer.liftPos.TALL);
                    } else if (control2Left.wasJustPressed()) {
                        scorer.setTargetLiftPos(PowerplayScorer.liftPos.MED);
                    } else if (control2Right.wasJustPressed()) {
                        scorer.setTargetLiftPos(PowerplayScorer.liftPos.LOW);
                    } else if (control2Down.wasJustPressed()) {
                        scorer.setTargetLiftPos(PowerplayScorer.liftPos.FLOOR);
                    }
                }

                scorer.runLiftToPos();
                scorer.runPassThruStates();
            }

            scorer.runClaw();
            scorer.runPivot();
            scorer.runPassThruServos();

            //everything below is telemetry
            myTelemetry.addLine();
            scorer.printTelemetry(myTelemetry);
            myTelemetry.update();
        }
    }
}
