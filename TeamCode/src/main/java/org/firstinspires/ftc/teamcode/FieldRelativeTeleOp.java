package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.List;


@TeleOp(name="Field Relative", group = "21836 Teleop")

public class FieldRelativeTeleOp extends LinearOpMode {

    PowerplayScorer scorer = new PowerplayScorer();
    MarvelsMecanumDrive drivetrain = new MarvelsMecanumDrive();
    List<LynxModule> hubs;

    @Override
    public void runOpMode() throws InterruptedException {

//      Initialize telemetry and dashboard
        MultipleTelemetry myTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        FtcDashboard dashboard = FtcDashboard.getInstance();

//      initializes code:
        scorer.init(hardwareMap);
        drivetrain.init(hardwareMap);

        hubs = hardwareMap.getAll(LynxModule.class);

//      instantiates both gamepads:
        GamepadEx Gamepad1 = new GamepadEx(gamepad1);
        GamepadEx Gamepad2 = new GamepadEx(gamepad2);

        ButtonReader control2A = new ButtonReader(Gamepad2, GamepadKeys.Button.A); //drop + open claw
        ButtonReader control2B = new ButtonReader(Gamepad2, GamepadKeys.Button.B); //close claw + lift
        ButtonReader control2X = new ButtonReader(Gamepad2, GamepadKeys.Button.X); //claw override
        ButtonReader control2Y = new ButtonReader(Gamepad2, GamepadKeys.Button.Y); //claw spin

        ButtonReader control2LShoulder = new ButtonReader(Gamepad2, GamepadKeys.Button.LEFT_BUMPER);
        ButtonReader control2RShoulder = new ButtonReader(Gamepad2, GamepadKeys.Button.RIGHT_BUMPER);

        ButtonReader control2Up = new ButtonReader(Gamepad2, GamepadKeys.Button.DPAD_UP);
        ButtonReader control2Left = new ButtonReader(Gamepad2, GamepadKeys.Button.DPAD_LEFT);
        ButtonReader control2Right = new ButtonReader(Gamepad2, GamepadKeys.Button.DPAD_RIGHT);
        ButtonReader control2Down = new ButtonReader(Gamepad2, GamepadKeys.Button.DPAD_DOWN);

        double control1LeftY;
        double control1LeftX;
        double control1RightX;
        double control2LeftY;

        double precisionScale = 1;

        waitForStart();

//      teleop control loop
        while (opModeIsActive()) {
            // get button inputs

            control2B.readValue();
            control2A.readValue();
            control2X.readValue();
            control2Y.readValue();

            control2LShoulder.readValue();
            control2RShoulder.readValue();

            control2Up.readValue();
            control2Left.readValue();
            control2Right.readValue();
            control2Down.readValue();

            control1LeftY = Gamepad1.getLeftY();
            control1LeftX = Gamepad1.getLeftX();
            control1RightX = Gamepad1.getRightX();

            control2LeftY = Gamepad2.getLeftY();

            scorer.runClaw();
            scorer.runPivot();
            scorer.runPassthrough();
            scorer.runLiftToPos();

            scorer.targetLiftPos = Math.min((scorer.liftController.getSetPoint() + (TeleOpConfig.LIFT_MANUAL_CONTROL_SCALE * control2LeftY)), TeleOpConfig.HEIGHT_TALL);

            //claw and passthrough buttons
            if (control2X.wasJustPressed()) {
                scorer.toggleClaw(); //claw override
            }
            if(control2Y.wasJustPressed()){
                scorer.togglePassthrough();
            }
            if (control2A.wasJustPressed()) {
                scorer.dropClaw();
            }
            if (control2B.wasJustPressed()) {
                scorer.liftClaw();
            }
            if (control2LShoulder.wasJustPressed()) {
                scorer.togglePivot(); //pivot override
            }
            if (control2RShoulder.wasJustPressed()) {
                scorer.lift_motor2.resetEncoder();
            }


            // lift heights
            if (control2Up.wasJustPressed()) {
                scorer.setLiftPos(PowerplayScorer.liftHeights.TALL);
            }
            if (control2Left.wasJustPressed()) {
                scorer.setLiftPos(PowerplayScorer.liftHeights.MED);
            }
            if (control2Right.wasJustPressed()) {
                scorer.setLiftPos(PowerplayScorer.liftHeights.LOW);
            }
            if (control2Down.wasJustPressed()) {
                scorer.setLiftPos(PowerplayScorer.liftHeights.GROUND);
            }


            //field-centric reset
            if (Gamepad1.isDown(GamepadKeys.Button.A)) {
                drivetrain.resetRotation();
            }


            precisionScale = (TeleOpConfig.PRECISION_MODE_SCALE - 1) * Gamepad1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) + 1;

            control1LeftX *= precisionScale;
            control1LeftY *= precisionScale;
            control1RightX *= precisionScale;

            // runs field-centric driving using analog stick inputs
            drivetrain.driveFieldCentric(control1LeftX, control1LeftY, control1RightX);



            //everything below is telemetry
            if (scorer.limitSwitch.getState()) {
                myTelemetry.addData("Limit switch", "is not triggered");
            } else {
                myTelemetry.addData("Limit switch", "is triggered");
            }

            if (scorer.clawIsPass) {
                myTelemetry.addData("Claw is", "passing through");
            } else if (scorer.clawIsOpen){
                myTelemetry.addData("Claw is", "open");
            } else {
                myTelemetry.addData("Claw is", "closed");
            }

            if (scorer.passIsFront) {
                myTelemetry.addData("Passthrough is in the", "front");
            } else {
                myTelemetry.addData("Passthrough is in the", "back");
            }

            myTelemetry.addData("Lift position:", scorer.targetLiftPosName);
            myTelemetry.addData("Lift encoder raw output:", scorer.lift_motor2.encoder.getPosition());
            myTelemetry.addData("Lift target pos:", scorer.targetLiftPos);
            myTelemetry.addData("Lift motors output", scorer.liftVelocity);

            myTelemetry.addData("Passthrough status", scorer.currentPassPos);
            myTelemetry.addData("Current draw lift 1",scorer.lift_motor1.motorEx.getCurrent(CurrentUnit.AMPS));

            myTelemetry.addData("Current draw lift 2",scorer.lift_motor2.motorEx.getCurrent(CurrentUnit.AMPS));

            myTelemetry.addData("Current draw lift 3",scorer.lift_motor3.motorEx.getCurrent(CurrentUnit.AMPS));
            myTelemetry.addData("Hub 0 draw", hubs.get(0).getCurrent(CurrentUnit.AMPS));
            myTelemetry.addData("Hub 0 name", hubs.get(0).getDeviceName());
            myTelemetry.addData("Hub 1 draw", hubs.get(1).getCurrent(CurrentUnit.AMPS));
            myTelemetry.addData("Hub 1 name", hubs.get(1).getDeviceName());

            myTelemetry.addData("Status", "power: x:" + control1LeftX + " y:" + control1LeftY + " z:" + control1RightX);
            myTelemetry.addData("Field-relative heading", drivetrain.rotYaw);
            myTelemetry.update();
        }
    }
}
