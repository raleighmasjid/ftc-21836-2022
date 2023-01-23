package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="Field Relative", group = "21836 Teleop")

public class FieldRelativeTeleOp extends LinearOpMode {

    PowerplayScorer scorer = new PowerplayScorer();
    MarvelsMecanumDrive drivetrain = new MarvelsMecanumDrive();

    @Override
    public void runOpMode() throws InterruptedException {

//      Initialize telemetry and dashboard
        MultipleTelemetry mytelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        FtcDashboard dashboard = FtcDashboard.getInstance();

//      initializes code:
        scorer.init(hardwareMap);
        drivetrain.init(hardwareMap);


//      instantiates both gamepads:
        GamepadEx Gamepad1 = new GamepadEx(gamepad1);
        GamepadEx Gamepad2 = new GamepadEx(gamepad2);

        ButtonReader control1A = new ButtonReader(Gamepad1, GamepadKeys.Button.A); //reset field-centric heading

        ButtonReader control2A = new ButtonReader(Gamepad2, GamepadKeys.Button.A); //drop + open claw
        ButtonReader control2B = new ButtonReader(Gamepad2, GamepadKeys.Button.B); //close claw + lift
        ButtonReader control2X = new ButtonReader(Gamepad2, GamepadKeys.Button.X); //claw override
        ButtonReader control2Y = new ButtonReader(Gamepad2, GamepadKeys.Button.Y); //claw spin

        ButtonReader control2LShoulder = new ButtonReader(Gamepad2, GamepadKeys.Button.LEFT_BUMPER);

        ButtonReader control2Up = new ButtonReader(Gamepad2, GamepadKeys.Button.DPAD_UP);
        ButtonReader control2Left = new ButtonReader(Gamepad2, GamepadKeys.Button.DPAD_LEFT);
        ButtonReader control2Right = new ButtonReader(Gamepad2, GamepadKeys.Button.DPAD_RIGHT);
        ButtonReader control2Down = new ButtonReader(Gamepad2, GamepadKeys.Button.DPAD_DOWN);

        double control1LeftY;
        double control1LeftX;
        double control1RightX;
        double control2LeftY;

        double targetPos;

        scorer.lift_motor2.resetEncoder();
        scorer.setLiftPos(PowerplayScorer.heightVal.ONE);
        drivetrain.resetRotation();

        scorer.limitSwitch = hardwareMap.get(DigitalChannel.class, "sensor_digital");

        // set the digital channel to input.
        scorer.limitSwitch.setMode(DigitalChannel.Mode.INPUT);

        waitForStart();

//      teleop control loop
        while (opModeIsActive()) {
            // get button inputs
            control1A.readValue();

            control2B.readValue();
            control2A.readValue();
            control2X.readValue();
            control2Y.readValue();
            control2LShoulder.readValue();

            control2Up.readValue();
            control2Left.readValue();
            control2Right.readValue();
            control2Down.readValue();

//            constantly moves the claw to its position dictated by "clawOpen"
            scorer.runClaw();
            scorer.runPivot();
            scorer.runPassthrough();
            scorer.runLiftPos();

            scorer.liftController.setTolerance(TeleOpConfig.LIFT_E_TOLERANCE, TeleOpConfig.LIFT_V_TOLERANCE);
            scorer.liftController.setPIDF(
                    TeleOpConfig.LIFT_P,
                    TeleOpConfig.LIFT_I,
                    TeleOpConfig.LIFT_D,
                    TeleOpConfig.LIFT_F
            );

            mytelemetry.addData("Claw is open:", scorer.clawIsOpen);
            mytelemetry.addData("Lift position:", scorer.liftPosStr);
            mytelemetry.addData("Lift encoder raw output:", scorer.lift_motor2.encoder.getPosition());
            mytelemetry.addData("Lift target pos:", scorer.liftController.getSetPoint());

            control1LeftY = Gamepad1.getLeftY();
            control1LeftX = Gamepad1.getLeftX();
            control1RightX = Gamepad1.getRightX();

            control2LeftY = Gamepad2.getLeftY();

            targetPos = scorer.liftController.getSetPoint() + (7 * control2LeftY);
            scorer.liftController.setSetPoint(targetPos);

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



            if (control2Up.wasJustPressed()) {
                scorer.setLiftPos(PowerplayScorer.heightVal.TALL);
            }
            if (control2Left.wasJustPressed()) {
                scorer.setLiftPos(PowerplayScorer.heightVal.MED);
            }
            if (control2Right.wasJustPressed()) {
                scorer.setLiftPos(PowerplayScorer.heightVal.LOW);
            }
            if (control2Down.wasJustPressed()) {
                scorer.setLiftPos(PowerplayScorer.heightVal.GROUND);
            }


            if (control1A.isDown()) {
                drivetrain.resetRotation();
            }

            // runs field-centric driving using analog stick inputs
            drivetrain.driveFieldCentric(control1LeftX, control1LeftY, control1RightX);

            if (scorer.limitSwitch.getState()) {
                mytelemetry.addData("Digital Touch", "Is Not Pressed");
            } else {
                mytelemetry.addData("Digital Touch", "Is Pressed");
            }


            mytelemetry.addData("Lift motor 1 output", scorer.lift_motor1.get());
            mytelemetry.addData("Lift motor 2 output", scorer.lift_motor2.get());
            mytelemetry.addData("Lift motor 3 output", scorer.lift_motor3.get());

            mytelemetry.addData("Status", "power: x:" + control1LeftX + " y:" + control1LeftY + " z:" + control1RightX);
            mytelemetry.addData("Field-relative heading", drivetrain.rotYaw);
            mytelemetry.addData("Passthrough is in the front", scorer.passIsFront);
            mytelemetry.update();
        }
    }
}
