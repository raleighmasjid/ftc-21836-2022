package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name = "Main TeleOp", group = "21836 Teleop")

public class MainTeleOp extends LinearOpMode {

    // Define types
    MultipleTelemetry myTelemetry;
    GamepadEx Gamepad1, Gamepad2;

    @Override
    public void runOpMode() throws InterruptedException {

        // Do stuff on init
        MotorEx leftFront = new MotorEx(hardwareMap, "leftfront");
        MotorEx leftBack = new MotorEx(hardwareMap, "leftback");
        MotorEx rightFront = new MotorEx(hardwareMap, "rightfront");
        MotorEx rightBack = new MotorEx(hardwareMap, "rightback");
        MotorEx liftMotor = new MotorEx(hardwareMap, "liftmotor");

        // Initialize telemetry
        myTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // initializes both gamepads:
        Gamepad1 = new GamepadEx(gamepad1);
        Gamepad2 = new GamepadEx(gamepad2);

        double moveYLeft;
        double moveYRight;
        waitForStart();

        //teleop control loop
        while (opModeIsActive()) {

            // Do stuff while op mode is active

            moveYLeft = Gamepad1.getLeftY();
            leftFront.set(moveYLeft);
            leftBack.set(moveYLeft);




            moveYRight = Gamepad1.getRightY();
            rightFront.set(moveYRight);
            rightBack.set(moveYRight);





        }
    }
}
