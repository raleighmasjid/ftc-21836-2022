package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class MecanumDrivetrain {

    private final MecanumDrive drivebase;

    private final HardwareMap hardwareMap;

    private final IMU imu;

    private MotorEx getMotor(String name) {
        return new MotorEx(hardwareMap, name, 537.7, 312);
    }

    public MecanumDrivetrain(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        MotorEx[] motors = new MotorEx[]{
                getMotor("left front"),
                getMotor("right front"),
                getMotor("left back"),
                getMotor("right back")
        };

        for (MotorEx motor : motors) motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        drivebase = new MecanumDrive(motors[0], motors[1], motors[2], motors[3]);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        )));
    }

    public void drive(double xCommand, double yCommand, double turnCommand) {
        drivebase.driveFieldCentric(xCommand, yCommand, turnCommand, imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
    }
}
