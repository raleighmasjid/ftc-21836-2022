package org.firstinspires.ftc.teamcode.robot;

import static org.firstinspires.ftc.teamcode.autonomous.DriveConstants.MAX_RPM;
import static org.firstinspires.ftc.teamcode.autonomous.DriveConstants.TICKS_PER_REV;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


public class TeleOpMecanumDrive {
    private final IMU imu;

    private final MecanumDrive mecanumDrivetrain;

    private double headingOffset, heading;

    // ftclib field-centric mecanum drive code:
    public void driveFieldCentric(double leftX, double leftY, double rightX) {
        heading = getIMUHeading() - headingOffset;
        mecanumDrivetrain.driveFieldCentric(leftX, leftY, rightX, heading);
    }

    public double getHeading() {
        return heading;
    }

    private double getIMUHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public void resetHeading() {
        headingOffset = getIMUHeading();
    }

    public void setHeading(double startAngle) {
        headingOffset = getIMUHeading() - startAngle;
    }

    public TeleOpMecanumDrive(HardwareMap hw) {
        headingOffset = 0.0;

        // Assign motors using their hardware map names, each drive-type can have different names if needed
        MotorEx
                motor_frontLeft = new MotorEx(hw, "left front", TICKS_PER_REV, MAX_RPM),
                motor_frontRight = new MotorEx(hw, "right front", TICKS_PER_REV, MAX_RPM),
                motor_backLeft = new MotorEx(hw, "left back", TICKS_PER_REV, MAX_RPM),
                motor_backRight = new MotorEx(hw, "right back", TICKS_PER_REV, MAX_RPM);

        imu = hw.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        );
        IMU.Parameters parameters = new IMU.Parameters(orientation);
        imu.initialize(parameters);

        motor_frontLeft.setInverted(true);
        motor_backLeft.setInverted(true);
        motor_frontRight.setInverted(true);
        motor_backRight.setInverted(true);

        motor_frontLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motor_frontRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motor_backLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motor_backRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        // Initialize the FTCLib drive-base
        mecanumDrivetrain = new MecanumDrive(
                motor_frontLeft,
                motor_frontRight,
                motor_backLeft,
                motor_backRight
        );
    }
}

