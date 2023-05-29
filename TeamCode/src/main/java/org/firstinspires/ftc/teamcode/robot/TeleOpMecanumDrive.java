package org.firstinspires.ftc.teamcode.robot;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.jetbrains.annotations.Contract;


public class TeleOpMecanumDrive {
    private final IMU imu;

    private final MecanumDrive mecanumDrivetrain;

    private double headingOffset, latestIMUReading;

    @NonNull
    @Contract("_, _ -> new")
    private MotorEx drivetrainMotor(HardwareMap hw, String name) {
        return new MotorEx(hw, name, 537.7, 312);
    }

    public TeleOpMecanumDrive(HardwareMap hw) {
        headingOffset = 0.0;

        // Assign motors using their hardware map names, each drive-type can have different names if needed
        MotorEx
                motor_frontLeft = drivetrainMotor(hw, "left front"),
                motor_frontRight = drivetrainMotor(hw, "right front"),
                motor_backLeft = drivetrainMotor(hw, "left back"),
                motor_backRight = drivetrainMotor(hw, "right back");

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

    // ftclib field-centric mecanum drive code:
    public void driveFieldCentric(double leftX, double leftY, double rightX) {
        mecanumDrivetrain.driveFieldCentric(leftX, leftY, rightX, getHeading());
    }

    public double getHeading() {
        return getIMUHeading() - headingOffset;
    }

    private double getIMUHeading() {
        return latestIMUReading;
    }

    public void readIMU() {
        latestIMUReading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    /**
     * Set internal heading of the robot to correct field-centric direction
     *
     * @param angle Angle of the robot in degrees, 0 facing forward and increases counter-clockwise
     */
    public void setHeading(double angle) {
        headingOffset = getIMUHeading() - angle;
    }
}

