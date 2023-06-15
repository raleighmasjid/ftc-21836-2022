package org.firstinspires.ftc.teamcode.systems;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


public class MecanumDrivetrain {
    private final IMU imu;

    private final MecanumDrive mecanumDrivetrain;

    private double headingOffset = 0.0;
    private double latestIMUReading = 0.0;

    private MotorEx drivetrainMotor(HardwareMap hw, String name) {
        return new MotorEx(hw, name, 537.7, 312);
    }

    public MecanumDrivetrain(HardwareMap hw) {

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
        double angle = (latestIMUReading - headingOffset) % 360.0;
        return
                (angle == -0.0) ? 0.0 :
                        (angle > 180.0) ? angle - 360.0 :
                                (angle <= -180.0) ? angle + 360.0 :
                                        angle;
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
        headingOffset = latestIMUReading - angle;
    }
}

