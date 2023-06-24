package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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

    protected final MotorEx[] motors;

    private double headingOffset = 0.0;
    private double latestIMUReading = 0.0;

    public static MotorEx getDrivetrainMotor(HardwareMap hw, String name) {
        return new MotorEx(hw, name, 537.7, 312);
    }

    public MecanumDrivetrain(HardwareMap hw) {
        // Assign motors using their hardware map names, each drive-type can have different names if needed
        motors = new MotorEx[]{
                getDrivetrainMotor(hw, "left front"),
                getDrivetrainMotor(hw, "right front"),
                getDrivetrainMotor(hw, "left back"),
                getDrivetrainMotor(hw, "right back")
        };

        motors[0].setInverted(true);
        motors[1].setInverted(false);
        motors[2].setInverted(true);
        motors[3].setInverted(false);

        for (MotorEx motor : motors) {
            motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        }

        // Initialize the FTCLib drive-base
        mecanumDrivetrain = new MecanumDrive(false, motors[0], motors[1], motors[2], motors[3]);

        imu = hw.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        )));
    }

    public static double normalizeAngle(double angle) {
        angle %= 360.0;
        return
                (angle == -0.0) ? 0.0 :
                        (angle > 180.0) ? angle - 360.0 :
                                (angle <= -180.0) ? angle + 360.0 :
                                        angle;
    }

    public void readCurrentHeading() {
        latestIMUReading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    /**
     * Set internal heading of the robot to correct field-centric direction
     *
     * @param angle Angle of the robot in degrees, 0 facing forward and increases counter-clockwise
     */
    public void setCurrentHeading(double angle) {
        headingOffset = latestIMUReading - angle;
    }

    public double getHeading() {
        return normalizeAngle(latestIMUReading - headingOffset);
    }

    protected double getMotorPos(int motorIndex) {
        return motors[motorIndex].encoder.getPosition();
    }

    protected double getY() {
        return (getMotorPos(0) + getMotorPos(1) + getMotorPos(2) + getMotorPos(3)) * 0.25;
    }

    protected double getX() {
        return (getMotorPos(0) - getMotorPos(1) - getMotorPos(2) + getMotorPos(3)) * 0.25;
    }

    public void run(double leftX, double leftY, double rightX) {
        mecanumDrivetrain.driveFieldCentric(leftX, leftY, rightX, getHeading());
    }

    public void printNumericalTelemetry(MultipleTelemetry telemetry) {
        telemetry.addData("Robot current heading", getHeading());
        telemetry.addLine();
        telemetry.addData("Robot current x", getX());
        telemetry.addData("Robot current y", getY());
        telemetry.addLine();
        telemetry.addData("Front left", getMotorPos(0));
        telemetry.addData("Front right", getMotorPos(1));
        telemetry.addData("Back left", getMotorPos(2));
        telemetry.addData("Back right", getMotorPos(3));
    }
}
