package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.Arrays;
import java.util.Collections;


public class MecanumDrivetrain {

    private final IMU imu;

    private final MecanumDrive mecanumDrivetrain;

    protected final MotorEx[] motors;

    private double headingOffset, latestIMUReading;

    private final HardwareMap hw;
    private final double motorCPR, motorRPM;

    protected final VoltageSensor batteryVoltageSensor;

    protected MotorEx getMotor(String name) {
        return new MotorEx(hw, name, motorCPR, motorRPM);
    }

    public MecanumDrivetrain(HardwareMap hw, double motorCPR, double motorRPM) {
        this.hw = hw;
        this.motorCPR = motorCPR;
        this.motorRPM = motorRPM;
        this.batteryVoltageSensor = hw.voltageSensor.iterator().next();

        // Assign motors using their hardware map names, each drive-type can have different names if needed
        motors = new MotorEx[]{
                getMotor("left front"),
                getMotor("right front"),
                getMotor("left back"),
                getMotor("right back")
        };

        motors[0].setInverted(true);
        motors[1].setInverted(false);
        motors[2].setInverted(true);
        motors[3].setInverted(false);

        for (MotorEx motor : motors) motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        // Initialize the FTCLib drive-base
        mecanumDrivetrain = new MecanumDrive(false, motors[0], motors[1], motors[2], motors[3]);

        imu = hw.get(IMU.class, "imu");
        imu.resetDeviceConfigurationForOpMode();
        imu.resetYaw();
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        )));

        resetPosition();
    }

    public static double normalizeAngle(double angle) {
        angle %= 360.0;
        if (angle <= -180.0) return angle + 360.0;
        if (angle > 180.0) return angle - 360.0;
        if (angle == -0.0) return 0.0;
        return angle;
    }

    public void readIMU() {
        latestIMUReading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    /**
     * Set internal heading of the robot to correct field-centric direction
     *
     * @param angle Angle of the robot in degrees, 0 facing forward and increases counter-clockwise
     */
    public void setCurrentHeading(double angle) {
        headingOffset = latestIMUReading - normalizeAngle(angle);
    }

    public double getHeading() {
        return normalizeAngle(latestIMUReading - headingOffset);
    }

    public int getMotorPos(int motorIndex) {
        return motors[motorIndex].encoder.getPosition();
    }

    public double getY() {
        return (getMotorPos(0) + getMotorPos(1) + getMotorPos(2) + getMotorPos(3)) * 0.25;
    }

    public double getX() {
        return (getMotorPos(0) - getMotorPos(1) - getMotorPos(2) + getMotorPos(3)) * 0.25;
    }

    public void resetPosition() {
        for (MotorEx motor : motors) motor.encoder.reset();
    }

    public void run(double xCommand, double yCommand, double turnCommand) {
        // normalize inputs
        double max = Collections.max(Arrays.asList(xCommand, yCommand, turnCommand, 1.0));
        xCommand /= max;
        yCommand /= max;
        turnCommand /= max;

        mecanumDrivetrain.driveFieldCentric(xCommand, yCommand, turnCommand, getHeading());
    }

    public void printNumericalTelemetry(MultipleTelemetry telemetry) {
        telemetry.addData("Drivetrain current heading", getHeading());
    }
}

