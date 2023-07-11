package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.control.controllers.PIDFController;

import java.util.Arrays;
import java.util.Collections;

@Config
public class HeadingLockingMecanum extends MecanumDrivetrain {

    public static double
            kP = 0.0,
            kI = 0.0,
            kD = 0.0,
            kS = 0.0,
            FILTER_GAIN = 0.85;

    public static int HEADING_FILTER_COUNT = 300;

    private final PIDFController headingController = new PIDFController();

    public HeadingLockingMecanum(HardwareMap hw, double motorCPR, double motorRPM) {
        super(hw, motorCPR, motorRPM);
    }

    @Override
    public void readIMU() {
        headingController.pid.setGains(
                kP,
                kI,
                kD
        );
        headingController.pid.derivFilter.setGains(
                FILTER_GAIN,
                HEADING_FILTER_COUNT
        );
        headingController.feedforward.setGains(0.0, 0.0, kS);
        super.readIMU();
    }

    @Override
    public void run(double xCommand, double yCommand, double turnCommand) {
        double voltage = batteryVoltageSensor.getVoltage();
        double scalar = 12.0 / voltage;

        if (turnCommand != 0.0) {
            setTargetHeading(getHeading());
            turnCommand *= scalar;
        } else {
            headingController.pid.setError(-normalizeAngle(headingController.pid.getTarget() - getHeading()));
            turnCommand = headingController.update(getHeading(), voltage);
        }

        xCommand *= scalar;
        yCommand *= scalar;

        double max = Collections.max(Arrays.asList(xCommand, yCommand, turnCommand, 1.0));
        xCommand /= max;
        yCommand /= max;
        turnCommand /= max;

        super.run(xCommand, yCommand, turnCommand);
    }

    public void setTargetHeading(double angle) {
        headingController.pid.setTarget(normalizeAngle(angle));
    }

    @Override
    public void setCurrentHeading(double angle) {
        super.setCurrentHeading(angle);
        setTargetHeading(angle);
    }

    @Override
    public void printNumericalTelemetry(MultipleTelemetry telemetry) {
        super.printNumericalTelemetry(telemetry);
        telemetry.addLine();
        telemetry.addData("Drivetrain target heading", headingController.pid.getTarget());
        telemetry.addLine();
        telemetry.addData("Drivetrain heading error derivative (ticks/s)", headingController.pid.getErrorDerivative());
    }
}
