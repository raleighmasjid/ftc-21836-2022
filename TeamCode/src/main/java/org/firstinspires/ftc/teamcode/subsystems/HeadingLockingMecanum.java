package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.control.controllers.PIDFController;

@Config
public class HeadingLockingMecanum extends MecanumDrivetrain {

    public static double
            kP = 0.005,
            kI = 0.01,
            kD = 0.0,
            kS = 0.0,
            MAX_OUTPUT_WITH_INTEGRAL = 1.0,
            FILTER_GAIN = 0.85,
            SETTLING_TIME = 0.3;

    public static int FILTER_COUNT = 300;

    private final ElapsedTime settlingTimer;

    private final PIDFController headingController = new PIDFController();

    public HeadingLockingMecanum(HardwareMap hw, double motorCPR, double motorRPM) {
        super(hw, motorCPR, motorRPM);
        settlingTimer = new ElapsedTime();
    }

    @Override
    public void readIMU() {
        headingController.pid.setGains(
                kP,
                kI,
                kD,
                MAX_OUTPUT_WITH_INTEGRAL
        );
        headingController.pid.derivFilter.setGains(
                FILTER_GAIN,
                FILTER_COUNT
        );
        headingController.feedforward.setGains(0.0, 0.0, kS);
        super.readIMU();
    }

    @Override
    public void run(double xCommand, double yCommand, double turnCommand) {
        double voltage = batteryVoltageSensor.getVoltage();
        double scalar = 12.0 / voltage;
        boolean useManualInput = turnCommand != 0.0;

        if (useManualInput) {
            turnCommand *= scalar;
            settlingTimer.reset();
        }

        if (useManualInput || settlingTimer.seconds() <= SETTLING_TIME) {
            setTargetHeading(getHeading());
        } else {
            headingController.pid.setError(-normalizeAngle(headingController.pid.getTarget() - getHeading()));
            turnCommand = headingController.update(getHeading(), voltage);
        }

        super.run(xCommand * scalar, yCommand * scalar, turnCommand);
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
