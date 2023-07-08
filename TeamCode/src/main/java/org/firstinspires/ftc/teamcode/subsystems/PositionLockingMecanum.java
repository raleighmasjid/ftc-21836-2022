package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.control.controllers.PIDFController;

@Config
public class PositionLockingMecanum extends MecanumDrivetrain {

    public static double
            HEADING_kP = 0.0,
            HEADING_kI = 0.0,
            HEADING_kD = 0.0,
            HEADING_FILTER_GAIN = 0.85,
            POSITION_kP = 0.0,
            POSITION_kI = 0.0,
            POSITION_kD = 0.0,
            POSITION_FILTER_GAIN = 0.0,
            kS = 0.0;

    public static int
            HEADING_FILTER_COUNT = 300,
            POSITION_FILTER_COUNT = 0;

    public final PIDFController xController = new PIDFController();
    public final PIDFController yController = new PIDFController();
    public final PIDFController headingController = new PIDFController();
    public final PIDFController[] positionControllers = {xController, yController};
    public final PIDFController[] controllers = {xController, yController, headingController};

    public PositionLockingMecanum(HardwareMap hw, double motorCPR, double motorRPM) {
        super(hw, motorCPR, motorRPM);
    }

    @Override
    public void readIMU() {
        for (PIDFController controller : positionControllers) {
            controller.pid.setGains(
                    POSITION_kP,
                    POSITION_kI,
                    POSITION_kD
            );
            controller.pid.derivFilter.setGains(
                    POSITION_FILTER_GAIN,
                    POSITION_FILTER_COUNT
            );
        }
        headingController.pid.setGains(
                HEADING_kP,
                HEADING_kI,
                HEADING_kD
        );
        headingController.pid.derivFilter.setGains(
                HEADING_FILTER_GAIN,
                HEADING_FILTER_COUNT
        );
        for (PIDFController controller : controllers) controller.feedforward.setGains(0.0, 0.0, kS);
        super.readIMU();
    }

    public void run(double xCommand, double yCommand, double turnCommand) {
        double voltage = batteryVoltageSensor.getVoltage();

        if (xCommand == 0.0) xCommand = xController.update(getX(), voltage);
        else {
            xController.pid.setTarget(getX());
            xCommand *= (12.0 / voltage);
        }

        if (yCommand == 0.0) yCommand = yController.update(getY(), voltage);
        else {
            yController.pid.setTarget(getY());
            yCommand *= (12.0 / voltage);
        }

        if (turnCommand == 0.0) {
            turnCommand = headingController.update(getHeading(), voltage);
        } else {
            setTargetHeading(getHeading());
            turnCommand *= (12.0 / voltage);
        }

        super.run(xCommand, yCommand, turnCommand, false);
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
        telemetry.addData("Robot target heading", headingController.pid.getTarget());
        telemetry.addData("Robot target x", xController.pid.getTarget());
        telemetry.addData("Robot target y", yController.pid.getTarget());
        telemetry.addLine();
        telemetry.addData("Robot heading error derivative (ticks/s)", headingController.pid.getErrorDerivative());
        telemetry.addData("Robot x error derivative (ticks/s)", xController.pid.getErrorDerivative());
        telemetry.addData("Robot y error derivative (ticks/s)", yController.pid.getErrorDerivative());
    }
}
