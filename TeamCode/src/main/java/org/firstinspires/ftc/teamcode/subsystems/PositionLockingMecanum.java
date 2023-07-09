package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.control.controllers.JointController;

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

    public final JointController xController = new JointController();
    public final JointController yController = new JointController();
    public final JointController headingController = new JointController();
    public final JointController[] positionControllers = {xController, yController};
    public final JointController[] controllers = {xController, yController, headingController};

    public PositionLockingMecanum(HardwareMap hw, double motorCPR, double motorRPM) {
        super(hw, motorCPR, motorRPM);
    }

    @Override
    public void readIMU() {
        for (JointController controller : positionControllers) {
            controller.feedback.setGains(
                    POSITION_kP,
                    POSITION_kI,
                    POSITION_kD
            );
            controller.feedback.derivFilter.setGains(
                    POSITION_FILTER_GAIN,
                    POSITION_FILTER_COUNT
            );
        }
        headingController.feedback.setGains(
                HEADING_kP,
                HEADING_kI,
                HEADING_kD
        );
        headingController.feedback.derivFilter.setGains(
                HEADING_FILTER_GAIN,
                HEADING_FILTER_COUNT
        );
        for (JointController controller : controllers)
            controller.feedforward.setGains(0.0, 0.0, kS);
        super.readIMU();
    }

    public void run(double xCommand, double yCommand, double turnCommand) {
        double voltage = batteryVoltageSensor.getVoltage();

        if (xCommand == 0.0) xCommand = xController.calculate(getX(), voltage);
        else {
            xController.feedback.setTarget(getX());
            xCommand *= (12.0 / voltage);
        }

        if (yCommand == 0.0) yCommand = yController.calculate(getY(), voltage);
        else {
            yController.feedback.setTarget(getY());
            yCommand *= (12.0 / voltage);
        }

        if (turnCommand == 0.0) {
            headingController.feedback.setError(normalizeAngle(-normalizeAngle(headingController.feedback.getTarget() - getHeading())));
            turnCommand = headingController.calculate(getHeading(), voltage);
        } else {
            setTargetHeading(getHeading());
            turnCommand *= (12.0 / voltage);
        }

        super.run(xCommand, yCommand, turnCommand, false);
    }

    public void setTargetHeading(double angle) {
        headingController.feedback.setTarget(normalizeAngle(angle));
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
        telemetry.addData("Drivetrain target heading", headingController.feedback.getTarget());
        telemetry.addData("Drivetrain target x", xController.feedback.getTarget());
        telemetry.addData("Drivetrain target y", yController.feedback.getTarget());
        telemetry.addLine();
        telemetry.addData("Drivetrain heading error derivative (ticks/s)", headingController.feedback.getErrorDerivative());
        telemetry.addData("Drivetrain x error derivative (ticks/s)", xController.feedback.getErrorDerivative());
        telemetry.addData("Drivetrain y error derivative (ticks/s)", yController.feedback.getErrorDerivative());
    }
}
