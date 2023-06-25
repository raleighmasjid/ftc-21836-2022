package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.control.controller.PIDFController;

public class PositionLockingMecanum extends MecanumDrivetrain {

    public final PIDFController xController = new PIDFController();
    public final PIDFController yController = new PIDFController();
    public final PIDFController headingController = new PIDFController();
    public final PIDFController[] positionControllers = {xController, yController};
    public final PIDFController[] controllers = {xController, yController, headingController};
    private final VoltageSensor batteryVoltageSensor;

    public PositionLockingMecanum(HardwareMap hw) {
        super(hw);
        this.batteryVoltageSensor = hw.voltageSensor.iterator().next();
        for (PIDFController controller : controllers) {
            controller.setOutputBounds(-1.0, 1.0);
        }
    }

    @Override
    public void run(double xCommand, double yCommand, double turnCommand) {
        double voltage = batteryVoltageSensor.getVoltage();

        if (xCommand == 0.0) {
            xCommand = xController.update(getX(), voltage);
        } else {
            xController.pid.setTarget(getX());
        }

        if (yCommand == 0.0) {
            yCommand = yController.update(getY(), voltage);
        } else {
            yController.pid.setTarget(getY());
        }

        if (turnCommand == 0.0) {
            turnCommand = headingController.update(headingController.pid.getTarget() - normalizeAngle(headingController.pid.getTarget() - getHeading()), voltage);
        } else {
            headingController.pid.setTarget(getHeading());
        }

        super.run(xCommand, yCommand, turnCommand);
    }

    @Override
    public void printNumericalTelemetry(MultipleTelemetry telemetry) {
        super.printNumericalTelemetry(telemetry);
        telemetry.addLine();
        telemetry.addData("Robot target heading", headingController.pid.getTarget());
        telemetry.addData("Robot target x", xController.pid.getTarget());
        telemetry.addData("Robot target y", yController.pid.getTarget());
        telemetry.addLine();
        telemetry.addData("Robot heading error derivative (ticks/s)", headingController.pid.getErrorVelocity());
        telemetry.addData("Robot x error derivative (ticks/s)", xController.pid.getErrorVelocity());
        telemetry.addData("Robot y error derivative (ticks/s)", yController.pid.getErrorVelocity());
    }
}
