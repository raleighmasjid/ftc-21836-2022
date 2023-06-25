package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.control.controller.PIDController;

public class PositionLockingMecanum extends MecanumDrivetrain {

    public final PIDController xController = new PIDController();
    public final PIDController yController = new PIDController();
    public final PIDController headingController = new PIDController();
    public final PIDController[] positionControllers = {xController, yController};

    public PositionLockingMecanum(HardwareMap hw) {
        super(hw);
    }

    @Override
    public void run(double xCommand, double yCommand, double turnCommand) {

        if (xCommand == 0.0) {
            evaluateIntegralReset(xController);
            xCommand = xController.update(getX());
        } else {
            xController.setTarget(getX());
        }

        if (yCommand == 0.0) {
            evaluateIntegralReset(yController);
            yCommand = yController.update(getY());
        } else {
            yController.setTarget(getY());
        }

        if (turnCommand == 0.0) {
            evaluateIntegralReset(headingController);
            turnCommand = headingController.update(headingController.getTarget() - normalizeAngle(headingController.getTarget() - getHeading()));
        } else {
            headingController.setTarget(getHeading());
        }

        super.run(xCommand, yCommand, turnCommand);
    }

    protected void evaluateIntegralReset(PIDController controller) {
        double error = controller.getError();
        if (error == 0.0 || Math.signum(error) != Math.signum(controller.getLastError())) {
            controller.resetIntegral();
        }
    }

    @Override
    public void printNumericalTelemetry(MultipleTelemetry telemetry) {
        super.printNumericalTelemetry(telemetry);
        telemetry.addLine();
        telemetry.addData("Robot target heading", headingController.getTarget());
        telemetry.addData("Robot target x", xController.getTarget());
        telemetry.addData("Robot target y", yController.getTarget());
        telemetry.addLine();
        telemetry.addData("Robot heading error derivative (ticks/s)", headingController.getErrorVelocity());
        telemetry.addData("Robot x error derivative (ticks/s)", xController.getErrorVelocity());
        telemetry.addData("Robot y error derivative (ticks/s)", yController.getErrorVelocity());
    }
}
