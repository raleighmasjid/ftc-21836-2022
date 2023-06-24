package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.control.controller.PIDController;

public class HeadingLockingMecanum extends MecanumDrivetrain {

    public final PIDController headingController = new PIDController();

    private double targetHeading;

    public HeadingLockingMecanum(HardwareMap hw) {
        super(hw);
    }

    @Override
    public void run(double xCommand, double yCommand, double turnCommand) {
        if (turnCommand != 0) {
            targetHeading = getCurrentHeading();
            headingController.setTarget(targetHeading);
        } else
            turnCommand = headingController.update(targetHeading - normalizeAngle(targetHeading - getCurrentHeading()));

        double error = headingController.getError();
        if (error == 0.0 || Math.signum(error) != Math.signum(headingController.getLastError())) {
            headingController.resetIntegral();
        }

        super.run(xCommand, yCommand, turnCommand);
    }

    public void setTargetHeading(double targetHeading) {
        this.targetHeading = targetHeading;
    }
}
