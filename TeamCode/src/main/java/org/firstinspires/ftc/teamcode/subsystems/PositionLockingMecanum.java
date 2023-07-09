package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.control.controllers.FeedforwardController;
import org.firstinspires.ftc.teamcode.control.controllers.JointController;
import org.firstinspires.ftc.teamcode.control.controllers.PIDController;
import org.firstinspires.ftc.teamcode.control.controllers.gains.FeedforwardGains;
import org.firstinspires.ftc.teamcode.control.controllers.gains.PIDGains;

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

    private final PIDController xPID = new PIDController();
    public final JointController xController = new JointController(xPID, new FeedforwardController());

    private final PIDController yPID = new PIDController();
    public final JointController yController = new JointController(yPID, new FeedforwardController());

    private final PIDController headingPid = new PIDController();
    public final JointController headingController = new JointController(headingPid, new FeedforwardController());

    public final PIDController[] positionPIDs = {xPID, yPID};
    public final JointController[] controllers = {xController, yController, headingController};

    public PositionLockingMecanum(HardwareMap hw, double motorCPR, double motorRPM) {
        super(hw, motorCPR, motorRPM);
    }

    @Override
    public void readIMU() {
        for (PIDController controller : positionPIDs) {
            controller.setGains(new PIDGains(
                    POSITION_kP,
                    POSITION_kI,
                    POSITION_kD
            ));
            controller.derivFilter.setGains(
                    POSITION_FILTER_GAIN,
                    POSITION_FILTER_COUNT
            );
        }
        headingPid.setGains(new PIDGains(
                HEADING_kP,
                HEADING_kI,
                HEADING_kD
        ));
        headingPid.derivFilter.setGains(
                HEADING_FILTER_GAIN,
                HEADING_FILTER_COUNT
        );
        for (JointController controller : controllers)
            controller.feedforward.setGains(new FeedforwardGains(0.0, 0.0, kS));
        super.readIMU();
    }

    public void run(double xCommand, double yCommand, double turnCommand) {
        double voltage = batteryVoltageSensor.getVoltage();

        if (xCommand == 0.0)
            xCommand = xController.calculate(new MotionState(getX(), 0.0), voltage);
        else {
            xController.feedback.setTarget(new MotionState(getX(), 0.0));
            xCommand *= (12.0 / voltage);
        }

        if (yCommand == 0.0)
            yCommand = yController.calculate(new MotionState(getY(), 0.0), voltage);
        else {
            yController.feedback.setTarget(new MotionState(getY(), 0.0));
            yCommand *= (12.0 / voltage);
        }

        if (turnCommand == 0.0) {
            headingPid.setError(normalizeAngle(-normalizeAngle(headingPid.getTarget() - getHeading())));
            turnCommand = headingController.calculate(new MotionState(getHeading(), 0.0), voltage);
        } else {
            setTargetHeading(getHeading());
            turnCommand *= (12.0 / voltage);
        }

        super.run(xCommand, yCommand, turnCommand, false);
    }

    public void setTargetHeading(double angle) {
        headingController.feedback.setTarget(new MotionState(normalizeAngle(angle), 0.0));
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
        telemetry.addData("Drivetrain target heading", headingPid.getTarget());
        telemetry.addData("Drivetrain target x", xPID.getTarget());
        telemetry.addData("Drivetrain target y", yPID.getTarget());
        telemetry.addLine();
        telemetry.addData("Drivetrain heading error derivative (ticks/s)", headingPid.getErrorDerivative());
        telemetry.addData("Drivetrain x error derivative (ticks/s)", xPID.getErrorDerivative());
        telemetry.addData("Drivetrain y error derivative (ticks/s)", yPID.getErrorDerivative());
    }
}
