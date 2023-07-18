package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.control.State;
import org.firstinspires.ftc.teamcode.control.controllers.PIDController;
import org.firstinspires.ftc.teamcode.control.gainmatrices.PIDGains;

@Config
public class HeadingLockingMecanum extends MecanumDrivetrain {

    public static double
            kS = 0.0,
            FILTER_GAIN = 0.8,
            TURN_SETTLING_TIME = 0.3,
            TRANSLATION_SETTLING_TIME = 0.3;

    public static int FILTER_COUNT = 50;

    public static PIDGains gains = new PIDGains(
            0.0275,
            0.005,
            0.0015,
            0.3
    );

    private boolean correctHeading = true;

    private double lastXCommand = 0.0, lastYCommand = 0.0, headingTarget;

    private final ElapsedTime turnSettlingTimer, translationSettlingTimer;

    private final PIDController headingController = new PIDController();

    public HeadingLockingMecanum(HardwareMap hw, double motorCPR, double motorRPM) {
        super(hw, motorCPR, motorRPM);
        turnSettlingTimer = new ElapsedTime();
        translationSettlingTimer = new ElapsedTime();
    }

    @Override
    public void readIMU() {
        headingController.setGains(gains);
        headingController.derivFilter.setGains(FILTER_GAIN, FILTER_COUNT);
        super.readIMU();
    }

    @Override
    public void run(double xCommand, double yCommand, double turnCommand) {
        double scalar = 12.0 / batteryVoltageSensor.getVoltage();
        boolean useManualInput = turnCommand != 0.0;

        if (!correctHeading || useManualInput) {
            turnCommand *= scalar;
            turnSettlingTimer.reset();
        }

        if (correctHeading) {
            boolean xStopped = (lastXCommand != 0) && (xCommand == 0);
            boolean yStopped = (lastYCommand != 0) && (yCommand == 0);
            if (xStopped || yStopped) translationSettlingTimer.reset();

            if (useManualInput || turnSettlingTimer.seconds() <= TURN_SETTLING_TIME) {
                headingTarget = getHeading();
            } else if (translationSettlingTimer.seconds() > TRANSLATION_SETTLING_TIME) {
                headingController.setError(-normalizeAngle(headingTarget - getHeading()));
                double pidOutput = headingController.calculate(new State(getHeading()));
                turnCommand = pidOutput + (Math.signum(pidOutput) * kS * scalar);
            }
        }

        lastXCommand = xCommand;
        lastYCommand = yCommand;
        super.run(xCommand * scalar, yCommand * scalar, turnCommand);
    }

    public void setTargetHeading(double angle) {
        headingTarget = angle;
    }

    @Override
    public void setCurrentHeading(double angle) {
        super.setCurrentHeading(angle);
        setTargetHeading(angle);
    }

    public void toggleHeadingCorrection() {
        correctHeading = !correctHeading;
    }

    public void printTelemetry(MultipleTelemetry telemetry) {
        telemetry.addData("Heading correction is", correctHeading ? "active" : "disabled");
    }

    @Override
    public void printNumericalTelemetry(MultipleTelemetry telemetry) {
        super.printNumericalTelemetry(telemetry);
        telemetry.addLine();
        telemetry.addData("Drivetrain target heading", headingTarget);
        telemetry.addLine();
        telemetry.addData("Drivetrain heading error derivative (ticks/s)", headingController.getErrorDerivative());
    }
}
