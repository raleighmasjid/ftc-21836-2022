package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.control.State;
import org.firstinspires.ftc.teamcode.control.controllers.gainmatrices.FeedforwardGains;
import org.firstinspires.ftc.teamcode.control.controllers.gainmatrices.PIDGains;
import org.firstinspires.ftc.teamcode.control.controllers.pid.PIDVAController;

@Config
public class HeadingLockingMecanum extends MecanumDrivetrain {

    public static double
            kP = 0.005,
            kI = 0.01,
            kD = 0.0,
            kS = 0.0,
            MAX_OUTPUT_WITH_INTEGRAL = 1.0,
            FILTER_GAIN = 0.85,
            TURN_SETTLING_TIME = 0.3,
            TRANSLATION_SETTLING_TIME = 0.3;

    public static int FILTER_COUNT = 300;

    private double lastXCommand = 0.0, lastYCommand = 0.0, headingTarget;

    private final ElapsedTime turnSettlingTimer, translationSettlingTimer;

    private final PIDVAController headingController = new PIDVAController();

    public HeadingLockingMecanum(HardwareMap hw, double motorCPR, double motorRPM) {
        super(hw, motorCPR, motorRPM);
        turnSettlingTimer = new ElapsedTime();
        translationSettlingTimer = new ElapsedTime();
    }

    @Override
    public void readIMU() {
        headingController.setGains(
                new PIDGains(kP, kI, kD, MAX_OUTPUT_WITH_INTEGRAL),
                new FeedforwardGains(0.0, 0.0, kS)
        );
        headingController.derivFilter.setGains(
                FILTER_GAIN,
                FILTER_COUNT
        );
        super.readIMU();
    }

    @Override
    public void run(double xCommand, double yCommand, double turnCommand) {
        double voltage = batteryVoltageSensor.getVoltage();
        double scalar = 12.0 / voltage;
        boolean useManualInput = turnCommand != 0.0;

        boolean xStopped = (lastXCommand != 0) && (xCommand == 0);
        boolean yStopped = (lastYCommand != 0) && (yCommand == 0);
        if (xStopped || yStopped) translationSettlingTimer.reset();

        if (useManualInput) {
            turnCommand *= scalar;
            turnSettlingTimer.reset();
        }

        if (useManualInput || turnSettlingTimer.seconds() <= TURN_SETTLING_TIME) {
            headingTarget = getHeading();
        } else if (translationSettlingTimer.seconds() > TRANSLATION_SETTLING_TIME) {
            headingController.setTarget(new State(
                    normalizeAngle(headingTarget - getHeading()) + getHeading()
            ));
            turnCommand = -headingController.calculate(new State(getHeading()), voltage);
        }

        lastXCommand = xCommand;
        lastYCommand = yCommand;
        super.run(xCommand * scalar, yCommand * scalar, turnCommand);
    }

    public void setTargetHeading(double angle) {
        headingController.setTarget(new State(normalizeAngle(angle)));
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
        telemetry.addData("Drivetrain target heading", headingTarget);
        telemetry.addLine();
        telemetry.addData("Drivetrain heading error derivative (ticks/s)", headingController.getErrorDerivative());
    }
}
