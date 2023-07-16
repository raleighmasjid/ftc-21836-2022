package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.control.PIDController;

@Config
public class Lift {

    public static double
            kP = 0,
            kI = 0,
            kD = 0,
            kG = 0.1;

    private final MotorEx[] motors;
    private final HardwareMap hardwareMap;
    private final PIDController controller = new PIDController();

    private double currentPosition = 0.0;

    enum Position {
        LOW, MEDIUM, TALL, FLOOR, TWO, THREE, FOUR, FIVE
    }

    private MotorEx getMotor(String name) {
        return new MotorEx(hardwareMap, name, 145.1, 1150.0);
    }

    public Lift(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        motors = new MotorEx[]{
                getMotor("lift motor 2"),
                getMotor("lift motor 1"),
                getMotor("lift motor 3")
        };
        motors[1].setInverted(true);
        motors[2].setInverted(true);
    }

    public void setTargetPosition(double targetPosition) {
        controller.setTarget(targetPosition);
    }

    public void runToPosition() {
        currentPosition = motors[0].encoder.getPosition();
        controller.setGains(kP, kI, kD);
        run(controller.calculate(currentPosition));
    }

    public void run(double power) {
        for (MotorEx motor : motors) {
            motor.set(power + kG);
        }
    }

    public void printTelemetry(MultipleTelemetry telemetry) {
        telemetry.addData("Lift position", currentPosition);
    }
}
