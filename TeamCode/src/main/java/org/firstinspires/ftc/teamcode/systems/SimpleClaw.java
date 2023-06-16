package org.firstinspires.ftc.teamcode.systems;


import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.SimpleServo;

/**
 * Single-servo claw
 *
 * @author Arshad Anas
 * @since 2023/06/14
 */
public class SimpleClaw {

    protected final SimpleServo servo;

    protected double openAngle, closedAngle;

    protected boolean closed = false;

    /**
     * Initialize claw
     */
    public SimpleClaw(SimpleServo servo, double openAngle, double closedAngle) {
        this.servo = servo;
        updateAngles(openAngle, closedAngle);
    }

    public void updateAngles(double openAngle, double closedAngle) {
        this.openAngle = openAngle;
        this.closedAngle = closedAngle;
    }

    /**
     * Toggles the state of the claw
     */
    public void toggle() {
        setClosed(!closed);
    }

    /**
     * Set state of the claw
     *
     * @param closed True if closed; false if open
     */
    public void setClosed(boolean closed) {
        this.closed = closed;
    }

    /**
     * Get state of the claw;
     * true if closed; false if open
     */
    public boolean getClosed() {
        return closed;
    }

    /**
     * Holds {@link #servo} position
     */
    public void run() {
        servo.turnToAngle(closed ? closedAngle : openAngle);
    }

    /**
     * Print status
     *
     * @param telemetry MultipleTelemetry object to add data to
     */
    public void printTelemetry(MultipleTelemetry telemetry) {
        telemetry.addData("Claw is", closed ? "closed" : "open");
    }
}
