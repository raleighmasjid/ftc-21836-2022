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

    protected double ANGLE_OPEN, ANGLE_CLOSED;

    protected boolean closed = false;

    /**
     * Initialize claw
     */
    public SimpleClaw(SimpleServo servo, double ANGLE_OPEN, double ANGLE_CLOSED) {
        this.servo = servo;
        updateAngles(ANGLE_OPEN, ANGLE_CLOSED);
    }

    public void updateAngles(double ANGLE_OPEN, double ANGLE_CLOSED) {
        this.ANGLE_OPEN = ANGLE_OPEN;
        this.ANGLE_CLOSED = ANGLE_CLOSED;
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
        servo.turnToAngle(closed ? ANGLE_CLOSED : ANGLE_OPEN);
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
