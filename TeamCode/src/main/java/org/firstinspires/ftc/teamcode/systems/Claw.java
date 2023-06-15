package org.firstinspires.ftc.teamcode.systems;


import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.SimpleServo;

/**
 * Single-servo claw
 *
 * @author Arshad Anas
 * @since 2023/06/14
 */
public class Claw {

    private SimpleServo servo;

    private double openAngle, closedAngle;

    private boolean isOpen = true;

    /**
     * Initialize claw
     */
    public Claw(SimpleServo servo, double openAngle, double closedAngle) {
        this.servo = servo;
        setAngles(openAngle, closedAngle);
    }

    public void setAngles(double openAngle, double closedAngle) {
        this.openAngle = openAngle;
        this.closedAngle = closedAngle;
    }

    /**
     * Toggles the value of {@link #isOpen}
     */
    public void toggle() {
        setState(!isOpen);
    }

    /**
     * Set state of the claw
     *
     * @param open True if open; false if closed
     */
    public void setState(boolean open) {
        isOpen = open;
    }

    /**
     * Get state of the claw;
     * true if open, false if closed
     */
    public boolean getState() {
        return isOpen;
    }

    /**
     * Holds {@link #servo} position
     */
    public void run() {
        servo.turnToAngle(isOpen ? openAngle : closedAngle);
    }

    /**
     * Print status
     *
     * @param telemetry MultipleTelemetry object to add data to
     */
    public void printTelemetry(MultipleTelemetry telemetry) {
        telemetry.addData("Claw is", isOpen ? "open" : "closed");
    }
}
