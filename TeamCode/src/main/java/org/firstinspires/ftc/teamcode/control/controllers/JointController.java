package org.firstinspires.ftc.teamcode.control.controllers;

import com.acmerobotics.roadrunner.profile.MotionState;

/**
 * Wrapper class combining a PID controller and a kV-kA-kS feedforward controller.
 */
public class JointController implements FeedbackController {
    public final FeedbackController feedback;
    public final FeedforwardController feedforward;

    /**
     * @param feedback    PID feedback controller
     * @param feedforward kV-kA-kS feedforward controller
     */
    public JointController(FeedbackController feedback, FeedforwardController feedforward) {
        this.feedback = feedback;
        this.feedforward = feedforward;
    }

    public void setTarget(MotionState targetState) {
        feedback.setTarget(targetState);
        feedforward.setTarget(targetState);
    }

    public double calculate(MotionState measuredState) {
        return calculate(measuredState, 12.0);
    }

    /**
     * Run a single iteration of the controller.
     *
     * @param voltage measured battery voltage (for feedforward voltage correction)
     */
    public double calculate(MotionState measuredState, double voltage) {
        double feedbackOutput = feedback.calculate(measuredState);
        return feedbackOutput + feedforward.calculate(voltage, feedbackOutput);
    }
}
