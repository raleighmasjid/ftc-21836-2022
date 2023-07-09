package org.firstinspires.ftc.teamcode.control.controllers;

import com.acmerobotics.roadrunner.profile.MotionState;

import org.firstinspires.ftc.teamcode.control.MotionProfiler;

public class ProfiledController extends JointController implements FeedbackController {

    public MotionProfiler profiler = new MotionProfiler();

    public ProfiledController(FeedbackController feedback, FeedforwardController feedforward) {
        super(feedback, feedforward);
    }

    public double calculate(MotionState measuredState, double voltage) {
        super.setTarget(profiler.update());
        return super.calculate(measuredState, voltage);
    }

    public double calculate(MotionState measuredState) {
        return this.calculate(measuredState, 12.0);
    }
}
