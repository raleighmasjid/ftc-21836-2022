package org.firstinspires.ftc.teamcode.control.controllers;

import com.acmerobotics.roadrunner.profile.MotionState;

import org.firstinspires.ftc.teamcode.control.MotionProfiler;

public class ProfiledJointController extends JointController implements FeedbackController {

    public MotionProfiler profiler = new MotionProfiler();

    public ProfiledJointController(FeedbackController feedback, FeedforwardController feedforward) {
        super(feedback, feedforward);
    }

    @Override
    public void setTarget(MotionState targetState) {
        profiler.setTarget(profiler.update(), targetState);
    }

    public void setTarget(MotionState measuredState, MotionState targetState) {
        profiler.setTarget(measuredState, targetState);
    }

    @Override
    public double calculate(MotionState measuredState, double voltage) {
        super.setTarget(profiler.update());
        return super.calculate(measuredState, voltage);
    }

    @Override
    public double calculate(MotionState measuredState) {
        return this.calculate(measuredState, 12.0);
    }
}
