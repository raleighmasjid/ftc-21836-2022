package org.firstinspires.ftc.teamcode.control.controllers;

import org.firstinspires.ftc.teamcode.control.MotionProfiler;

public class ProfiledPIDF extends PIDFController {

    public MotionProfiler profiler = new MotionProfiler();

    /**
     * Initialize fields <p>
     *
     * @param pid         PID feedback controller
     * @param feedforward kV-kA-kS feedforward controller
     */
    public ProfiledPIDF(PIDController pid, FeedforwardController feedforward) {
        super(pid, feedforward);
    }

    public ProfiledPIDF() {
        this(new PIDController(), new FeedforwardController());
    }

    @Override
    public double update(double currentPosition, double voltage) {
        profiler.update();
        setTargetState(profiler.getX(), profiler.getV(), profiler.getA());
        return super.update(currentPosition, voltage);
    }

    @Override
    public double update(double currentPosition) {
        return this.update(currentPosition, 12.0);
    }
}
