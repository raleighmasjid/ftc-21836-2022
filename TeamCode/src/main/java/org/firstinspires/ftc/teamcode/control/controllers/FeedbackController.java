package org.firstinspires.ftc.teamcode.control.controllers;

import com.acmerobotics.roadrunner.profile.MotionState;

public interface FeedbackController extends Controller {
    double calculate(MotionState measuredState);
}
