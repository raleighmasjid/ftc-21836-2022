package org.firstinspires.ftc.teamcode.control.controllers;

import org.firstinspires.ftc.teamcode.control.State;

public interface FeedbackController extends Controller {
    double calculate(State measurement);
}
