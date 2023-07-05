package org.firstinspires.ftc.teamcode.control.filters;

public interface Filter {

    double getEstimate(double newValue);

    void clearMemory();
}
