package org.firstinspires.ftc.teamcode.control.filter;

public interface Filter {

    double getEstimate(double newValue);

    void clearMemory();
}
