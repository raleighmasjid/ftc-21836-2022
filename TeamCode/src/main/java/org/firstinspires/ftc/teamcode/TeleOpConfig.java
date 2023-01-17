package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.config.Config;

@Config
public class TeleOpConfig {
    public static double CLAW_RIGHT_CLOSED = 0.5;
    public static double CLAW_RIGHT_OPEN = 0.4;

    public static double HEIGHT_ONE = 0;
    public static double HEIGHT_TWO = 0;
    public static double HEIGHT_THREE = 0;
    public static double HEIGHT_FOUR = 0;
    public static double HEIGHT_FIVE = 0;

    public static double HEIGHT_GROUND = 250;
    public static double HEIGHT_LOW = 500;
    public static double HEIGHT_MEDIUM = 800;
    public static double HEIGHT_TALL = 1500;

    public static double LIFT_P = 0.01;
    public static double LIFT_I = 0;
    public static double LIFT_D = 0;
    public static double LIFT_F = 0;
    public static double LIFT_E_TOLERANCE = 5;
    public static double LIFT_V_TOLERANCE = 10;

    public static double pivotFront = 0.24;
    public static double pivotBack = Math.PI;
    public static double pass1Front = 6.28;
    public static double pass1Back = Math.PI;
    public static double pass2Front = 0.5;
    public static double pass2Back = Math.PI;
}