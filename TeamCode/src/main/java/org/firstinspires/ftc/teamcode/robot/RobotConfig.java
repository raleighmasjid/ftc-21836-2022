package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;

@Config
public class RobotConfig {
    public static double
            HEIGHT_FLOOR = 0,
            HEIGHT_2_CONES = 1.35,
            HEIGHT_LOW = 6.2,
            HEIGHT_MEDIUM = 16.4,
            HEIGHT_TALL = 26.6,
            HEIGHT_1_STAGE = 9.6,
            LIFT_kG_4 = 0.18,
            LIFT_kG_3 = 0.14,
            LIFT_kP = 0.15,
            LIFT_kI = 0.21,
            LIFT_kD = 0.0,
            LIFT_kV_UP = 0.019,
            LIFT_kA_UP = 0.001,
            LIFT_kV_DOWN = 0.0087,
            LIFT_kA_DOWN = 0.0005,
            LIFT_kS = 0.0,
            LIFT_FILTER_GAIN_kD = 0.98,
            LIFT_FILTER_GAIN_VELO = 0.8,
            LIFT_FILTER_GAIN_ACCEL = 0.8,
            LIFT_MAX_VELO = 46.6,
            LIFT_MAX_ACCEL = 189.16,
            LIFT_MAX_JERK = 600,
            LIFT_MAX_PID_OUTPUT_WITH_INTEGRAL = 0.6,
            LIFT_TOLERANCE_POS = 0.15843625,
            LIFT_INCHES_PER_TICK = 0.03168725,
            SCALE_PRECISION_MODE = 0.3,
            ANGLE_CLAW_CLOSED = 0,
            ANGLE_CLAW_OPEN = 62,
            ANGLE_ARMS_DOWN = 100,
            ANGLE_PIVOT_FRONT = 17,
            ANGLE_PIVOT_BACK = 216,
            ANGLE_PASS_FRONT = 8,
            ANGLE_PASS_TILT_OFFSET = 45,
            ANGLE_PASS_BACK = 310,
            ANGLE_PASS_MINI_TILT_OFFSET = 17,
            ANGLE_PIVOT_POS = (ANGLE_PASS_BACK - ANGLE_PASS_FRONT) * 0.5,
            PASS_PIVOT_POS_TOLERANCE = 30,
            PASS_MAX_VELO = 600,
            PASS_MAX_ACCEL = 3000,
            PASS_MAX_JERK = 7000,
            TIME_CLAW = 0.0,
            TIME_LIFT_MEDIUM = 0.8,
            TIME_LIFT_TALL = 1;
    public static int
            LIFT_FILTER_COUNT_kD = 5,
            LIFT_FILTER_COUNT_VELO = 300,
            LIFT_FILTER_COUNT_ACCEL = 300;
}
