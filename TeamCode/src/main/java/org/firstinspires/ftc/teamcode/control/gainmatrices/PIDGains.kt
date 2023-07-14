package org.firstinspires.ftc.teamcode.control.gainmatrices

data class PIDGains @JvmOverloads constructor(
    @JvmField var kP: Double = 0.0,
    @JvmField var kI: Double = 0.0,
    @JvmField var kD: Double = 0.0,
    @JvmField var maxOutputWithIntegral: Double = Double.POSITIVE_INFINITY,
)
