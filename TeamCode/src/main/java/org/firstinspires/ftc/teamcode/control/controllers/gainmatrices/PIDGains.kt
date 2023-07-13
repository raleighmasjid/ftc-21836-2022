package org.firstinspires.ftc.teamcode.control.controllers.gainmatrices

data class PIDGains @JvmOverloads constructor(
    val kP: Double = 0.0,
    val kI: Double = 0.0,
    val kD: Double = 0.0,
    val maxOutputWithIntegral: Double = Double.POSITIVE_INFINITY
)
