package org.firstinspires.ftc.teamcode.control.controllers.coefficients

data class PIDGains @JvmOverloads constructor(
    val kP: Double,
    val kI: Double,
    val kD: Double,
    val maxOutputWithIntegral: Double = Double.POSITIVE_INFINITY
)
