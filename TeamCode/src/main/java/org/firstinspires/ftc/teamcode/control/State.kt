package org.firstinspires.ftc.teamcode.control

data class State @JvmOverloads constructor(
    val x: Double,
    val v: Double = 0.0,
    val a: Double = 0.0
)
