package org.firstinspires.ftc.teamcode.control

data class State @JvmOverloads constructor(
    @JvmField val x: Double = 0.0,
    @JvmField val v: Double = 0.0,
    @JvmField val a: Double = 0.0,
    @JvmField val j: Double = 0.0,
)
