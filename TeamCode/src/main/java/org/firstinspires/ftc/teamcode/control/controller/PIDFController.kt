package org.firstinspires.ftc.teamcode.control.controller

import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sign

/**
 * PID controller with various feedforward components.
 */
class PIDFController
/**
 * Constructor for [PIDFController]. [kV], [kA], and [kStatic] are designed for DC motor feedforward
 * control (the most common kind of feedforward in FTC).
 *
 * @param kP proportional gain
 * @param kI integral gain
 * @param kD derivative gain
 * @param kV feedforward velocity gain
 * @param kA feedforward acceleration gain
 * @param kStatic additive feedforward constant
 * @param filterGain derivative filter weight, 0 = unsmoothed, 0 < x < 1 increasingly smoothed, 1 = broken
 * @param maxIntegrationVelocity max velocity that integral path will continue integration
 */
@JvmOverloads constructor(
    kP: Double = 0.0,
    kI: Double = 0.0,
    kD: Double = 0.0,
    filterGain: Double = 0.8,
    kV: Double = 0.0,
    kA: Double = 0.0,
    kStatic: Double = 0.0,
    var maxIntegrationVelocity: Double = 1.0
) {
    private var outputBounded: Boolean = false
    private var minOutput: Double = 0.0
    private var maxOutput: Double = 0.0

    val PID: PIDController = PIDController(kP, kI, kD, filterGain)
    val Feedforward: FeedforwardController = FeedforwardController(kV, kA, kStatic)

    /**
     * Sets bounds on the output of the controller.
     *
     * @param min minimum output
     * @param max maximum output
     */
    fun setOutputBounds(min: Double, max: Double) {
        if (min != max) {
            outputBounded = true
            minOutput = min(min, max)
            maxOutput = max(min, max)
            maxIntegrationVelocity = min(maxOutput, maxIntegrationVelocity)
        }
    }

    /**
     * Run a single iteration of the controller.
     *
     * @param measuredPosition measured position
     */
    fun update(measuredPosition: Double): Double {
        val PIDCommand: Double = PID.update(measuredPosition)
        val FFCommand: Double = Feedforward.update(PIDCommand)
        val output = PIDCommand + FFCommand

        PID.integrate =
            abs(output) < maxIntegrationVelocity || sign(output) != sign(PID.lastError)

        return if (outputBounded) max(minOutput, min(output, maxOutput)) else output
    }
}
