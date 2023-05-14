package org.firstinspires.ftc.teamcode.control.controller

import com.acmerobotics.roadrunner.util.NanoClock
import org.firstinspires.ftc.teamcode.robot.RobotConfig
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
 * @param filterGain derivative filter weight, 0 = unsmoothed, 0 < x < 1 increasingly smoothed, 1 = broken
 * @param kV feedforward velocity gain
 * @param kA feedforward acceleration gain
 * @param kStatic additive feedforward constant
 * @param maxIntegrationVelocity max velocity that integral path will continue integration
 * @param clock clock
 */
@JvmOverloads constructor(
    private var kP: Double,
    private var kI: Double,
    private var kD: Double,
    private var filterGain: Double = 0.8,
    private var kV: Double = 0.0,
    private var kA: Double = 0.0,
    private var kStatic: Double = 0.0,
    private var maxIntegrationVelocity: Double = 1.0,
    private val clock: NanoClock = NanoClock.system()
) {
    private var outputBounded: Boolean = false
    private var minOutput: Double = 0.0
    private var maxOutput: Double = 0.0

    private val PID: PIDController = PIDController(kP, kI, kD, filterGain)
    private val FF: FeedForwardController = FeedForwardController(kV, kA, kStatic)

    fun setGains(
        kP: Double,
        kI: Double,
        kD: Double,
        filterGain: Double,
        kV: Double,
        kA: Double,
        kStatic: Double,
        maxIntegrationVelocity: Double = this.maxIntegrationVelocity
    ) {
        PID.setGains(kP, kI, kD, filterGain)
        FF.setGains(kV, kA, kStatic)
        this.maxIntegrationVelocity = maxIntegrationVelocity
    }

    /**
     * Target position (that is, the controller setpoint).
     */
    var targetPosition: Double = 0.0

    /**
     * Target velocity.
     */
    var targetVelocity: Double = 0.0

    /**
     * Target acceleration.
     */
    var targetAcceleration: Double = 0.0

    /**
     * Error computed in the last call to [update].
     */
    private var lastError: Double = 0.0

    /**
     * Sets bounds on the output of the controller.
     *
     * @param min minimum output
     * @param max maximum output
     */
    fun setOutputBounds(min: Double, max: Double) {
        if (min < max) {
            outputBounded = true
            minOutput = min
            maxOutput = max
            maxIntegrationVelocity = min(maxOutput, maxIntegrationVelocity)
        }
    }

    private fun getPositionError(measuredPosition: Double): Double {
        return targetPosition - measuredPosition
    }

    fun atTargetPosition(measuredPosition: Double): Boolean {
        return PID.atTargetPosition(measuredPosition)
    }

    fun setPositionTolerance(tolerance: Double) {
        PID.positionTolerance = tolerance
    }

    /**
     * Run a single iteration of the controller.
     *
     * @param measuredPosition measured position (feedback)
     * @param measuredVelocity measured velocity
     */
    @JvmOverloads
    fun update(
        measuredPosition: Double,
        measuredVelocity: Double? = null
    ): Double {
        val PIDCommand: Double = PID.update(measuredPosition)
        val FFCommand: Double = FF.update(PIDCommand)
        val output = PIDCommand + FFCommand

        PID.integrate =
            Math.abs(output) < RobotConfig.LIFT_INTEGRATION_MAX_VELO || sign(output) != sign(PID.lastError)

        return if (outputBounded) max(minOutput, min(output, maxOutput)) else output
    }

    /**
     * Reset the controller's integral sum.
     */
    fun reset() {
        PID.errorSum = 0.0
        PID.lastError = 0.0
        PID.integrate = true
    }
}
