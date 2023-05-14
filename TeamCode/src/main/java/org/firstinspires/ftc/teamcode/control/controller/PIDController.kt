package org.firstinspires.ftc.teamcode.control.controller

import com.acmerobotics.roadrunner.util.NanoClock
import org.firstinspires.ftc.teamcode.control.filter.IIRLowPassFilter
import kotlin.math.abs
import kotlin.math.sign

/**
 * PID controller
 */
class PIDController
/**
 * @param kP proportional gain
 * @param kI integral gain
 * @param kD derivative gain
 * @param filterGain derivative filter weight, 0 = unsmoothed, 0 < x < 1 increasingly smoothed, 1 = broken
 * @param clock clock
 */
@JvmOverloads constructor(
    private var kP: Double,
    private var kI: Double,
    private var kD: Double,
    private var filterGain: Double = 0.8,
    private val clock: NanoClock = NanoClock.system()
) {
    private var errorSum: Double = 0.0
    var lastError: Double = 0.0
    private var lastUpdateTimestamp: Double = Double.NaN

    var errorDeriv: Double = 0.0
    var integrate: Boolean = true
    var positionTolerance: Double = 2.0

    private var derivFilter: IIRLowPassFilter =
        IIRLowPassFilter(
            filterGain
        )

    fun setGains(
        kP: Double,
        kI: Double,
        kD: Double,
        filterGain: Double = this.filterGain
    ) {
        this.kP = kP
        this.kI = kI
        this.kD = kD
        this.filterGain = filterGain
        derivFilter.setGain(filterGain)
    }

    /**
     * Target position (the controller setpoint).
     */
    var targetPosition: Double = 0.0

    private fun getPositionError(measuredPosition: Double): Double {
        return targetPosition - measuredPosition
    }

    fun atTargetPosition(measuredPosition: Double): Boolean {
        return abs(getPositionError(measuredPosition)) <= positionTolerance
    }

    /**
     * Run a single iteration of the controller.
     *
     * @param measuredPosition measured position
     */
    fun update(measuredPosition: Double): Double {
        val currentTimestamp = clock.seconds()
        val error = getPositionError(measuredPosition)
        val dy = error - lastError
        val currentFilterEstimate = derivFilter.getEstimate(dy)
        return if (lastUpdateTimestamp.isNaN()) {
            lastError = error
            lastUpdateTimestamp = currentTimestamp
            0.0
        } else {
            val dt = currentTimestamp - lastUpdateTimestamp
            errorDeriv = currentFilterEstimate / dt

            errorSum += if (integrate) (0.5 * (error + lastError) * dt) else 0.0

            if (sign(error) != sign(lastError)) resetIntegral()

            lastError = error
            lastUpdateTimestamp = currentTimestamp

            (kP * error + kI * errorSum + kD * errorDeriv)
        }
    }

    /**
     * Reset the controller's integral sum.
     */
    fun resetIntegral() {
        errorSum = 0.0
        lastError = 0.0
        integrate = true
    }
}
