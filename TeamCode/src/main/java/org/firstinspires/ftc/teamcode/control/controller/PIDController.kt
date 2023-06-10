package org.firstinspires.ftc.teamcode.control.controller

import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.control.filter.FIRLowPassFilter
import kotlin.math.sign

/**
 * PID controller
 */
class PIDController
/**
 * @param kP proportional gain
 * @param kI integral gain
 * @param kD derivative gain
 * @param filterGain derivative filter smoothness, increases over the interval 0 ≤ x < 1
 */
@JvmOverloads constructor(
    private var kP: Double,
    private var kI: Double,
    private var kD: Double,
    private val filterGain: Double = 0.8,
    private val filterCount: Int = 5
) {
    private val dtTimer = ElapsedTime()
    var lastError = 0.0
    var errorSum = 0.0
    var errorDeriv = 0.0
    var integrate = true

    private val derivFilter: FIRLowPassFilter = FIRLowPassFilter(filterGain, filterCount)

    fun setGains(
        kP: Double,
        kI: Double,
        kD: Double,
        filterGain: Double = this.filterGain,
        filterCount: Int = this.filterCount
    ) {
        this.kP = kP
        this.kI = kI
        this.kD = kD
        derivFilter.setGains(filterGain, filterCount)
    }

    /**
     * Target position (the controller setpoint).
     */
    var targetPosition: Double = 0.0

    /**
     * Run a single iteration of the controller.
     *
     * @param measuredPosition measured position
     */
    fun update(measuredPosition: Double): Double {
        val error = targetPosition - measuredPosition
        val dt = dtTimer.seconds()
        return if (dt == 0.0) {
            lastError = error
            0.0
        } else {
            dtTimer.reset()
            errorDeriv = derivFilter.getEstimate((error - lastError) / dt)

            if (integrate) errorSum += 0.5 * (error + lastError) * dt
            if (sign(error) != sign(lastError)) resetIntegral()

            lastError = error

            (kP * error) + (kI * errorSum) + (kD * errorDeriv)
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
