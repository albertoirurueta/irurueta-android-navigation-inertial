/*
 * Copyright (C) 2026 Alberto Irurueta Carro (alberto@irurueta.com)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *         http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package com.irurueta.android.navigation.inertial.processors.filters

import com.irurueta.navigation.inertial.calibration.Triad
import com.irurueta.units.Measurement

/**
 * First order IIR averaging filter.
 * IIR (Infinite Impulse Response) filters of first order follow expressions like the one below:
 * output = a * input + b * prevOutput, where a = 1.0-b,
 * where:
 * b = e^-2*π*fc -> ln(b) = -2*π*fc -> fc = -ln(b) / (2*π) and fc is the cutoff frequency.
 * For typical default values:
 * dt = 0.02 (50 Hz) s, and timeConstant = 0.1, dt is the time interval between samples,
 * b = timeConstant / (timeConstant + dt) = 0.1 / (0.1 + 0.02)
 * b = 0.8333
 * Consequently fc = -ln(0.8333) / (2*π) = 0.02 Hz
 *
 * @property timeConstant a constant relative to the period between consecutive samples to
 * determine the cut frequency of the low-pass filter. If not specified by default this is 0.1.
 */
class LowPassAveragingFilter<U : Enum<*>, M : Measurement<U>, T : Triad<U, M, T>>(
    timeConstant: Double = DEFAULT_TIME_CONSTANT
) : AveragingFilter<U, M, T>(timeConstant) {

    /**
     * Copy constructor.
     *
     * @param input filter instance to copy from.
     */
    constructor(input: LowPassAveragingFilter<U, M, T>) : this() {
        copyFrom(input)
    }

    /**
     * Makes a deep copy from provided filter into this instance.
     *
     * @param input filter instance to copy from.
     */
    fun copyFrom(input: LowPassAveragingFilter<U, M, T>) {
        super.copyFrom(input)
    }

    /**
     * Makes a deep copy to provided filter from this instance.
     *
     * @param output filter instance to copy to.
     */
    fun copyTo(output: LowPassAveragingFilter<U, M, T>) {
        super.copyTo(output)
    }

    /**
     * Internal method to be in charge of processing a single triad of measurements.
     *
     * @param input input triad to be filtered.
     * @param output triad containing result of filtering. This triad is used as previous filter
     * state.
     * @param intervalSeconds time interval between consecutive samples expressed in seconds.
     * @return true if result is reliable, false otherwise.
     */
    override fun process(input: T, output: T, intervalSeconds: Double): Boolean {
        if (intervalSeconds <= 0.0) {
            return false
        }

        val alpha = timeConstant / (timeConstant + intervalSeconds)
        val oneMinusAlpha = 1.0 - alpha

        val inputX = input.valueX
        val inputY = input.valueY
        val inputZ = input.valueZ

        val prevX = output.valueX
        val prevY = output.valueY
        val prevZ = output.valueZ

        val outputX = alpha * prevX + oneMinusAlpha * inputX
        val outputY = alpha * prevY + oneMinusAlpha * inputY
        val outputZ = alpha * prevZ + oneMinusAlpha * inputZ

        output.setValueCoordinatesAndUnit(outputX, outputY, outputZ, input.unit)

        return true
    }
}
