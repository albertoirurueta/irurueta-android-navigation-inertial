/*
 * Copyright (C) 2022 Alberto Irurueta Carro (alberto@irurueta.com)
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
package com.irurueta.android.navigation.inertial.old.estimators.filter

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
 */
class LowPassAveragingFilter(timeConstant: Double = DEFAULT_TIME_CONSTANT) :
    AveragingFilter(timeConstant) {

    /**
     * Copy constructor.
     *
     * @param input filter instance to copy from.
     */
    constructor(input: LowPassAveragingFilter) : this() {
        copyFrom(input)
    }

    /**
     * Makes a deep copy from provided filter into this instance.
     *
     * @param input filter instance to copy from.
     */
    fun copyFrom(input: LowPassAveragingFilter) {
        super.copyFrom(input)
    }

    /**
     * Makes a deep copy to provided filter from this instance.
     *
     * @param output filter instance to copy to.
     */
    fun copyTo(output: LowPassAveragingFilter) {
        super.copyTo(output)
    }

    /**
     * Internal method to be implemented by subclasses and in charge of processing a single
     * measurement.
     *
     * @param valueX x-coordinate of sample to be filtered.
     * @param valueY y-coordinate of sample to be filtered.
     * @param valueZ z-coordinate of sample to be filtered.
     * @param output array containing result of filtering. Must have length 3.
     * @param intervalSeconds time interval between consecutive samples expressed in seconds.
     * @return true if result is reliable, false otherwise.
     */
    override fun process(
        valueX: Double,
        valueY: Double,
        valueZ: Double,
        output: DoubleArray,
        intervalSeconds: Double
    ): Boolean {
        if (intervalSeconds <= 0.0) {
            return false
        }

        val alpha = timeConstant / (timeConstant + intervalSeconds)
        val oneMinusAlpha = 1.0 - alpha

        output[0] = alpha * output[0] + oneMinusAlpha * valueX
        output[1] = alpha * output[1] + oneMinusAlpha * valueY
        output[2] = alpha * output[2] + oneMinusAlpha * valueZ

        return true
    }
}