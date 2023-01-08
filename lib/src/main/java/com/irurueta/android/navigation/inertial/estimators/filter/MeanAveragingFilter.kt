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
package com.irurueta.android.navigation.inertial.estimators.filter

import com.irurueta.algebra.ArrayUtils
import java.util.*
import kotlin.collections.ArrayDeque
import kotlin.math.ceil

/**
 * FIR mean averaging filter.
 * Computes average values using a FIR (Finite Impulse Response) filter with a length equal to:
 * ceil(1.0 / dt * timeConstant), where dt is the time interval between samples.
 * For typical default values:
 * dt = 0.02 (%0Hz) and timeConstant = 0.1.
 * Consequently FIR length is 5 samples.
 */
class MeanAveragingFilter(timeConstant: Double = DEFAULT_TIME_CONSTANT) :
    AveragingFilter(timeConstant) {

    /**
     * Last set of measurements within the FIR window length being averaged.
     */
    private val values = ArrayDeque<DoubleArray>()

    /**
     * Copy constructor.
     *
     * @param input filter instance to copy from.
     */
    constructor(input: MeanAveragingFilter) : this() {
        copyFrom(input)
    }

    /**
     * Makes a deep copy from provided filter into this instance.
     *
     * @param input filter instance to copy from.
     */
    fun copyFrom(input: MeanAveragingFilter) {
        super.copyFrom(input)

        values.clear()
        for (value in input.values) {
            values.addLast(value.copyOf())
        }
    }

    /**
     * Makes a deep copy to provided filter from this instance.
     *
     * @param output filter instance to copy to.
     */
    fun copyTo(output: MeanAveragingFilter) {
        output.copyFrom(this)
    }

    /**
     * Resets this filter to its initial state.
     */
    override fun reset() {
        super.reset()
        values.clear()
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

        val freq = 1.0 / intervalSeconds

        val filterWindow = ceil(freq * timeConstant).toInt()

        values.addLast(doubleArrayOf(valueX, valueY, valueZ))

        while (values.size > filterWindow) {
            values.removeFirst()
        }

        if (!values.isEmpty()) {
            computeMean(output)
        } else {
            output[0] = valueX
            output[1] = valueY
            output[2] = valueZ
        }

        return true
    }

    /**
     * Computes mean values of samples within window.
     *
     * @param output array containing result of filtering.
     */
    private fun computeMean(output: DoubleArray) {
        Arrays.fill(output, 0.0)

        for (value in values) {
            ArrayUtils.sum(output, value, output)
        }

        ArrayUtils.multiplyByScalar(output, 1.0 / values.size, output)
    }
}