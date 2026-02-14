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
import com.irurueta.sorting.Sorter
import com.irurueta.units.Measurement
import kotlin.math.ceil
import kotlin.math.min

/**
 * Median averaging filter.
 * Computes median values using a filter with a length equal to:
 * ceil(1.0 / dt * timeConstant), where dt is the time interval between samples.
 * For typical default values:
 * dt = 0.02 (%0Hz) and timeConstant = 0.1.
 * Consequently FIR length is 5 samples.
 *
 * @property timeConstant a constant relative to the period between consecutive samples to
 * determine the cut frequency of the low-pass filter. If not specified by default this is 0.1.
 */
class MedianAveragingFilter<U : Enum<*>, M : Measurement<U>, T : Triad<U, M, T>>(
    timeConstant: Double = DEFAULT_TIME_CONSTANT
) : AveragingFilter<U, M, T>(timeConstant) {

    /**
     * Last set of measurements within the FIR window length being averaged.
     */
    private val values = ArrayDeque<T>()

    /**
     * Sorter to compute median values.
     */
    private val sorter = Sorter.create<Double>()

    /**
     * Reused Array to contain values to be used for median computation.
     */
    private var windowedValues: DoubleArray? = null

    /**
     * Copy constructor.
     *
     * @param input filter instance to copy from.
     */
    constructor(input: MedianAveragingFilter<U, M, T>) : this() {
        copyFrom(input)
    }

    /**
     * Makes a deep copy from provided filter into this instance.
     *
     * @param input filter instance to copy from.
     */
    fun copyFrom(input: MedianAveragingFilter<U, M, T>) {
        super.copyFrom(input)

        values.clear()
        for (value in input.values) {
            values.addLast(value.copy())
        }
    }

    /**
     * Makes a deep copy to provided filter from this instance.
     *
     * @param output filter instance to copy to.
     */
    fun copyTo(output: MedianAveragingFilter<U, M, T>) {
        output.copyFrom(this)
    }

    /**
     * Resets this filter to its initial state.
     */
    override fun reset() {
        super.reset()
        values.clear()
        windowedValues = null
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

        val freq = 1.0 / intervalSeconds

        val filterWindow = ceil(freq * timeConstant).toInt()

        var windowedValues = this.windowedValues
        if (windowedValues == null || windowedValues.size != filterWindow) {
            val valuesSize = min(filterWindow, values.size + 1)
            windowedValues = DoubleArray(valuesSize)
            this.windowedValues = windowedValues
        }

        val valueX = input.valueX
        val valueY = input.valueY
        val valueZ = input.valueZ

        val value = input.copy()
        values.addLast(value)

        while (values.size > filterWindow) {
            values.removeFirst()
        }

        if (values.size > 1) {
            computeMedian(output, input.unit)
        } else {
            output.setValueCoordinatesAndUnit(valueX, valueY, valueZ, input.unit)
        }

        return true
    }

    /**
     * Computes median values of samples within window.
     *
     * @param output measurement containing result of filtering.
     * @param unit output unit to be set
     */
    private fun computeMedian(output: T, unit: U) {
        val windowedValues = this.windowedValues ?: return

        // X coordinate
        for ((index, value) in values.withIndex()) {
            windowedValues[index] = value.valueX
        }
        output.valueX = sorter.median(windowedValues)

        // Y coordinate
        for ((index, value) in values.withIndex()) {
            windowedValues[index] = value.valueY
        }
        output.valueY = sorter.median(windowedValues)

        // Z coordinate
        for ((index, value) in values.withIndex()) {
            windowedValues[index] = value.valueZ
        }
        output.valueZ = sorter.median(windowedValues)

        output.unit = unit
    }
}