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

import com.irurueta.sorting.Sorter
import java.util.*
import kotlin.collections.ArrayDeque
import kotlin.math.ceil
import kotlin.math.min

/**
 * Median averaging filter.
 * Computes median values using a filter with a length equal to:
 * ceil(1.0 / dt * timeConstant), where dt is the time interval between samples.
 * For typical default values:
 * dt = 0.02 (%0Hz) and timeConstant = 0.1.
 * Consequently FIR length is 5 samples.
 */
class MedianAveragingFilter(timeConstant: Double = DEFAULT_TIME_CONSTANT) :
    AveragingFilter(timeConstant) {
    /**
     * Last set of measurements within the FIR window length being averaged.
     */
    private val values = ArrayDeque<DoubleArray>()

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
    constructor(input: MedianAveragingFilter) : this() {
        copyFrom(input)
    }

    /**
     * Makes a deep copy from provided filter into this instance.
     *
     * @param input filter instance to copy from.
     */
    fun copyFrom(input: MedianAveragingFilter) {
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
    fun copyTo(output: MedianAveragingFilter) {
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
    ) : Boolean {
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

        values.addLast(doubleArrayOf(valueX, valueY, valueZ))

        while (values.size > filterWindow) {
            values.removeFirst()
        }

        if (values.size > 1) {
            computeMedian(output)
        } else {
            output[0] = valueX
            output[1] = valueY
            output[2] = valueZ
        }

        return true
    }

    /**
     * Computes median values of samples within window.
     *
     * @param output array containing result of filtering.
     */
    private fun computeMedian(output: DoubleArray) {
        val windowedValues = this.windowedValues ?: return

        Arrays.fill(output, 0.0)

        for (i in output.indices) {
            for ((index, value) in values.withIndex()) {
                windowedValues[index] = value[i]
            }

            output[i] = sorter.median(windowedValues, 0, values.size)
        }
    }
}