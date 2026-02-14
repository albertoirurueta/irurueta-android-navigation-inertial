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

import com.irurueta.units.TimeConverter

/**
 * Base class for an averaging filter that can be used to filter sensor data in estimators, such as
 * accelerometer samples to obtain gravity by low-pass filtering.
 */
abstract class AveragingFilter private constructor() {

    /**
     * Constructor.
     *
     * @param timeConstant a constant relative to the period between consecutive samples to
     * determine the cut frequency of the low-pass filter. If not specified by default this is 0.1
     */
    constructor(timeConstant: Double = DEFAULT_TIME_CONSTANT) : this() {
        this.timeConstant = timeConstant
    }

    /**
     * Timestamp of previous sample expressed in nanoseconds.
     */
    private var previousTimestamp: Long = -1L

    /**
     * Gets a constant relative to the period between consecutive samples to determine
     * the cut frequency of the low-pass filter. If not specified by default this is 0.1
     */
    var timeConstant: Double = DEFAULT_TIME_CONSTANT
        private set

    /**
     * Filters provided values.
     *
     * @param valueX x-coordinate of sample to be filtered.
     * @param valueY y-coordinate of sample to be filtered.
     * @param valueZ z-coordinate of sample to be filtered.
     * @param output array containing result of filtering. Must have length 3.
     * @param timestamp timestamp expressed in nano seconds.
     * @return true if result is reliable, false otherwise.
     * @throws IllegalArgumentException if provided output array does not have length 3.
     */
    @Throws(IllegalArgumentException::class)
    fun filter(
        valueX: Double,
        valueY: Double,
        valueZ: Double,
        output: DoubleArray,
        timestamp: Long
    ): Boolean {
        require(output.size == OUTPUT_LENGTH)

        return if (previousTimestamp >= 0) {
            val diff = timestamp - previousTimestamp
            val diffSeconds = TimeConverter.nanosecondToSecond(diff.toDouble())
            previousTimestamp = timestamp
            process(valueX, valueY, valueZ, output, diffSeconds)
        } else {
            previousTimestamp = timestamp
            false
        }
    }

    /**
     * Resets this filter to its initial state.
     */
    open fun reset() {
        previousTimestamp = -1L
    }

    /**
     * Makes a deep copy from provided filter into this instance.
     *
     * @param input filter instance to copy from.
     */
    protected fun copyFrom(input: AveragingFilter) {
        previousTimestamp = input.previousTimestamp
        timeConstant = input.timeConstant
    }

    /**
     * Makes a deep copy to provided filter from this instance.
     *
     * @param output filter instance to copy to.
     */
    protected fun copyTo(output: AveragingFilter) {
        output.copyFrom(this)
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
    protected abstract fun process(
        valueX: Double,
        valueY: Double,
        valueZ: Double,
        output: DoubleArray,
        intervalSeconds: Double
    ): Boolean

    companion object {
        /**
         * Default time constant.
         */
        const val DEFAULT_TIME_CONSTANT = 0.1

        /**
         * Required length of provided output array obtained after filtering with implementations of
         * this filter.
         */
        const val OUTPUT_LENGTH = 3
    }
}