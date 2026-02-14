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
import com.irurueta.units.TimeConverter

/**
 * Base class for an averaging filter that can be used to filter sensor data in estimators, such as
 * accelerometer samples to obtain gravity by low-pass filtering.
 *
 * @param <T> type of triad to be filtered (usually an AccelerationTriad).
 * @see Triad
 * @see com.irurueta.navigation.inertial.calibration.AccelerationTriad
 */
abstract class AveragingFilter<U : Enum<*>, M : Measurement<U>, T : Triad<U, M, T>>
private constructor() {

    /**
     * Constructor.
     *
     * @param timeConstant a constant relative to the period between consecutive samples to
     * determine the cut frequency of the low-pass filter. If not specified by default this is 0.1.
     *
     * @throws IllegalArgumentException if provided value is zero or negative.
     */
    @Throws(IllegalArgumentException::class)
    constructor(timeConstant: Double = DEFAULT_TIME_CONSTANT) : this() {
        this.timeConstant = timeConstant
    }

    /**
     * Timestamp of previous sample expressed in nanoseconds.
     */
    private var previousTimestamp: Long = -1L

    /**
     * Gets a constant relative to the period between consecutive samples to determine
     * the cut frequency of the low-pass filter. If not specified by default this is 0.1.
     *
     * @throws IllegalArgumentException if provided value is zero or negative.
     */
    var timeConstant: Double = DEFAULT_TIME_CONSTANT
        @Throws(IllegalArgumentException::class)
        private set(value) {
            require(value > 0.0)
            field = value
        }

    /**
     * Filters provided triad.
     *
     * @param input input triad to be filtered.
     * @param output triad containing result of filtering.
     * @param timestamp timestamp expressed in nano seconds.
     * @return true if result is reliable, false otherwise.
     */
    fun filter(input: T, output: T, timestamp: Long): Boolean {
        return if (previousTimestamp >= 0) {
            val diff = timestamp - previousTimestamp
            val diffSeconds = TimeConverter.nanosecondToSecond(diff.toDouble())
            previousTimestamp = timestamp
            process(input, output, diffSeconds)
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
    protected fun <F : AveragingFilter<U, M, T>> copyFrom(input: F) {
        previousTimestamp = input.previousTimestamp
        timeConstant = input.timeConstant
    }

    /**
     * Makes a deep copy to provided filter from this instance.
     *
     * @param output filter instance to copy to.
     */
    protected fun <F : AveragingFilter<U, M, T>> copyTo(output: F) {
        output.copyFrom(this)
    }

    /**
     * Internal method to be implemented by subclasses and in charge of processing a single triad
     * of measurements.
     *
     * @param input input triad to be filtered.
     * @param output triad containing result of filtering.
     * @param intervalSeconds time interval between consecutive samples expressed in seconds.
     * @return true if result is reliable, false otherwise.
     */
    protected abstract fun process(input: T, output: T, intervalSeconds: Double): Boolean

    companion object {
        /**
         * Default time constant.
         */
        const val DEFAULT_TIME_CONSTANT = 0.1
    }
}