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

import com.irurueta.navigation.inertial.calibration.TimeIntervalEstimator
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
     * Estimator to obtain average time interval between samples.
     */
    protected val timeIntervalEstimator = TimeIntervalEstimator(Integer.MAX_VALUE)

    /**
     * Timestamp of first sample expressed in nanoseconds.
     */
    private var initialTimestamp: Long = 0L

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
    ) : Boolean {
        require(output.size == OUTPUT_LENGTH)

        val diff = timestamp - initialTimestamp
        val diffSeconds = TimeConverter.nanosecondToSecond(diff.toDouble())
        if (timeIntervalEstimator.numberOfProcessedSamples == 0) {
            initialTimestamp = timestamp
        }
        timeIntervalEstimator.addTimestamp(diffSeconds)

        return process(valueX, valueY, valueZ, output, timestamp)
    }

    /**
     * Resets this filter to its initial state.
     */
    open fun reset() {
        timeIntervalEstimator.reset()
        initialTimestamp = 0L
    }

    /**
     * Makes a deep copy from provided filter into this instance.
     *
     * @param input filter instance to copy from.
     */
    protected fun copyFrom(input: AveragingFilter) {
        timeIntervalEstimator.copyFrom(input.timeIntervalEstimator)
        initialTimestamp = input.initialTimestamp
        timeConstant = input.timeConstant
    }

    /**
     * Makes a deep copy to provided filtr from this instance.
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
     * @param timestamp timestamp expressed in nano seconds.
     * @return true if result is reliable, false otherwise.
     */
    protected abstract fun process(
        valueX: Double,
        valueY: Double,
        valueZ: Double,
        output: DoubleArray,
        timestamp: Long
    ) : Boolean

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