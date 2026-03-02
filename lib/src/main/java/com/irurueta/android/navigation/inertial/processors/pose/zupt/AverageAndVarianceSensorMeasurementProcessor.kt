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

package com.irurueta.android.navigation.inertial.processors.pose.zupt

import com.irurueta.android.navigation.inertial.collectors.measurements.SensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.TriadConvertible
import com.irurueta.navigation.inertial.calibration.Triad
import kotlin.math.sqrt

/**
 * Processor to compute average and variance of sensor measurements.
 *
 * @param windowNanoseconds amount of time to take measurements into account to compute variance and
 * average. Value is expressed in nanoseconds.
 * @throws IllegalArgumentException if provided value is negative.
 * @param M type of sensor measurement.
 * @param T type of triad.
 */
class AverageAndVarianceSensorMeasurementProcessor<M, T>(
    windowNanoseconds: Long = DEFAULT_WINDOW_NANOSECONDS
) where M : SensorMeasurement<M>, T : Triad<*, *, T>, M : TriadConvertible<T> {
    /**
     * Triad being reused to convert measurements to.
     */
    private var triad: T? = null

    /**
     * Set of measurements within the FIR window length being averaged.
     */
    private var measurements = ArrayDeque<M>()

    /**
     * Amount of time to take measurements into account to compute variance and average of
     * measurements. Value is expressed in nanoseconds.
     *
     * @throws IllegalArgumentException if provided value is negative.
     */
    var windowNanoseconds: Long = windowNanoseconds
        @Throws(IllegalArgumentException::class)
        set(value) {
            require(value >= 0)
            field = value
        }

    /**
     * Number of measurements being averaged.
     */
    var count: Int = 0
        private set

    /**
     * Average value.
     */
    var average: Double? = null
        private set

    /**
     * Variance of measurements.
     */
    var variance: Double? = null
        private set

    /**
     * Processes provided measurement.
     *
     * @param measurement measurement to process.
     */
    fun process(measurement: M) {
        val timestamp = measurement.timestamp
        // add copy of new measurement
        measurements.addLast(measurement.copy())

        // remove old measurements
        val timestampLimit = timestamp - windowNanoseconds
        measurements.removeIf { m -> m.timestamp < timestampLimit }

        // compute average
        var averageX = 0.0
        var averageY = 0.0
        var averageZ = 0.0
        var counter = 0
        for (m in measurements) {
            val t = convertAndRetuseTriad(m)
            averageX += t.valueX
            averageY += t.valueY
            averageZ += t.valueZ
            counter++
        }

        count = counter

        averageX /= counter
        averageY /= counter
        averageZ /= counter

        average = sqrt(averageX * averageX + averageY * averageY + averageZ * averageZ)

        var varianceX = 0.0
        var varianceY = 0.0
        var varianceZ = 0.0
        for (m in measurements) {
            val t = convertAndRetuseTriad(m)
            val diffX = t.valueX - averageX
            val diffY = t.valueY - averageY
            val diffZ = t.valueZ - averageZ

            val diffX2 = diffX * diffX
            val diffY2 = diffY * diffY
            val diffZ2 = diffZ * diffZ

            varianceX += diffX2
            varianceY += diffY2
            varianceZ += diffZ2
        }

        varianceX /= counter
        varianceY /= counter
        varianceZ /= counter

        variance = varianceX + varianceY + varianceZ
    }

    /**
     * Resets processor.
     */
    fun reset() {
        measurements.clear()
        count = 0
        average = null
        variance = null
    }

    /**
     * Converts provided measurement to triad and returns it.
     *
     * @param measurement measurement to convert.
     * @return converted triad.
     */
    private fun convertAndRetuseTriad(measurement: M): T {
        val t = triad
        if (t != null) {
            measurement.toTriad(t)
            return t
        } else {
            val t2 = measurement.toTriad()
            triad = t2
            return t2
        }
    }

    // Initialize
    init {
        this.windowNanoseconds = windowNanoseconds
    }

    companion object {
        /**
         * Default amount of time to take measurements into account to compute variance
         * of accelerometer and gyroscope measurements.
         * By default, this is 200 ms (0.2s), which is about 10 samples at a rate of 50Hz.
         */
        const val DEFAULT_WINDOW_NANOSECONDS = 200_000_000L
    }
}