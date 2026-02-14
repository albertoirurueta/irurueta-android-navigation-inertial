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

package com.irurueta.android.navigation.inertial.processors.attitude

import android.location.Location
import com.irurueta.android.navigation.inertial.collectors.measurements.AccelerometerSensorMeasurement
import com.irurueta.android.navigation.inertial.processors.filters.AveragingFilter
import com.irurueta.android.navigation.inertial.processors.filters.LowPassAveragingFilter
import com.irurueta.navigation.inertial.calibration.AccelerationTriad
import com.irurueta.units.Acceleration
import com.irurueta.units.AccelerationUnit

/**
 * Collects accelerometer measurements and uses a low-pass filter to assume it is the gravity
 * component of specific force and converts it to NED coordinates system.
 *
 * @property averagingFilter an averaging filter for accelerometer samples to obtain
 * sensed gravity component of specific force.
 * @property location current device location.
 * @property adjustGravityNorm indicates whether gravity norm must be adjusted to either Earth
 * standard norm, or norm at provided location. If no location is provided, this should only be
 * enabled when device is close to sea level.
 * @property processorListener listener to notify new gravity measurements.
 */
class AccelerometerGravityProcessor(
    val averagingFilter: AveragingFilter<AccelerationUnit, Acceleration, AccelerationTriad> = LowPassAveragingFilter(),
    location: Location? = null,
    adjustGravityNorm: Boolean = true,
    processorListener: OnProcessedListener<AccelerometerSensorMeasurement>? = null
) : BaseGravityProcessor<AccelerometerSensorMeasurement>(
    location,
    adjustGravityNorm,
    processorListener
) {
    /**
     * Accelerometer measurement being reused and containing measurements converted into NED
     * coordinates system.
     */
    private val nedMeasurement = AccelerometerSensorMeasurement()

    /**
     * Acceleration triad to be reused containing input values for averaging filter.
     */
    private val filterInput = AccelerationTriad()

    /**
     * Processes an accelerometer sensor measurement collected by a collector or a syncer.
     * Notice that this processor will convert provided measurements to NED coordinates system if
     * needed, and results will always be returned in NED coordinates system.
     *
     * @param measurement measurement expressed in ENU or NED coordinates system to be processed.
     * @param timestamp optional timestamp that can be provided to override timestamp associated to
     * accelerometer measurement. If not set, the timestamp from measurement is used.
     * @return true if a new gravity is estimated, false otherwise.
     */
    override fun process(
        measurement: AccelerometerSensorMeasurement,
        timestamp: Long
    ): Boolean {
        measurement.toNed(nedMeasurement)
        nedMeasurement.toTriad(filterInput)

        return if (averagingFilter.filter(filterInput, triad, timestamp)) {
            finishProcessingAndNotify(measurement, timestamp)
            true
        } else {
            false
        }
    }
}