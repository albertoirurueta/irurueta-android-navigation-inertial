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

package com.irurueta.android.navigation.inertial.processors.pose

import android.location.Location
import com.irurueta.android.navigation.inertial.collectors.measurements.AccelerometerSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.AttitudeAndAccelerometerSyncedSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.AttitudeSensorMeasurement
import com.irurueta.android.navigation.inertial.processors.attitude.AccelerometerGravityProcessor
import com.irurueta.android.navigation.inertial.processors.attitude.AttitudeProcessor
import com.irurueta.android.navigation.inertial.processors.filters.AveragingFilter
import com.irurueta.android.navigation.inertial.processors.filters.LowPassAveragingFilter
import com.irurueta.navigation.inertial.calibration.AccelerationTriad
import com.irurueta.navigation.inertial.calibration.SpeedTriad
import com.irurueta.units.Acceleration
import com.irurueta.units.AccelerationUnit

/**
 * Estimates relative (or absolute) pose from an unknown location.
 * This class estimates attitude by using Android's relative attitude sensor.
 * Accelerometer and gyroscope are then taken into account to update device position.
 *
 * @property initialSpeed initial device speed.
 * @property averagingFilter an averaging filter for accelerometer samples to obtain
 * sensed gravity component of specific force.
 * @property processorListener listener to notify new poses.
 */
class AttitudeRelativePoseProcessor(
    initialSpeed: SpeedTriad = SpeedTriad(),
    val averagingFilter: AveragingFilter<AccelerationUnit, Acceleration, AccelerationTriad> = LowPassAveragingFilter(),
    processorListener: OnProcessedListener? = null
) : BaseRelativePoseProcessor(initialSpeed, processorListener) {
    /**
     * Relative or absolute attitude processor.
     */
    private val attitudeProcessor = AttitudeProcessor()

    /**
     * Internal gravity processor from accelerometer measurements.
     */
    private val gravityProcessor = AccelerometerGravityProcessor(averagingFilter)

    /**
     * Gets or sets device location
     */
    override var location: Location?
        get() = gravityProcessor.location
        set(value) {
            gravityProcessor.location = value
        }

    /**
     * Indicates whether gravity norm must be adjusted to either Earth
     * standard norm, or norm at provided location. If no location is provided, this should only be
     * enabled when device is close to sea level.
     */
    override var adjustGravityNorm: Boolean
        get() = gravityProcessor.adjustGravityNorm
        set(value) {
            gravityProcessor.adjustGravityNorm = value
        }

    /**
     * Processes provided synced measurement to estimate current attitude and position.
     *
     * @param syncedMeasurement synced measurement to be processed.
     * @return true if new pose is estimated, false otherwise.
     */
    fun process(
        syncedMeasurement: AttitudeAndAccelerometerSyncedSensorMeasurement
    ): Boolean {
        val attitudeMeasurement = syncedMeasurement.attitudeMeasurement
        val accelerometerMeasurement = syncedMeasurement.accelerometerMeasurement
        val timestamp = syncedMeasurement.timestamp
        return if (attitudeMeasurement != null && accelerometerMeasurement != null) {
            process(attitudeMeasurement, accelerometerMeasurement, timestamp)
        } else {
            false
        }
    }

    /**
     * Processes provided measurements to estimate current attitude and position.
     *
     * @param attitudeMeasurement attitude measurement.
     * @param accelerometerMeasurement accelerometer measurement.
     * @param timestamp timestamp when all measurements are assumed to occur.
     * @return true if new pose is estimated, false otherwise.
     */
    private fun process(
        attitudeMeasurement: AttitudeSensorMeasurement,
        accelerometerMeasurement: AccelerometerSensorMeasurement,
        timestamp: Long
    ): Boolean {
        attitudeProcessor.process(attitudeMeasurement).copyTo(currentAttitude)

        if (!gravityProcessor.process(accelerometerMeasurement, timestamp)) {
            return false
        }

        gravityProcessor.getGravity(gravity)

        return processPose(accelerometerMeasurement, timestamp)
    }
}