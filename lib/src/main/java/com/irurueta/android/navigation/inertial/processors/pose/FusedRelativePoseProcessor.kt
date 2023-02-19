/*
 * Copyright (C) 2023 Alberto Irurueta Carro (alberto@irurueta.com)
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

import com.irurueta.android.navigation.inertial.collectors.*
import com.irurueta.android.navigation.inertial.estimators.pose.SpeedTriad
import com.irurueta.android.navigation.inertial.processors.attitude.LeveledRelativeAttitudeProcessor

/**
 * Estimates relative pose from an unknown location.
 * This class estimates device attitude by fusing gravity and gyroscope measurements.
 * Accelerometer and gyroscope are then taken into account to update device position.
 *
 * @property initialSpeed initial device speed in body coordinates.
 * @property processorListener listener to notify new poses.
 */
class FusedRelativePoseProcessor(
    initialSpeed: SpeedTriad = SpeedTriad(),
    processorListener: OnProcessedListener? = null
) : BaseFusedRelativePoseProcessor<GravitySensorMeasurement,
        AccelerometerGravityGyroscopeAndMagnetometerSyncedSensorMeasurement>(
    initialSpeed,
    processorListener
) {
    /**
     * Attitude processor in charge of fusing gravity + gyroscope
     * measurements to estimate current relative device attitude.
     */
    override val attitudeProcessor = LeveledRelativeAttitudeProcessor()

    /**
     * Processes provided synced measurement to estimate current attitude and position.
     *
     * @param syncedMeasurement synced measurement to be processed.
     * @return true if new pose is estimated, false otherwise.
     */
    fun process(syncedMeasurement: AccelerometerGravityAndGyroscopeSyncedSensorMeasurement): Boolean {
        val accelerometerMeasurement = syncedMeasurement.accelerometerMeasurement
        val gravityMeasurement = syncedMeasurement.gravityMeasurement
        val gyroscopeMeasurement = syncedMeasurement.gyroscopeMeasurement
        val timestamp = syncedMeasurement.timestamp
        return if (accelerometerMeasurement != null && gravityMeasurement != null
            && gyroscopeMeasurement != null
        ) {
            process(
                accelerometerMeasurement,
                gravityMeasurement,
                gyroscopeMeasurement,
                timestamp
            )
        } else {
            false
        }
    }

    /**
     * Processes provided measurements to estimate current attitude and position.
     *
     * @param accelerometerMeasurement accelerometer measurement.
     * @param gravityMeasurement gravity measurement.
     * @param gyroscopeMeasurement gyroscope measurement.
     * @param timestamp timestamp when all measurements are assumed to occur.
     * @return true if new pose is estimated, false otherwise.
     */
    private fun process(
        accelerometerMeasurement: AccelerometerSensorMeasurement,
        gravityMeasurement: GravitySensorMeasurement,
        gyroscopeMeasurement: GyroscopeSensorMeasurement,
        timestamp: Long
    ): Boolean {
        return if (processAttitude(
                gravityMeasurement,
                gyroscopeMeasurement,
                timestamp
            )
        ) {
            processPose(accelerometerMeasurement, gyroscopeMeasurement, timestamp)
        } else {
            false
        }
    }
}