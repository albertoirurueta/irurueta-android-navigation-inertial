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

import android.location.Location
import com.irurueta.android.navigation.inertial.old.collectors.AccelerometerGyroscopeAndMagnetometerSyncedSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.AccelerometerSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.GyroscopeSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.MagnetometerSensorMeasurement
import com.irurueta.android.navigation.inertial.processors.attitude.AccelerometerDoubleFusedGeomagneticAttitudeProcessor
import com.irurueta.navigation.frames.NEDVelocity

/**
 * Estimates absolute pose expressed in ECEF coordinates.
 * This class estimates device attitude by double fusing accelerometer, gyroscope and magnetometer
 * measurements.
 * Accelerometer and gyroscope are then taken into account to update device position.
 *
 * @property initialLocation initial device location.
 * @property initialVelocity initial velocity of device expressed in NED coordinates.
 * @property estimatePoseTransformation true to estimate 3D metric pose transformation.
 * @property processorListener listener to notify new poses.
 */
class AccelerometerDoubleFusedECEFAbsolutePoseProcessor(
    initialLocation: Location,
    initialVelocity: NEDVelocity = NEDVelocity(),
    estimatePoseTransformation: Boolean = false,
    processorListener: OnProcessedListener? = null
) : BaseDoubleFusedECEFAbsolutePoseProcessor<AccelerometerSensorMeasurement, AccelerometerGyroscopeAndMagnetometerSyncedSensorMeasurement>(
    initialLocation,
    initialVelocity,
    estimatePoseTransformation,
    processorListener
) {
    /**
     * Attitude processor in charge of fusing accelerometer + gyroscope and magnetometer
     * measurements to estimate current device attitude.
     */
    override val attitudeProcessor = AccelerometerDoubleFusedGeomagneticAttitudeProcessor()

    /**
     * Processes provided synced measurement to estimate current attitude and position.
     *
     * @param syncedMeasurement synced measurement to be processed.
     * @return true if new pose is estimated, false otherwise.
     */
    fun process(syncedMeasurement: AccelerometerGyroscopeAndMagnetometerSyncedSensorMeasurement): Boolean {
        val accelerometerMeasurement = syncedMeasurement.accelerometerMeasurement
        val gyroscopeMeasurement = syncedMeasurement.gyroscopeMeasurement
        val magnetometerMeasurement = syncedMeasurement.magnetometerMeasurement
        val timestamp = syncedMeasurement.timestamp
        return if (accelerometerMeasurement != null && gyroscopeMeasurement != null
            && magnetometerMeasurement != null
        ) {
            process(
                accelerometerMeasurement,
                gyroscopeMeasurement,
                magnetometerMeasurement,
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
     * @param gyroscopeMeasurement gyroscope measurement.
     * @param magnetometerMeasurement magnetometer measurement.
     * @param timestamp timestamp when all measurements are assumed to occur.
     * @return true if new pose is estimated, false otherwise.
     */
    private fun process(
        accelerometerMeasurement: AccelerometerSensorMeasurement,
        gyroscopeMeasurement: GyroscopeSensorMeasurement,
        magnetometerMeasurement: MagnetometerSensorMeasurement,
        timestamp: Long
    ): Boolean {
        return if (processAttitude(
                accelerometerMeasurement,
                gyroscopeMeasurement,
                magnetometerMeasurement,
                timestamp
            )
        ) {
            processPose(accelerometerMeasurement, gyroscopeMeasurement, timestamp)
        } else {
            false
        }
    }

    init {
        // initialize attitude processor location
        attitudeProcessor.location = initialLocation
    }
}