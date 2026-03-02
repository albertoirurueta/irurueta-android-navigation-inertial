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
import com.irurueta.android.navigation.inertial.collectors.measurements.AccelerometerGravityGyroscopeAndMagnetometerSyncedSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.AccelerometerSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.GravitySensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.GyroscopeSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.MagnetometerSensorMeasurement
import com.irurueta.android.navigation.inertial.processors.attitude.FusedGeomagneticAttitudeProcessor
import com.irurueta.android.navigation.inertial.processors.pose.zupt.ZuptProcessorBuilder
import com.irurueta.android.navigation.inertial.processors.pose.zupt.ZuptSettings
import com.irurueta.navigation.frames.NEDVelocity

/**
 * Estimates absolute pose using local plane navigation.
 * This class estimated device attitude by fusing gravity, gyroscope and magnetometer
 * measurements.
 * Accelerometer and gyroscope are then taken into account to update device position.
 *
 * @property initialLocation initial device location.
 * @property initialVelocity initial velocity of device expressed in NED coordinates.
 * @property estimatePoseTransformation true to estimate 3D metric pose transformation.
 * @property processorListener listener to notify new poses.
 * @property zuptSettings settings for ZUPT (Zero Velocity Update) evaluation.
 */
class FusedLocalPoseProcessor(
    initialLocation: Location,
    initialVelocity: NEDVelocity = NEDVelocity(),
    estimatePoseTransformation: Boolean = false,
    processorListener: OnProcessedListener? = null,
    val zuptSettings: ZuptSettings = ZuptSettings()
) : BaseFusedLocalPoseProcessor<GravitySensorMeasurement, AccelerometerGravityGyroscopeAndMagnetometerSyncedSensorMeasurement>(
    initialLocation,
    initialVelocity,
    estimatePoseTransformation,
    processorListener
) {
    /**
     * Attitude processor in charge of fusing gravity + gyroscope and magnetometer
     * measurements to estimate current device attitude.
     */
    override val attitudeProcessor = FusedGeomagneticAttitudeProcessor()

    /**
     * Computes scores to determine whether ZUPT (Zero Velocity Update) must be performed to reset
     * velocity.
     */
    private val zuptProcessor =
        ZuptProcessorBuilder<AccelerometerGravityGyroscopeAndMagnetometerSyncedSensorMeasurement>(
            zuptSettings
        ).build()

    /**
     * Processes provided synced measurement to estimate current attitude and position.
     *
     * @param syncedMeasurement synced measurement to be processed.
     * @return true if new pose is estimated, false otherwise.
     */
    fun process(syncedMeasurement: AccelerometerGravityGyroscopeAndMagnetometerSyncedSensorMeasurement): Boolean {
        val accelerometerMeasurement = syncedMeasurement.accelerometerMeasurement
        val gravityMeasurement = syncedMeasurement.gravityMeasurement
        val gyroscopeMeasurement = syncedMeasurement.gyroscopeMeasurement
        val magnetometerMeasurement = syncedMeasurement.magnetometerMeasurement
        val timestamp = syncedMeasurement.timestamp
        return if (accelerometerMeasurement != null && gravityMeasurement != null
            && gyroscopeMeasurement != null && magnetometerMeasurement != null
        ) {
            process(
                accelerometerMeasurement,
                gravityMeasurement,
                gyroscopeMeasurement,
                magnetometerMeasurement,
                timestamp,
                zuptProcessor.process(syncedMeasurement)
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
     * @param magnetometerMeasurement magnetometer measurement.
     * @param timestamp timestamp when all measurements are assumed to occur.
     * @param zuptScore Value between 0.0 and 1.0 that indicates the likeliness to apply a Zero
     * Velocity Update, where 0.0 indicates no likeliness and 1.0 indicates full likeliness of a
     * ZUPT. Likeliness, is used to weight velocity updates where a score of 1.0 resets velocity to
     * zero and a score of 0.0 keeps expected velocity update.
     * @return true if new pose is estimated, false otherwise.
     */
    private fun process(
        accelerometerMeasurement: AccelerometerSensorMeasurement,
        gravityMeasurement: GravitySensorMeasurement,
        gyroscopeMeasurement: GyroscopeSensorMeasurement,
        magnetometerMeasurement: MagnetometerSensorMeasurement,
        timestamp: Long,
        zuptScore: Double
    ): Boolean {
        return if (processAttitude(
                gravityMeasurement,
                gyroscopeMeasurement,
                magnetometerMeasurement,
                timestamp
            )
        ) {
            processPose(accelerometerMeasurement, gyroscopeMeasurement, timestamp, zuptScore)
        } else {
            false
        }
    }

    init {
        // initialize attitude processor location
        attitudeProcessor.location = initialLocation
    }
}