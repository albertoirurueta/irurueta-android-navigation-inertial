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

import com.irurueta.android.navigation.inertial.collectors.measurements.AccelerometerAndGyroscopeSyncedSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.AccelerometerSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.GyroscopeSensorMeasurement
import com.irurueta.android.navigation.inertial.processors.attitude.AccelerometerLeveledRelativeAttitudeProcessor
import com.irurueta.android.navigation.inertial.processors.pose.zupt.ZuptProcessorBuilder
import com.irurueta.android.navigation.inertial.processors.pose.zupt.ZuptSettings
import com.irurueta.navigation.inertial.calibration.SpeedTriad

/**
 * Estimates relative pose from an unknown location.
 * This class estimates device attitude by fusing accelerometer and gyroscope measurements and.
 * then they are taken into account to update device position.
 *
 * @property initialSpeed initial device speed in body coordinates.
 * @property processorListener listener to notify new poses.
 * @property zuptSettings settings for ZUPT (Zero Velocity Update) evaluation.
 */
class AccelerometerFusedRelativePoseProcessor(
    initialSpeed: SpeedTriad = SpeedTriad(),
    processorListener: OnProcessedListener? = null,
    val zuptSettings: ZuptSettings = ZuptSettings()
) : BaseFusedRelativePoseProcessor<AccelerometerSensorMeasurement, AccelerometerAndGyroscopeSyncedSensorMeasurement>(
    initialSpeed,
    processorListener
) {
    /**
     * Attitude processor in charge of fusing accelerometer + gyroscope
     * measurements to estimate current relative device attitude.
     */
    override val attitudeProcessor = AccelerometerLeveledRelativeAttitudeProcessor()

    /**
     * Computes scores to determine whether ZUPT (Zero Velocity Update) must be performed to reset
     * velocity.
     */
    private val zuptProcessor =
        ZuptProcessorBuilder<AccelerometerAndGyroscopeSyncedSensorMeasurement>(zuptSettings).build()

    /**
     * Processes provided synced measurement to estimate current attitude and position.
     *
     * @param syncedMeasurement synced measurement to be processed.
     * @return true if new pose is estimated, false otherwise.
     */
    fun process(syncedMeasurement: AccelerometerAndGyroscopeSyncedSensorMeasurement): Boolean {
        val accelerometerMeasurement = syncedMeasurement.accelerometerMeasurement
        val gyroscopeMeasurement = syncedMeasurement.gyroscopeMeasurement
        val timestamp = syncedMeasurement.timestamp
        return if (accelerometerMeasurement != null && gyroscopeMeasurement != null
        ) {
            process(
                accelerometerMeasurement,
                gyroscopeMeasurement,
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
     * @param gyroscopeMeasurement gyroscope measurement.
     * @param timestamp timestamp when all measurements are assumed to occur.
     * @param zuptScore Value between 0.0 and 1.0 that indicates the likeliness to apply a Zero
     * Velocity Update, where 0.0 indicates no likeliness and 1.0 indicates full likeliness of a
     * ZUPT. Likeliness, is used to weight velocity updates where a score of 1.0 resets velocity to
     * zero and a score of 0.0 keeps expected velocity update.
     * @return true if new pose is estimated, false otherwise.
     */
    private fun process(
        accelerometerMeasurement: AccelerometerSensorMeasurement,
        gyroscopeMeasurement: GyroscopeSensorMeasurement,
        timestamp: Long,
        zuptScore: Double
    ): Boolean {
        return if (processAttitude(
                accelerometerMeasurement,
                gyroscopeMeasurement,
                timestamp
            )
        ) {
            processPose(accelerometerMeasurement, timestamp, zuptScore)
        } else {
            false
        }
    }
}