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

import com.irurueta.android.navigation.inertial.collectors.measurements.AccelerometerAndGyroscopeSyncedSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.AccelerometerSensorMeasurement

/**
 * Estimates leveled relative attitude by fusing leveling attitude obtained
 * from accelerometer sensor, and relative attitude obtained from gyroscope sensor.
 *
 * @property processorListener listener to notify new leveled relative attitudes.
 */
class AccelerometerLeveledRelativeAttitudeProcessor(
    processorListener: OnProcessedListener<AccelerometerSensorMeasurement, AccelerometerAndGyroscopeSyncedSensorMeasurement>? = null
) : BaseLeveledRelativeAttitudeProcessor<AccelerometerSensorMeasurement, AccelerometerAndGyroscopeSyncedSensorMeasurement>(
    processorListener
) {
    /**
     * Internal processor to estimate gravity from accelerometer sensor measurements.
     */
    override val gravityProcessor = AccelerometerGravityProcessor()

    /**
     * Processes provided synced measurement to estimate current leveled relative attitude.
     *
     * @param syncedMeasurement synced measurement to be processed.
     * @return true if a new leveled relative attitude has been estimated, false otherwise.
     */
    override fun process(syncedMeasurement: AccelerometerAndGyroscopeSyncedSensorMeasurement): Boolean {
        val accelerometerMeasurement = syncedMeasurement.accelerometerMeasurement
        val gyroscopeMeasurement = syncedMeasurement.gyroscopeMeasurement
        return if (accelerometerMeasurement != null && gyroscopeMeasurement != null) {
            process(accelerometerMeasurement, gyroscopeMeasurement, syncedMeasurement.timestamp)
        } else {
            false
        }
    }
}