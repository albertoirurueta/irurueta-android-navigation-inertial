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
package com.irurueta.android.navigation.inertial.old.processors.attitude

import com.irurueta.android.navigation.inertial.old.collectors.GravityAndMagnetometerSyncedSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.GravitySensorMeasurement

/**
 * Estimates leveled absolute attitude using gravity and magnetometer sensors.
 * Roll and pitch Euler angles are leveled using gravity sensor.
 * Yaw angle is obtained from magnetometer once the leveling is estimated.
 */
class GeomagneticAttitudeProcessor(
    processorListener: OnProcessedListener<GravitySensorMeasurement,
            GravityAndMagnetometerSyncedSensorMeasurement>? = null
) : BaseGeomagneticAttitudeProcessor<GravitySensorMeasurement,
        GravityAndMagnetometerSyncedSensorMeasurement>(processorListener) {

    /**
     * Internal processor to estimate gravity from gravity sensor measurements.
     */
    override val gravityProcessor = GravityProcessor()

    /**
     * Processes provided synced measurement to estimated fused leveled absolute attitude.
     *
     * @param syncedMeasurement synced measurement to be processed.
     * @return true if a new leveled absolute attitude is processed, false otherwise.
     */
    override fun process(
        syncedMeasurement: GravityAndMagnetometerSyncedSensorMeasurement): Boolean {
        val gravityMeasurement = syncedMeasurement.gravityMeasurement
        val magnetometerMeasurement = syncedMeasurement.magnetometerMeasurement
        return if (gravityMeasurement != null && magnetometerMeasurement != null) {
            process(gravityMeasurement, magnetometerMeasurement)
        } else {
            false
        }
    }
}