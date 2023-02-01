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
package com.irurueta.android.navigation.inertial.processors

import com.irurueta.android.navigation.inertial.collectors.GravityGyroscopeAndMagnetometerSyncedSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.GravitySensorMeasurement

/**
 * Estimates absolute attitude by fusing absolute leveled geomagnetic attitude and leveled relative
 * attitude.
 *
 * @property processorListener listener to notify new fused absolute attitude.
 */
class DoubleFusedGeomagneticAttitudeProcessor(processorListener: OnProcessedListener<GravitySensorMeasurement, GravityGyroscopeAndMagnetometerSyncedSensorMeasurement>? = null
) : BaseDoubleFusedGeomagneticAttitudeProcessor<GravitySensorMeasurement, GravityGyroscopeAndMagnetometerSyncedSensorMeasurement>(processorListener) {

    /**
     * Internal processor to estimate leveled absolute attitude.
     */
    override val geomagneticProcessor = GeomagneticAttitudeProcessor()

    /**
     * Internal processor to estimate leveled relative attitude.
     */
    override val relativeGyroscopeProcessor = LeveledRelativeAttitudeProcessor()

    /**
     * Processes provided synced measurement to estimate current fused absolute attitude.
     *
     * @param syncedMeasurement synced measurement to be processed.
     * @return true if a new fused absolute attitude has been estimated, false otherwise.
     */
    override fun process(syncedMeasurement: GravityGyroscopeAndMagnetometerSyncedSensorMeasurement): Boolean {
        val gravityMeasurement = syncedMeasurement.gravityMeasurement
        val gyroscopeMeasurement = syncedMeasurement.gyroscopeMeasurement
        val magnetometerMeasurement = syncedMeasurement.magnetometerMeasurement
        return if (gravityMeasurement != null && gyroscopeMeasurement != null && magnetometerMeasurement != null) {
            process(gravityMeasurement, gyroscopeMeasurement, magnetometerMeasurement)
        } else {
            false
        }
    }
}