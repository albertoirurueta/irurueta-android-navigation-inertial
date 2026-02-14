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

import android.location.Location
import com.irurueta.android.navigation.inertial.ENUtoNEDConverter
import com.irurueta.android.navigation.inertial.collectors.measurements.GravitySensorMeasurement

/**
 * Collects gravity measurements based on android coordinates system (ENU), and converts them to
 * NED coordinates system.
 *
 * @property location current device location.
 * @property adjustGravityNorm indicates whether gravity norm must be adjusted to either Earth
 * standard norm, or norm at provided location. If no location is provided, this should only be
 * enabled when device is close to sea level.
 * @property processorListener listener to notify new gravity measurements.
 */
class GravityProcessor(
    location: Location? = null,
    adjustGravityNorm: Boolean = true,
    processorListener: OnProcessedListener<GravitySensorMeasurement>? = null
) : BaseGravityProcessor<GravitySensorMeasurement>(location, adjustGravityNorm, processorListener) {

    /**
     * Processes a gravity sensor measurement collected by a collector or a syncer.
     * @param measurement measurement expressed in ENU android coordinates system to be processed.
     * @param timestamp optional timestamp that can be provided to override timestamp associated to
     * gravity measurement. If not set, the timestamp from measurement is used.
     * @return true if a new gravity is estimated, false otherwise.
     *
     * @see com.irurueta.android.navigation.inertial.old.collectors.AccelerometerGravityAndGyroscopeSensorMeasurementSyncer
     * @see com.irurueta.android.navigation.inertial.old.collectors.AccelerometerGravityAndMagnetometerSensorMeasurementSyncer
     * @see com.irurueta.android.navigation.inertial.old.collectors.AccelerometerGravityGyroscopeAndMagnetometerSensorMeasurementSyncer
     * @see com.irurueta.android.navigation.inertial.old.collectors.BufferedGravitySensorCollector
     * @see com.irurueta.android.navigation.inertial.old.collectors.GravitySensorCollector
     */
    override fun process(measurement: GravitySensorMeasurement, timestamp: Long): Boolean {
        ENUtoNEDConverter.convert(
            measurement.gx.toDouble(),
            measurement.gy.toDouble(),
            measurement.gz.toDouble(),
            triad
        )
        gx = triad.valueX
        gy = triad.valueY
        gz = triad.valueZ

        adjustNorm()

        this.timestamp = timestamp
        accuracy = measurement.accuracy
        processorListener?.onProcessed(this, gx, gy, gz, this.timestamp, accuracy)
        return true
    }
}