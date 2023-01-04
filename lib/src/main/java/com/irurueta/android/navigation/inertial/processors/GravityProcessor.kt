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

import com.irurueta.android.navigation.inertial.ENUtoNEDTriadConverter
import com.irurueta.android.navigation.inertial.collectors.GravitySensorMeasurement

/**
 * Collects gravity measurements based on android coordinates system (ENU), and converts them to
 * NED coordinates system.
 *
 * @property processorListener listener to notify new gravity measurements.
 */
class GravityProcessor(
    processorListener: OnProcessedListener<GravitySensorMeasurement>? = null
) : BaseGravityProcessor<GravitySensorMeasurement>(processorListener) {

    /**
     * Processes a gravity sensor measurement collected by a collector or a syncer.
     * @param measurement measurement expressed in ENU android coordinates system to be processed.
     * @return true if a new gravity is estimated, false otherwise.
     *
     * @see com.irurueta.android.navigation.inertial.collectors.AccelerometerGravityAndGyroscopeSensorMeasurementSyncer
     * @see com.irurueta.android.navigation.inertial.collectors.AccelerometerGravityAndMagnetometerSensorMeasurementSyncer
     * @see com.irurueta.android.navigation.inertial.collectors.AccelerometerGravityGyroscopeAndMagnetometerSensorMeasurementSyncer
     * @see com.irurueta.android.navigation.inertial.collectors.BufferedGravitySensorCollector
     * @see com.irurueta.android.navigation.inertial.collectors.GravitySensorCollector
     */
    override fun process(measurement: GravitySensorMeasurement): Boolean {
        ENUtoNEDTriadConverter.convert(
            measurement.gx.toDouble(),
            measurement.gy.toDouble(),
            measurement.gz.toDouble(),
            triad
        )
        gx = triad.valueX
        gy = triad.valueY
        gz = triad.valueZ
        timestamp = measurement.timestamp
        accuracy = measurement.accuracy
        processorListener?.onProcessed(this, gx, gy, gz, timestamp, accuracy)
        return true
    }
}