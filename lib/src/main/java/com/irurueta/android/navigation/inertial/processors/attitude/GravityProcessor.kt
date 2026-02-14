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

import android.location.Location
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
) : BaseGravityProcessor<GravitySensorMeasurement>(
    location,
    adjustGravityNorm,
    processorListener
) {
    /**
     * Gravity measurement being reused and containing measurements converted into NED coordinates
     * system.
     */
    private val nedMeasurement = GravitySensorMeasurement()

    /**
     * Processes a gravity sensor measurement collected by a collector or a syncer.
     * Notice that this processor will convert provided measurements to NED coordinates system if
     * needed, and results will always be returned in NED coordinates system.
     *
     * @param measurement measurement expressed in ENU or NED coordinates system to be processed.
     * @param timestamp optional timestamp that can be provided to override timestamp associated to
     * gravity measurement. If not set, the timestamp from measurement is used.
     * @return true if a new gravity is estimated, false otherwise.
     */
    override fun process(
        measurement: GravitySensorMeasurement,
        timestamp: Long
    ): Boolean {
        measurement.toNed(nedMeasurement)
        nedMeasurement.toTriad(triad)
        finishProcessingAndNotify(measurement, timestamp)
        return true
    }
}