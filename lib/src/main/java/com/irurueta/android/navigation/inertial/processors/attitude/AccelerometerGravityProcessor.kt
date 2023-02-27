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
package com.irurueta.android.navigation.inertial.processors.attitude

import com.irurueta.android.navigation.inertial.ENUtoNEDConverter
import com.irurueta.android.navigation.inertial.collectors.AccelerometerSensorMeasurement
import com.irurueta.android.navigation.inertial.estimators.filter.AveragingFilter
import com.irurueta.android.navigation.inertial.estimators.filter.LowPassAveragingFilter

/**
 * Collects accelerometer measurements based on android coordinates system (ENU) and uses a low-pass
 * filter to assume it is the gravity component of specific force and converts it to NED coordinates
 * system.
 *
 * @property averagingFilter an averaging filter for accelerometer samples to obtain
 * sensed gravity component of specific force.
 * @property processorListener listener to notify new gravity measurements.
 */
class AccelerometerGravityProcessor(
    val averagingFilter: AveragingFilter = LowPassAveragingFilter(),
    processorListener: OnProcessedListener<AccelerometerSensorMeasurement>? = null
) : BaseGravityProcessor<AccelerometerSensorMeasurement>(processorListener) {

    /**
     * Array to be reused containing result of averaging filter for accelerometer measurements.
     */
    private val accelerometerAveragingFilterOutput = DoubleArray(AveragingFilter.OUTPUT_LENGTH)

    /**
     * Processes an accelerometer sensor measurement collected by a collector or a syncer.
     * @param measurement measurement expressed in ENU android coordinates system to be processed.
     * @param timestamp optional timestamp that can be provided to override timestamp associated to
     * accelerometer measurement. If not set, the timestamp from measurement is used.
     * @return true if a new gravity is estimated, false otherwise.
     *
     * @see com.irurueta.android.navigation.inertial.collectors.AccelerometerAndGyroscopeSensorMeasurementSyncer
     * @see com.irurueta.android.navigation.inertial.collectors.AccelerometerAndMagnetometerSensorMeasurementSyncer
     * @see com.irurueta.android.navigation.inertial.collectors.AccelerometerGravityAndGyroscopeSensorMeasurementSyncer
     * @see com.irurueta.android.navigation.inertial.collectors.AccelerometerGravityAndMagnetometerSensorMeasurementSyncer
     * @see com.irurueta.android.navigation.inertial.collectors.AccelerometerGravityGyroscopeAndMagnetometerSensorMeasurementSyncer
     * @see com.irurueta.android.navigation.inertial.collectors.AccelerometerGyroscopeAndMagnetometerSensorMeasurementSyncer
     * @see com.irurueta.android.navigation.inertial.collectors.BufferedAccelerometerSensorCollector
     * @see com.irurueta.android.navigation.inertial.collectors.AccelerometerSensorCollector
     */
    override fun process(measurement: AccelerometerSensorMeasurement, timestamp: Long): Boolean {
        val ax = measurement.ax.toDouble()
        val ay = measurement.ay.toDouble()
        val az = measurement.az.toDouble()
        val bx = measurement.bx?.toDouble()
        val by = measurement.by?.toDouble()
        val bz = measurement.bz?.toDouble()

        val currentAx = if (bx != null) ax - bx else ax
        val currentAy = if (by != null) ay - by else ay
        val currentAz = if (bz != null) az - bz else az

        return if (averagingFilter.filter(
                currentAx,
                currentAy,
                currentAz,
                accelerometerAveragingFilterOutput,
                timestamp
            )
        ) {
            ENUtoNEDConverter.convert(
                accelerometerAveragingFilterOutput[0],
                accelerometerAveragingFilterOutput[1],
                accelerometerAveragingFilterOutput[2],
                triad
            )

            gx = triad.valueX
            gy = triad.valueY
            gz = triad.valueZ
            this.timestamp = timestamp
            accuracy = measurement.accuracy
            processorListener?.onProcessed(this, gx, gy, gz, this.timestamp, accuracy)
            true
        } else {
            false
        }
    }
}