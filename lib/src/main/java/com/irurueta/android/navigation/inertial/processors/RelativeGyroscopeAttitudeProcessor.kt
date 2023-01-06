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

import com.irurueta.android.navigation.inertial.collectors.GyroscopeSensorMeasurement
import com.irurueta.geometry.Quaternion

/**
 * Estimates relative attitude of device respect to an arbitrary initial attitude using gyroscope
 * measurements only.
 *
 * @property processorListener listener to notify new relative attitudes.
 */
class RelativeGyroscopeAttitudeProcessor(processorListener: OnProcessedListener? = null) :
    BaseRelativeGyroscopeAttitudeProcessor(processorListener) {

    /**
     * Instance to be reused which contains variation of attitude between gyroscope samples.
     */
    private val deltaAttitude = Quaternion()

    /**
     * Processes a gyroscope sensor measurement to integrate angular speed values to obtain
     * an accumulated attitude from an arbitrary attitude of origin.
     *
     * @param measurement gyroscope measurement expressed in ENU android coordinates system to be
     * processed
     * @param timestamp optional timestamp that can be provided to override timestamp associated to
     * gyroscope measurement. If null, the timestamp from gyroscope measurement is used.
     * @return true if a new relative attitude is estimated, false otherwise.
     */
    override fun process(measurement: GyroscopeSensorMeasurement, timestamp: Long): Boolean {
        val isFirst = updateTimeInterval(timestamp)

        return if (!isFirst) {
            updateTriad(measurement)

            val dt = timeIntervalEstimator.averageTimeInterval

            val roll = triad.valueX * dt
            val pitch = triad.valueY * dt
            val yaw = triad.valueZ * dt
            deltaAttitude.setFromEulerAngles(roll, pitch, yaw)

            internalAttitude.combine(deltaAttitude)
            internalAttitude.normalize()

            internalAttitude.copyTo(attitude)

            processorListener?.onProcessed(this, attitude, measurement.accuracy)
            true
        } else {
            false
        }
    }
}