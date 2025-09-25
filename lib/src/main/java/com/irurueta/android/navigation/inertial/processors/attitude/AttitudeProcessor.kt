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
import com.irurueta.android.navigation.inertial.collectors.AttitudeSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.SensorAccuracy
import com.irurueta.geometry.Quaternion

/**
 * Processes an absolute or relative attitude obtained by an Android Sensor expressed in ENU
 * coordinates system and converts it to NED coordinates system.
 *
 * @property processorListener listener to notify new attitudes.
 */
class AttitudeProcessor(var processorListener: OnProcessedListener? = null) {

    /**
     * Converted absolute or relative attitude in NED coordinates.
     */
    var nedAttitude = Quaternion()
        private set

    /**
     * Processes an attitude measurement containing an absolute or relative attitude obtained by one
     * of the available android sensors expressed in ENU coordinates system.
     *
     * @param measurement measurement to be converted.
     * @return converted attitude in NED coordinates.
     */
    fun process(measurement: AttitudeSensorMeasurement): Quaternion {
        val enuAttitude = measurement.attitude

        ENUtoNEDConverter.convert(enuAttitude, nedAttitude)

        processorListener?.onProcessed(
            this,
            nedAttitude,
            measurement.timestamp,
            measurement.accuracy
        )

        return nedAttitude
    }

    /**
     * Interface to notify when a new attitude measurement has been processed.
     */
    fun interface OnProcessedListener {
        /**
         * Called when a new attitude measurement is processed.
         *
         * @param processor processor that raised this event.
         * @param attitude containing absolute or relative device attitude expressed in NED
         * coordinates.
         * @param timestamp time in nanoseconds at which the measurement was made. Each measurement
         * will be monotonically increasing using the same time base as
         * [android.os.SystemClock.elapsedRealtimeNanos].
         * @param accuracy attitude sensor accuracy.
         */
        fun onProcessed(
            processor: AttitudeProcessor,
            attitude: Quaternion,
            timestamp: Long,
            accuracy: SensorAccuracy?
        )
    }
}