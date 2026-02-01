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

package com.irurueta.android.navigation.inertial.collectors.interpolators

import com.irurueta.android.navigation.inertial.collectors.measurements.AttitudeSensorMeasurement
import com.irurueta.geometry.Quaternion
import kotlin.math.max
import kotlin.math.min

/**
 * Interpolates attitude measurements.
 */
class AttitudeSensorMeasurementInterpolator() :
    SensorMeasurementInterpolator<AttitudeSensorMeasurement>() {

    /**
     * Interpolates between two attitude measurements.
     *
     * @param measurement1 the first attitude measurement.
     * @param measurement2 the second attitude measurement.
     * @param alpha the interpolation factor (as a value between 0.0f and 1.0f).
     * @param targetNanoSeconds the target timestamp of resulting measurement expressed in
     * nanoseconds (using the same clock as the other measurements).
     * @param result the resulting attitude measurement.
     */
    override fun interpolate(
        measurement1: AttitudeSensorMeasurement,
        measurement2: AttitudeSensorMeasurement,
        alpha: Float,
        targetNanoSeconds: Long,
        result: AttitudeSensorMeasurement
    ) {
        Quaternion.slerp(
            measurement1.attitude,
            measurement2.attitude,
            min(max(0.0, alpha.toDouble()), 1.0),
            result.attitude
        )

        val headingAccuracy1 = measurement1.headingAccuracy
        val headingAccuracy2 = measurement2.headingAccuracy
        val headingAccuracy = if (headingAccuracy1 != null && headingAccuracy2 != null) {
            interpolate(headingAccuracy1, headingAccuracy2, alpha)
        } else {
            null
        }

        result.headingAccuracy = headingAccuracy
        result.timestamp = targetNanoSeconds
        result.accuracy = measurement1.accuracy
        result.sensorType = measurement1.sensorType
        result.sensorCoordinateSystem = measurement1.sensorCoordinateSystem
    }
}