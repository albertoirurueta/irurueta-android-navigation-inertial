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
package com.irurueta.android.navigation.inertial.old.collectors.interpolators

import com.irurueta.android.navigation.inertial.collectors.measurements.AttitudeSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.SensorAccuracy
import com.irurueta.geometry.Quaternion
import com.irurueta.statistics.UniformRandomizer
import org.junit.Assert.*
import org.junit.Test

class AttitudeDirectSensorMeasurementInterpolatorTest {

    @Test
    fun constructor_createsInstance() {
        assertNotNull(AttitudeDirectSensorMeasurementInterpolator())
    }

    @Test
    fun push_makesNoAction() {
        val interpolator = AttitudeDirectSensorMeasurementInterpolator()

        val measurement = generateMeasurement()
        interpolator.push(measurement)
    }

    @Test
    fun interpolate_computesExpectedValue() {
        val interpolator = AttitudeDirectSensorMeasurementInterpolator()

        val currentMeasurement = generateMeasurement()
        val result = AttitudeSensorMeasurement()
        val timestamp = System.nanoTime()

        assertTrue(interpolator.interpolate(currentMeasurement, timestamp, result))

        assertNotSame(currentMeasurement, result)
        assertEquals(currentMeasurement.attitude, result.attitude)
        assertEquals(currentMeasurement.headingAccuracy, currentMeasurement.headingAccuracy)
        assertEquals(currentMeasurement.timestamp, result.timestamp)
        assertEquals(currentMeasurement.accuracy, result.accuracy)
    }

    @Test
    fun reset_makesNoAction() {
        val interpolator = AttitudeDirectSensorMeasurementInterpolator()

        interpolator.reset()
    }

    private companion object {

        const val MIN_DEGREES = -90.0
        const val MAX_DEGREES = 90.0

        fun generateMeasurement(): AttitudeSensorMeasurement {
            val randomizer = UniformRandomizer()
            val roll = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
            val pitch = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
            val yaw = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
            val attitude = Quaternion(roll, pitch, yaw)

            val headingAccuracy = randomizer.nextFloat()
            val timestamp = System.nanoTime()
            return AttitudeSensorMeasurement(
                attitude,
                headingAccuracy,
                timestamp,
                SensorAccuracy.MEDIUM
            )
        }
    }
}