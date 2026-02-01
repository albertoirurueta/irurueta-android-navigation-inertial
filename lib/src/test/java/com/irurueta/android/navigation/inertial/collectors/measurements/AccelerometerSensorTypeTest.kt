/*
 * Copyright (C) 2025 Alberto Irurueta Carro (alberto@irurueta.com)
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

package com.irurueta.android.navigation.inertial.collectors.measurements

import android.hardware.Sensor
import org.junit.Assert.assertEquals
import org.junit.Test

class AccelerometerSensorTypeTest {

    @Test
    fun fromValues_returnsExpected() {
        assertEquals(2, AccelerometerSensorType.entries.size)
        assertEquals(
            AccelerometerSensorType.ACCELEROMETER,
            AccelerometerSensorType.from(Sensor.TYPE_ACCELEROMETER)
        )
        assertEquals(
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            AccelerometerSensorType.from(TYPE_ACCELEROMETER_UNCALIBRATED)
        )
    }

    companion object {
        const val TYPE_ACCELEROMETER_UNCALIBRATED = 35
    }
}