/*
 * Copyright (C) 2022 Alberto Irurueta Carro (alberto@irurueta.com)
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
package com.irurueta.android.navigation.inertial.collectors

import android.hardware.Sensor
import org.junit.Assert.*
import org.junit.Test
import org.junit.runner.RunWith
import org.robolectric.RobolectricTestRunner

@RunWith(RobolectricTestRunner::class)
class GyroscopeSensorTypeTest {

    @Test
    fun fromValues_returnsExpected() {
        assertEquals(2, GyroscopeSensorType.entries.size)
        assertEquals(
            GyroscopeSensorType.GYROSCOPE,
            GyroscopeSensorType.from(Sensor.TYPE_GYROSCOPE)
        )
        assertEquals(
            GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
            GyroscopeSensorType.from(Sensor.TYPE_GYROSCOPE_UNCALIBRATED)
        )
    }
}