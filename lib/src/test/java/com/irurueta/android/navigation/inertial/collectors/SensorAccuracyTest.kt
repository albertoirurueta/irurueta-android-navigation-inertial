/*
 * Copyright (C) 2021 Alberto Irurueta Carro (alberto@irurueta.com)
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

import android.hardware.SensorManager
import io.mockk.clearAllMocks
import org.junit.After
import org.junit.Assert.assertEquals
import org.junit.Test

class SensorAccuracyTest {

    @After
    fun tearDown() {
        clearAllMocks()
    }

    @Test
    fun sensorAccuracy_fromValues_returnsExpectedValues() {
        assertEquals(4, SensorAccuracy.values().size)
        assertEquals(
            SensorAccuracy.UNRELIABLE,
            SensorAccuracy.from(SensorManager.SENSOR_STATUS_UNRELIABLE)
        )
        assertEquals(
            SensorAccuracy.LOW,
            SensorAccuracy.from(SensorManager.SENSOR_STATUS_ACCURACY_LOW)
        )
        assertEquals(
            SensorAccuracy.MEDIUM,
            SensorAccuracy.from(SensorManager.SENSOR_STATUS_ACCURACY_MEDIUM)
        )
        assertEquals(
            SensorAccuracy.HIGH,
            SensorAccuracy.from(SensorManager.SENSOR_STATUS_ACCURACY_HIGH)
        )
    }
}