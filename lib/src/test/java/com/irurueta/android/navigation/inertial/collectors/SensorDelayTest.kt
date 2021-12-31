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
import org.junit.Assert.assertEquals
import org.junit.Test
import org.junit.runner.RunWith
import org.robolectric.RobolectricTestRunner

@RunWith(RobolectricTestRunner::class)
class SensorDelayTest {

    @Test
    fun sensorDelay_fromValues_returnsExpectedValues() {
        assertEquals(4, SensorDelay.values().size)
        assertEquals(SensorDelay.FASTEST, SensorDelay.from(SensorManager.SENSOR_DELAY_FASTEST))
        assertEquals(SensorDelay.GAME, SensorDelay.from(SensorManager.SENSOR_DELAY_GAME))
        assertEquals(SensorDelay.UI, SensorDelay.from(SensorManager.SENSOR_DELAY_UI))
        assertEquals(SensorDelay.NORMAL, SensorDelay.from(SensorManager.SENSOR_DELAY_NORMAL))
    }
}