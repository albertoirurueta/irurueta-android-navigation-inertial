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
package com.irurueta.android.navigation.inertial.test

import android.util.Log
import androidx.test.platform.app.InstrumentationRegistry
import com.irurueta.android.navigation.inertial.BatteryTemperatureService
import com.irurueta.android.navigation.inertial.ThreadSyncHelper
import com.irurueta.units.Temperature
import com.irurueta.units.TemperatureUnit
import org.junit.Assert.*
import org.junit.Before
import org.junit.Test

class BatteryTemperatureServiceTest {

    private val syncHelper = ThreadSyncHelper()

    @Volatile
    private var completed = 0

    @Before
    fun setUp() {
        completed = 0
    }

    @Test
    fun currentBatteryTemperatureCelsius_returnsValue() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val service = BatteryTemperatureService(context)
        val temperature = service.currentBatteryTemperatureCelsius

        assertNotNull(temperature)
    }

    @Test
    fun currentBatteryTemperature_returnsValue() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val service = BatteryTemperatureService(context)
        val temperature = service.currentBatteryTemperature

        assertNotNull(temperature)
    }

    @Test
    fun getCurrentBatteryTemperature_returnsValue() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val service = BatteryTemperatureService(context)
        val temperature = Temperature(0.0, TemperatureUnit.FAHRENHEIT)

        assertTrue(service.getCurrentBatteryTemperature(temperature))
        assertNotEquals(0.0, temperature.value.toDouble())
        assertEquals(TemperatureUnit.CELSIUS, temperature.unit)
    }

    @Test
    fun startAndStop_returnsValue() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val service =
            BatteryTemperatureService(context, batteryTemperatureChangedListener = { temperature ->
                assertTrue(temperature > 0.0f)
                Log.d("BatteryTemperatureServiceTest", "temperature: $temperature")
                syncHelper.notifyAll { completed++ }
            })

        service.start()

        syncHelper.waitOnCondition({ completed < 1 })
        assertEquals(1, completed)
        service.stop()
    }
}