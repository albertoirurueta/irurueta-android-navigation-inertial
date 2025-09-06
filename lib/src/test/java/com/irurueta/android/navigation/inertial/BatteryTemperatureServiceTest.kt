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
package com.irurueta.android.navigation.inertial

import android.annotation.SuppressLint
import android.content.BroadcastReceiver
import android.content.Context
import android.content.Intent
import android.content.IntentFilter
import android.os.BatteryManager
import androidx.test.core.app.ApplicationProvider
import com.irurueta.android.testutils.getPrivateProperty
import com.irurueta.android.testutils.setPrivateProperty
import com.irurueta.units.Temperature
import com.irurueta.units.TemperatureUnit
//import io.mockk.*
//import io.mockk.impl.annotations.MockK
//import io.mockk.impl.annotations.SpyK
//import io.mockk.junit4.MockKRule
//import org.junit.After
import org.junit.Assert.*
//import org.junit.Ignore
import org.junit.Rule
import org.junit.Test
import org.junit.runner.RunWith
import org.mockito.ArgumentCaptor
import org.mockito.Captor
import org.mockito.Mock
import org.mockito.Spy
import org.mockito.junit.MockitoJUnit
import org.mockito.junit.MockitoRule
import org.mockito.kotlin.any
import org.mockito.kotlin.capture
import org.mockito.kotlin.doReturn
import org.mockito.kotlin.eq
import org.mockito.kotlin.only
import org.mockito.kotlin.times
import org.mockito.kotlin.verify
import org.mockito.kotlin.verifyNoInteractions
import org.mockito.kotlin.whenever
import org.robolectric.RobolectricTestRunner

//@Ignore("Possible memory leak when running this test")
@RunWith(RobolectricTestRunner::class)
class BatteryTemperatureServiceTest {

    @get:Rule
    val mockitoRule: MockitoRule = MockitoJUnit.rule()

//    @get:Rule
//    val mockkRule = MockKRule(this)

//    @MockK
    @Mock
    private lateinit var listener: BatteryTemperatureService.OnBatteryTemperatureChangedListener

//    @MockK
    @Mock
    private lateinit var intent: Intent

//    @SpyK
    @Spy
    private var contextSpy = ApplicationProvider.getApplicationContext<Context>()

    @Captor
    private lateinit var intentFilterCaptor: ArgumentCaptor<IntentFilter>

    /*@After
    fun tearDown() {
        unmockkAll()
        clearAllMocks()
        System.gc()
    }*/

    @Test
    fun constructor_whenRequiredParameters_setsDefaultValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val service = BatteryTemperatureService(context)

        // check
        assertSame(context, service.context)
        assertNull(service.batteryTemperatureChangedListener)
        assertNull(service.currentBatteryTemperatureCelsius)
        assertNull(service.currentBatteryTemperature)
        assertFalse(service.getCurrentBatteryTemperature(Temperature(0.0, TemperatureUnit.CELSIUS)))
        assertFalse(service.running)
    }

    @Test
    fun constructor_whenAllParameters_setsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val service = BatteryTemperatureService(context, listener)

        // check
        assertSame(context, service.context)
        assertSame(listener, service.batteryTemperatureChangedListener)
        assertNull(service.currentBatteryTemperatureCelsius)
        assertNull(service.currentBatteryTemperature)
        assertFalse(service.getCurrentBatteryTemperature(Temperature(0.0, TemperatureUnit.CELSIUS)))
        assertFalse(service.running)
    }

    @Test
    fun batteryTemperatureChangedListener_getsAndSetsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val service = BatteryTemperatureService(context)

        // check default value
        assertNull(service.batteryTemperatureChangedListener)

        // set new value
        service.batteryTemperatureChangedListener = listener

        assertSame(listener, service.batteryTemperatureChangedListener)
    }

    @SuppressLint("UnspecifiedRegisterReceiverFlag")
    @Test
    fun currentBatteryTemperatureCelsius_whenNotAvailable_returnsNull() {
        doReturn(null).whenever(contextSpy).registerReceiver(eq(null), any())
//        every { contextSpy.registerReceiver(null, any()) }.returns(null)

        val service = BatteryTemperatureService(contextSpy)

        assertNull(service.currentBatteryTemperatureCelsius)
    }

    @SuppressLint("UnspecifiedRegisterReceiverFlag")
    @Test
    fun currentBatteryTemperatureCelsius_whenAvailable_returnsExpectedValue() {
        doReturn(300).whenever(intent).getIntExtra(BatteryManager.EXTRA_TEMPERATURE, 0)
//        every { intent.getIntExtra(BatteryManager.EXTRA_TEMPERATURE, 0) }.returns(300)
        doReturn(intent).whenever(contextSpy).registerReceiver(eq(null), any())
//        every { contextSpy.registerReceiver(null, any()) }.returns(intent)

        val service = BatteryTemperatureService(contextSpy)

        val temperature = service.currentBatteryTemperatureCelsius
        requireNotNull(temperature)
        assertEquals(30.0f, temperature, 0.0f)
    }

    @SuppressLint("UnspecifiedRegisterReceiverFlag")
    @Test
    fun currentBatteryTemperature_whenNotAvailable_returnsNull() {
        doReturn(null).whenever(contextSpy).registerReceiver(eq(null), any())
//        every { contextSpy.registerReceiver(null, any()) }.returns(null)

        val service = BatteryTemperatureService(contextSpy)

        assertNull(service.currentBatteryTemperature)
    }

    @SuppressLint("UnspecifiedRegisterReceiverFlag")
    @Test
    fun currentBatteryTemperature_whenAvailable_returnsExpectedValue() {
        doReturn(300).whenever(intent).getIntExtra(BatteryManager.EXTRA_TEMPERATURE, 0)
//        every { intent.getIntExtra(BatteryManager.EXTRA_TEMPERATURE, 0) }.returns(300)
        doReturn(intent).whenever(contextSpy).registerReceiver(eq(null), any())
//        every { contextSpy.registerReceiver(null, any()) }.returns(intent)

        val service = BatteryTemperatureService(contextSpy)

        val temperature = service.currentBatteryTemperature
        requireNotNull(temperature)
        assertEquals(30.0, temperature.value.toDouble(), 0.0)
        assertEquals(TemperatureUnit.CELSIUS, temperature.unit)
    }

    @SuppressLint("UnspecifiedRegisterReceiverFlag")
    @Test
    fun getCurrentBatteryTemperature_whenNotAvailable_returnsNull() {
        doReturn(null).whenever(contextSpy).registerReceiver(eq(null), any())
//        every { contextSpy.registerReceiver(null, any()) }.returns(null)

        val service = BatteryTemperatureService(contextSpy)

        val temperature = Temperature(1.0, TemperatureUnit.KELVIN)
        assertFalse(service.getCurrentBatteryTemperature(temperature))

        // check that temperature has not been modified
        assertEquals(1.0, temperature.value.toDouble(), 0.0)
        assertEquals(TemperatureUnit.KELVIN, temperature.unit)
    }

    @SuppressLint("UnspecifiedRegisterReceiverFlag")
    @Test
    fun getCurrentBatteryTemperature_whenAvailable_returnsExpectedValue() {
        doReturn(300).whenever(intent).getIntExtra(BatteryManager.EXTRA_TEMPERATURE, 0)
//        every { intent.getIntExtra(BatteryManager.EXTRA_TEMPERATURE, 0) }.returns(300)
        doReturn(intent).whenever(contextSpy).registerReceiver(eq(null), any())
//        every { contextSpy.registerReceiver(null, any()) }.returns(intent)

        val service = BatteryTemperatureService(contextSpy)

        val temperature = Temperature(1.0, TemperatureUnit.KELVIN)
        assertTrue(service.getCurrentBatteryTemperature(temperature))

        // check that temperature has been modified
        assertEquals(30.0, temperature.value.toDouble(), 0.0)
        assertEquals(TemperatureUnit.CELSIUS, temperature.unit)
    }

    @SuppressLint("UnspecifiedRegisterReceiverFlag")
    @Test
    fun start_whenNotRunning_registersReceiver() {
        val service = BatteryTemperatureService(contextSpy)

        val receiver: BroadcastReceiver? = service.getPrivateProperty("receiver")
        requireNotNull(receiver)
        assertFalse(service.running)

        service.start()

        verify(contextSpy, only()).registerReceiver(
            eq(receiver),
            capture(intentFilterCaptor)
        )
/*        val slot = slot<IntentFilter>()
        verify(exactly = 1) {
            contextSpy.registerReceiver(
                receiver,
                capture(slot)
            )
        }*/

        val intentFilter = intentFilterCaptor.value
//        val intentFilter = slot.captured
        assertEquals(1, intentFilter.countActions())
        assertEquals(Intent.ACTION_BATTERY_CHANGED, intentFilter.getAction(0))

        assertTrue(service.running)
    }

    @Test
    fun start_whenRunning_makesNoAction() {
        val service = BatteryTemperatureService(contextSpy)

        service.setPrivateProperty("running", true)
        assertTrue(service.running)

        service.start()

        verifyNoInteractions(contextSpy)
//        verify { contextSpy wasNot Called }
        assertTrue(service.running)
    }

    @Test
    fun stop_whenNotRunning_makesNoAction() {
        val service = BatteryTemperatureService(contextSpy)

        assertFalse(service.running)

        service.stop()

        verifyNoInteractions(contextSpy)
//        verify { contextSpy wasNot Called }
        assertFalse(service.running)
    }

    @Test
    fun stop_whenRunning_unregistersReceiver() {
        val service = BatteryTemperatureService(contextSpy)

        val receiver: BroadcastReceiver? = service.getPrivateProperty("receiver")
        requireNotNull(receiver)
        assertFalse(service.running)

        service.start()

        assertTrue(service.running)

        service.stop()

        verify(contextSpy, times(1)).unregisterReceiver(receiver)
//        verify(exactly = 1) { contextSpy.unregisterReceiver(receiver) }
        assertFalse(service.running)
    }
}