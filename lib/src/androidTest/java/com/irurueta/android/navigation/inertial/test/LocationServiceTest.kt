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
package com.irurueta.android.navigation.inertial.test

import android.location.Location
import android.util.Log
import androidx.test.core.app.ActivityScenario
import androidx.test.filters.RequiresDevice
import androidx.test.rule.GrantPermissionRule
import com.irurueta.android.navigation.inertial.LocationService
import com.irurueta.android.navigation.inertial.ThreadSyncHelper
import io.mockk.Called
import io.mockk.mockk
import io.mockk.spyk
import io.mockk.verify
import org.junit.Assert.*
import org.junit.Before
import org.junit.Rule
import org.junit.Test

class LocationServiceTest {

    private val syncHelper = ThreadSyncHelper()

    @Volatile
    private var completed = 0

    private var status: LocationService.GooglePlayStatus? = null

    private var googlePlayServicesAvailable = false

    private var locationEnabled = false

    private var currentLocationListener: LocationService.OnCurrentLocationListener? = null

    private var updateLocationListener: LocationService.OnLocationUpdateListener? = null

    @get:Rule
    val permissionRule: GrantPermissionRule = GrantPermissionRule.grant(
        android.Manifest.permission.ACCESS_COARSE_LOCATION,
        android.Manifest.permission.ACCESS_FINE_LOCATION,
        android.Manifest.permission.ACCESS_BACKGROUND_LOCATION
    )

    @Before
    fun setUp() {
        completed = 0
    }

    @Test
    fun googlePlayServicesStatus_returnsExpectedValue() {
        val scenario = ActivityScenario.launch(LocationActivity::class.java).use {
            it.onActivity { activity ->
                val service = LocationService(activity)

                val status = service.googlePlayServicesStatus
                this.status = status
                assertEquals(LocationService.GooglePlayStatus.SUCCESS, status)

                syncHelper.notifyAll { completed++ }
            }
        }
        assertNotNull(scenario)

        syncHelper.waitOnCondition({ completed < 1 })
        assertEquals(1, completed)
        assertEquals(LocationService.GooglePlayStatus.SUCCESS, status)
    }

    @Test
    fun googlePlayServicesAvailable_returnsExpectedValue() {
        val scenario = ActivityScenario.launch(LocationActivity::class.java).use {
            it.onActivity { activity ->
                val service = LocationService(activity)
                googlePlayServicesAvailable = service.googlePlayServicesAvailable
                assertTrue(googlePlayServicesAvailable)

                syncHelper.notifyAll { completed++ }
            }
        }
        assertNotNull(scenario)

        syncHelper.waitOnCondition({ completed < 1 })
        assertEquals(1, completed)
        assertTrue(googlePlayServicesAvailable)
    }

    @Test
    fun isLocationEnabled_returnsExpectedValue() {
        val scenario = ActivityScenario.launch(LocationActivity::class.java).use {
            it.onActivity { activity ->
                val service = LocationService(activity)

                val enabled = service.locationEnabled
                requireNotNull(enabled)
                locationEnabled = enabled
                assertTrue(enabled)

                syncHelper.notifyAll { completed++ }
            }
        }
        assertNotNull(scenario)

        syncHelper.waitOnCondition({ completed < 1 })
        assertEquals(1, completed)
        assertTrue(locationEnabled)
    }

    @RequiresDevice
    @Test
    fun getLastKnownLocation_whenCurrentLocationIsKnown_returnsExpectedValue() {
        val scenario = ActivityScenario.launch(LocationActivity::class.java).use {
            it.onActivity { activity ->
                val service = LocationService(activity)

                val enabled = service.locationEnabled
                requireNotNull(enabled)
                assertTrue(enabled)

                val currentLocationListener =
                    spyk(object : LocationService.OnCurrentLocationListener {
                        override fun onCurrentLocation(location: Location) {
                            assertNotNull(location)

                            val lastLocation = service.getLastKnownLocation()
                            assertNotSame(location, lastLocation)

                            syncHelper.notifyAll { completed++ }
                        }
                    })

                this.currentLocationListener = currentLocationListener

                service.getCurrentLocation(currentLocationListener)
            }
        }
        assertNotNull(scenario)

        syncHelper.waitOnCondition({ completed < 1 })
        assertEquals(1, completed)
        verify(exactly = 1) { currentLocationListener?.onCurrentLocation(any()) }
    }

    @RequiresDevice
    @Test
    fun getCurrentLocation_returnsExpectedValue() {
        val scenario = ActivityScenario.launch(LocationActivity::class.java).use {
            it.onActivity { activity ->
                val service = LocationService(activity)

                val enabled = service.locationEnabled
                requireNotNull(enabled)
                assertTrue(enabled)

                val currentLocationListener =
                    spyk(object : LocationService.OnCurrentLocationListener {
                        override fun onCurrentLocation(location: Location) {
                            assertNotNull(location)

                            syncHelper.notifyAll { completed++ }
                        }
                    })

                this.currentLocationListener = currentLocationListener

                service.getCurrentLocation(currentLocationListener)
            }
        }
        assertNotNull(scenario)

        syncHelper.waitOnCondition({ completed < 1 })
        assertEquals(1, completed)
        verify(exactly = 1) { currentLocationListener?.onCurrentLocation(any()) }
    }

    @RequiresDevice
    @Test
    fun cancelCurrentLocation_cancelsRequest() {
        val scenario = ActivityScenario.launch(LocationActivity::class.java).use {
            it.onActivity { activity ->
                val service = LocationService(activity)

                val enabled = service.locationEnabled
                requireNotNull(enabled)
                assertTrue(enabled)

                val currentLocationListener = mockk<LocationService.OnCurrentLocationListener>()
                this.currentLocationListener = currentLocationListener

                service.getCurrentLocation(currentLocationListener)
                service.cancelCurrentLocation()
            }
        }
        assertNotNull(scenario)

        val listener = currentLocationListener
        requireNotNull(listener)
        verify { listener wasNot Called }
    }

    @RequiresDevice
    @Test
    fun requestLocationUpdates_obtainsMultipleLocationUpdates() {
        val scenario = ActivityScenario.launch(LocationActivity::class.java).use {
            it.onActivity { activity ->
                val service = LocationService(activity)

                val enabled = service.locationEnabled
                requireNotNull(enabled)
                assertTrue(enabled)

                val updateListener = spyk(object : LocationService.OnLocationUpdateListener {
                    override fun onLocationChanged(location: Location) {
                        assertNotNull(location)

                        Log.i(LocationServiceTest::class.simpleName, "Location update: $location")
                        if (completed >= TIMES) {
                            service.cancelLocationUpdates()
                        }
                        syncHelper.notifyAll { completed++ }
                    }

                    override fun onLocationAvailability(available: Boolean) {
                    }
                })

                this.updateLocationListener = updateListener
                service.locationUpdateListener = updateListener

                service.requestLocationUpdates()
            }
        }
        assertNotNull(scenario)

        syncHelper.waitOnCondition({ completed < TIMES}, timeout = TIMEOUT)
        if (completed > 1) {
            verify(atLeast = 1) { updateLocationListener?.onLocationChanged(any()) }
        }
    }

    private companion object {
        const val TIMES = 5

        const val TIMEOUT = 1000L
    }
}