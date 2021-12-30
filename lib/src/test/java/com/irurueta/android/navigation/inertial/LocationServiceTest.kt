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
package com.irurueta.android.navigation.inertial

import android.content.Context
import android.location.Location
import android.location.LocationListener
import android.location.LocationManager
import android.os.Build
import android.os.CancellationSignal
import android.os.Looper
import androidx.test.core.app.ApplicationProvider
import com.google.android.gms.common.ConnectionResult
import com.google.android.gms.common.GoogleApiAvailability
import com.google.android.gms.location.*
import com.google.android.gms.tasks.CancellationTokenSource
import com.google.android.gms.tasks.OnSuccessListener
import com.google.android.gms.tasks.Task
import io.mockk.*
import org.junit.Assert.*
import org.junit.Test
import org.junit.runner.RunWith
import org.robolectric.RobolectricTestRunner
import org.robolectric.annotation.Config
import java.util.function.Consumer

@RunWith(RobolectricTestRunner::class)
class LocationServiceTest {

    @Test
    fun constructor_setsDefaultValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val service = LocationService(context)

        // check
        assertSame(context, service.context)
        assertEquals(
            LocationService.GooglePlayStatus.SERVICE_INVALID,
            service.googlePlayServicesStatus
        )
        assertFalse(service.googlePlayServicesAvailable)
        val locationEnabled = service.locationEnabled
        requireNotNull(locationEnabled)
        assertTrue(locationEnabled)
        assertEquals(LocationService.DEFAULT_SMALLEST_DISPLACEMENT, service.smallestDisplacement)
        assertEquals(LocationService.DEFAULT_UPDATE_INTERVAL, service.updateInterval)
        assertNull(service.locationUpdateListener)
    }

    @Test
    fun constructor_whenNoLocationManager_setsDefaultValues() {
        val context = spyk(ApplicationProvider.getApplicationContext())
        every { context.getSystemService(Context.LOCATION_SERVICE) }
            .returns(null as LocationManager?)
        val service = LocationService(context)

        val locationManager: LocationManager? = service.getPrivateProperty("locationManager")
        assertNull(locationManager)

        val fusedLocationClient: FusedLocationProviderClient? =
            service.getPrivateProperty("fusedLocationClient")
        assertNull(fusedLocationClient)

        val cancellationTokenSource: CancellationTokenSource? =
            service.getPrivateProperty("cancellationTokenSource")
        assertNull(cancellationTokenSource)
    }

    @Test
    fun googlePlayServicesStatus_callsExpectedMethod() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val service = LocationService(context)

        val googleApiAvailability = mockk<GoogleApiAvailability>()
        every { googleApiAvailability.isGooglePlayServicesAvailable(context) }.returns(
            ConnectionResult.SUCCESS
        )
        mockkStatic(GoogleApiAvailability::class)
        every {
            GoogleApiAvailability.getInstance()
        }.returns(googleApiAvailability)

        assertEquals(LocationService.GooglePlayStatus.SUCCESS, service.googlePlayServicesStatus)

        verify(exactly = 1) { GoogleApiAvailability.getInstance() }
        verify(exactly = 1) { googleApiAvailability.isGooglePlayServicesAvailable(context) }

        unmockkStatic(GoogleApiAvailability::class)
    }

    @Test
    fun googlePlayServicesAvailable_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val service = LocationService(context)

        val googleApiAvailability = mockk<GoogleApiAvailability>()
        every { googleApiAvailability.isGooglePlayServicesAvailable(context) }.returns(
            ConnectionResult.SUCCESS
        )
        mockkStatic(GoogleApiAvailability::class)
        every {
            GoogleApiAvailability.getInstance()
        }.returns(googleApiAvailability)

        assertTrue(service.googlePlayServicesAvailable)

        verify(exactly = 1) { GoogleApiAvailability.getInstance() }
        verify(exactly = 1) { googleApiAvailability.isGooglePlayServicesAvailable(context) }

        unmockkStatic(GoogleApiAvailability::class)
    }

    @Config(sdk = [Build.VERSION_CODES.P])
    @Test
    fun locationEnabled_whenLocationManagerAvailableAndSdkP_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val service = LocationService(context)

        val locationManager: LocationManager? = service.getPrivateProperty("locationManager")
        requireNotNull(locationManager)
        val locationManagerSpy = spyk(locationManager)
        every { locationManagerSpy.isLocationEnabled }.returns(true)
        service.setPrivateProperty("locationManager", locationManagerSpy)

        val locationEnabled = service.locationEnabled
        requireNotNull(locationEnabled)
        assertTrue(locationEnabled)
    }

    @Config(sdk = [Build.VERSION_CODES.P])
    @Test
    fun locationEnabled_whenNoLocationManagerAvailableAndSdkP_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val service = LocationService(context)

        service.setPrivateProperty("locationManager", null)

        assertNull(service.locationEnabled)
    }

    @Config(sdk = [Build.VERSION_CODES.O])
    @Test
    fun locationEnabled_whenSdkO_returnsNull() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val service = LocationService(context)

        assertNull(service.locationEnabled)
    }

    @Test
    fun getLastKnownLocation_whenProviderEnabled_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val service = LocationService(context)

        val locationManager: LocationManager? = service.getPrivateProperty("locationManager")
        requireNotNull(locationManager)
        val locationManagerSpy = spyk(locationManager)
        every { locationManagerSpy.isProviderEnabled(LocationService.FUSED_PROVIDER) }.returns(true)
        val location = mockk<Location>()
        every { locationManagerSpy.getLastKnownLocation(LocationService.FUSED_PROVIDER) }.returns(
            location
        )
        service.setPrivateProperty("locationManager", locationManagerSpy)

        assertSame(location, service.getLastKnownLocation())
    }

    @Test
    fun getLastKnownLocation_whenProviderDisabled_returnsNull() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val service = LocationService(context)

        val locationManager: LocationManager? = service.getPrivateProperty("locationManager")
        requireNotNull(locationManager)
        val locationManagerSpy = spyk(locationManager)
        every { locationManagerSpy.isProviderEnabled(LocationService.FUSED_PROVIDER) }.returns(false)
        service.setPrivateProperty("locationManager", locationManagerSpy)

        assertNull(service.getLastKnownLocation())
    }

    @Test
    fun getLastKnownLocation_whenNoProvider_returnsNull() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val service = LocationService(context)

        service.setPrivateProperty("locationManager", null)

        assertNull(service.getLastKnownLocation())
    }

    @Test
    fun getCurrentLocation_whenFusedClientAndTokenSource_callsExpectedMethods() {
        val context = spyk(ApplicationProvider.getApplicationContext())

        val googleApiAvailability = mockk<GoogleApiAvailability>()
        every { googleApiAvailability.isGooglePlayServicesAvailable(context) }.returns(
            ConnectionResult.SUCCESS
        )
        mockkStatic(GoogleApiAvailability::class)
        every {
            GoogleApiAvailability.getInstance()
        }.returns(googleApiAvailability)

        val service = LocationService(context)

        val fusedLocationClient: FusedLocationProviderClient? =
            service.getPrivateProperty("fusedLocationClient")
        val cancellationTokenSource: CancellationTokenSource? =
            service.getPrivateProperty("cancellationTokenSource")

        requireNotNull(fusedLocationClient)
        requireNotNull(cancellationTokenSource)

        val fusedLocationClientSpy = spyk(fusedLocationClient)
        val cancellationTokenSourceSpy = spyk(cancellationTokenSource)
        service.setPrivateProperty("fusedLocationClient", fusedLocationClientSpy)
        service.setPrivateProperty("cancellationTokenSource", cancellationTokenSourceSpy)

        val location = mockk<Location>()
        val task = mockk<Task<Location>>()
        every { task.addOnSuccessListener(any()) }.answers { answer ->
            @Suppress("UNCHECKED_CAST")
            val listener = answer.invocation.args[0] as OnSuccessListener<Location>
            listener.onSuccess(location)
            return@answers task
        }
        every {
            fusedLocationClientSpy.getCurrentLocation(
                LocationRequest.PRIORITY_HIGH_ACCURACY, any()
            )
        }.returns(task)

        val listener = mockk<LocationService.OnCurrentLocationListener>()
        justRun { listener.onCurrentLocation(any()) }
        service.getCurrentLocation(listener)

        verify(exactly = 1) {
            fusedLocationClientSpy.getCurrentLocation(
                LocationRequest.PRIORITY_HIGH_ACCURACY,
                any()
            )
        }
        verify(exactly = 1) { task.addOnSuccessListener(any()) }
        verify(exactly = 1) { listener.onCurrentLocation(location) }

        unmockkStatic(GoogleApiAvailability::class)
    }

    @Config(sdk = [Build.VERSION_CODES.R])
    @Test
    fun getCurrentLocation_whenNoFusedClientAndNoPreviousRequestAndSdkR_callsExpectedMethods() {
        val context = spyk(ApplicationProvider.getApplicationContext())

        val service = LocationService(context)

        val fusedLocationClient: FusedLocationProviderClient? =
            service.getPrivateProperty("fusedLocationClient")
        val cancellationTokenSource: CancellationTokenSource? =
            service.getPrivateProperty("cancellationTokenSource")

        assertNull(fusedLocationClient)
        assertNull(cancellationTokenSource)

        val cancellationSignal1: CancellationSignal? =
            service.getPrivateProperty("cancellationSignal")
        assertNull(cancellationSignal1)

        val location = mockk<Location>()
        val locationManager: LocationManager? = service.getPrivateProperty("locationManager")
        requireNotNull(locationManager)
        val locationManagerSpy = spyk(locationManager)
        service.setPrivateProperty("locationManager", locationManagerSpy)
        every {
            locationManagerSpy.getCurrentLocation(
                LocationService.FUSED_PROVIDER,
                any(),
                any(),
                any<Consumer<Location>>()
            )
        }.answers { answer ->
            val cancellationSignal = answer.invocation.args[1]
            assertNotNull(cancellationSignal)
            @Suppress("UNCHECKED_CAST")
            val consumer = answer.invocation.args[3] as Consumer<Location>
            consumer.accept(location)
        }

        val listener = mockk<LocationService.OnCurrentLocationListener>()
        justRun { listener.onCurrentLocation(any()) }
        service.getCurrentLocation(listener)

        val cancellationSignal2: CancellationSignal? =
            service.getPrivateProperty("cancellationSignal")
        assertNull(cancellationSignal2)

        verify(exactly = 1) {
            locationManagerSpy.getCurrentLocation(
                LocationService.FUSED_PROVIDER,
                any(),
                any(),
                any<Consumer<Location>>()
            )
        }
        verify(exactly = 1) { listener.onCurrentLocation(location) }
    }

    @Config(sdk = [Build.VERSION_CODES.R])
    @Test
    fun getCurrentLocation_whenNoFusedClientAndPreviousRequestAndSdkR_callsExpectedMethods() {
        val context = spyk(ApplicationProvider.getApplicationContext())

        val service = LocationService(context)

        val fusedLocationClient: FusedLocationProviderClient? =
            service.getPrivateProperty("fusedLocationClient")
        val cancellationTokenSource: CancellationTokenSource? =
            service.getPrivateProperty("cancellationTokenSource")

        assertNull(fusedLocationClient)
        assertNull(cancellationTokenSource)

        val cancellationSignal1: CancellationSignal? =
            service.getPrivateProperty("cancellationSignal")
        assertNull(cancellationSignal1)

        // set previous cancellation signal
        val cancellationSignal2 = spyk(CancellationSignal())
        service.setPrivateProperty("cancellationSignal", cancellationSignal2)

        val location = mockk<Location>()
        val locationManager: LocationManager? = service.getPrivateProperty("locationManager")
        requireNotNull(locationManager)
        val locationManagerSpy = spyk(locationManager)
        service.setPrivateProperty("locationManager", locationManagerSpy)
        every {
            locationManagerSpy.getCurrentLocation(
                LocationService.FUSED_PROVIDER,
                any(),
                any(),
                any<Consumer<Location>>()
            )
        }.answers { answer ->
            val cancellationSignal = answer.invocation.args[1]
            assertNotNull(cancellationSignal)
            assertNotSame(cancellationSignal2, cancellationSignal)
            @Suppress("UNCHECKED_CAST")
            val consumer = answer.invocation.args[3] as Consumer<Location>
            consumer.accept(location)
        }

        val listener = mockk<LocationService.OnCurrentLocationListener>()
        justRun { listener.onCurrentLocation(any()) }
        service.getCurrentLocation(listener)

        val cancellationSignal3: CancellationSignal? =
            service.getPrivateProperty("cancellationSignal")
        assertNull(cancellationSignal3)

        verify(exactly = 1) { cancellationSignal2.cancel() }
        verify(exactly = 1) {
            locationManagerSpy.getCurrentLocation(
                LocationService.FUSED_PROVIDER,
                any(),
                any(),
                any<Consumer<Location>>()
            )
        }
        verify(exactly = 1) { listener.onCurrentLocation(location) }
    }

    @Config(sdk = [Build.VERSION_CODES.R])
    @Test
    fun getCurrentLocation_whenNoFusedClientNoLocationManagerAndSdkR_callsExpectedMethods() {
        val context = spyk(ApplicationProvider.getApplicationContext())

        val service = LocationService(context)

        val fusedLocationClient: FusedLocationProviderClient? =
            service.getPrivateProperty("fusedLocationClient")
        val cancellationTokenSource: CancellationTokenSource? =
            service.getPrivateProperty("cancellationTokenSource")

        assertNull(fusedLocationClient)
        assertNull(cancellationTokenSource)

        val cancellationSignal1: CancellationSignal? =
            service.getPrivateProperty("cancellationSignal")
        assertNull(cancellationSignal1)

        // set previous cancellation signal
        val cancellationSignal2 = spyk(CancellationSignal())
        service.setPrivateProperty("cancellationSignal", cancellationSignal2)

        val locationManager: LocationManager? = service.getPrivateProperty("locationManager")
        assertNotNull(locationManager)
        service.setPrivateProperty("locationManager", null)

        val listener = mockk<LocationService.OnCurrentLocationListener>()
        justRun { listener.onCurrentLocation(any()) }
        service.getCurrentLocation(listener)

        val cancellationSignal3: CancellationSignal? =
            service.getPrivateProperty("cancellationSignal")
        assertNotNull(cancellationSignal3)
        assertNotSame(cancellationSignal2, cancellationSignal3)

        verify { listener wasNot Called }
    }

    @Config(sdk = [Build.VERSION_CODES.Q])
    @Test
    fun getCurrentLocation_whenNoFusedClientAndPreviousRequestAndSdkQ_callsExpectedMethods() {
        val context = spyk(ApplicationProvider.getApplicationContext())

        val service = LocationService(context)

        val fusedLocationClient: FusedLocationProviderClient? =
            service.getPrivateProperty("fusedLocationClient")
        val cancellationTokenSource: CancellationTokenSource? =
            service.getPrivateProperty("cancellationTokenSource")

        assertNull(fusedLocationClient)
        assertNull(cancellationTokenSource)

        val location = mockk<Location>()
        val locationManager: LocationManager? = service.getPrivateProperty("locationManager")
        requireNotNull(locationManager)
        val locationManagerSpy = spyk(locationManager)
        service.setPrivateProperty("locationManager", locationManagerSpy)
        every {
            @Suppress("DEPRECATION")
            locationManagerSpy.requestSingleUpdate(LocationService.FUSED_PROVIDER, any(), any())
        }.answers { answer ->
            val listener = answer.invocation.args[1] as LocationListener
            listener.onLocationChanged(location)
        }

        val listener = mockk<LocationService.OnCurrentLocationListener>()
        justRun { listener.onCurrentLocation(any()) }
        service.getCurrentLocation(listener)

        verify(exactly = 1) {
            @Suppress("DEPRECATION")
            locationManagerSpy.requestSingleUpdate(LocationService.FUSED_PROVIDER, any(), any())
        }
        verify(exactly = 1) { listener.onCurrentLocation(location) }
    }

    @Test
    fun getCurrentLocation_whenNoLocationManagerAndSdkQ_makesNoAction() {
        val context = spyk(ApplicationProvider.getApplicationContext())

        val service = LocationService(context)

        val fusedLocationClient: FusedLocationProviderClient? =
            service.getPrivateProperty("fusedLocationClient")
        val cancellationTokenSource: CancellationTokenSource? =
            service.getPrivateProperty("cancellationTokenSource")

        assertNull(fusedLocationClient)
        assertNull(cancellationTokenSource)

        val locationManager: LocationManager? = service.getPrivateProperty("locationManager")
        assertNotNull(locationManager)
        service.setPrivateProperty("locationManager", null)

        val listener = mockk<LocationService.OnCurrentLocationListener>()
        justRun { listener.onCurrentLocation(any()) }
        service.getCurrentLocation(listener)

        verify { listener wasNot Called }
    }

    @Config(sdk = [Build.VERSION_CODES.R])
    @Test
    fun getCurrentLocation_whenFusedClientAndNoCancellationTokenSource() {
        val context = spyk(ApplicationProvider.getApplicationContext())

        val googleApiAvailability = mockk<GoogleApiAvailability>()
        every { googleApiAvailability.isGooglePlayServicesAvailable(context) }.returns(
            ConnectionResult.SUCCESS
        )
        mockkStatic(GoogleApiAvailability::class)
        every {
            GoogleApiAvailability.getInstance()
        }.returns(googleApiAvailability)

        val service = LocationService(context)

        val fusedLocationClient: FusedLocationProviderClient? =
            service.getPrivateProperty("fusedLocationClient")
        val cancellationTokenSource: CancellationTokenSource? =
            service.getPrivateProperty("cancellationTokenSource")

        requireNotNull(fusedLocationClient)
        requireNotNull(cancellationTokenSource)

        service.setPrivateProperty("cancellationTokenSource", null)

        val cancellationSignal1: CancellationSignal? =
            service.getPrivateProperty("cancellationSignal")
        assertNull(cancellationSignal1)

        val location = mockk<Location>()
        val locationManager: LocationManager? = service.getPrivateProperty("locationManager")
        requireNotNull(locationManager)
        val locationManagerSpy = spyk(locationManager)
        service.setPrivateProperty("locationManager", locationManagerSpy)
        every {
            locationManagerSpy.getCurrentLocation(
                LocationService.FUSED_PROVIDER,
                any(),
                any(),
                any<Consumer<Location>>()
            )
        }.answers { answer ->
            val cancellationSignal = answer.invocation.args[1]
            assertNotNull(cancellationSignal)
            @Suppress("UNCHECKED_CAST")
            val consumer = answer.invocation.args[3] as Consumer<Location>
            consumer.accept(location)
        }

        val listener = mockk<LocationService.OnCurrentLocationListener>()
        justRun { listener.onCurrentLocation(any()) }
        service.getCurrentLocation(listener)

        val cancellationSignal2: CancellationSignal? =
            service.getPrivateProperty("cancellationSignal")
        assertNull(cancellationSignal2)

        verify(exactly = 1) {
            locationManagerSpy.getCurrentLocation(
                LocationService.FUSED_PROVIDER,
                any(),
                any(),
                any<Consumer<Location>>()
            )
        }
        verify(exactly = 1) { listener.onCurrentLocation(location) }

        unmockkStatic(GoogleApiAvailability::class)
    }

    @Config(sdk = [Build.VERSION_CODES.Q])
    @Test
    fun getCurrentLocation_whenNoLocationManagerSdkQ_makesNoAction() {
        val context = spyk(ApplicationProvider.getApplicationContext())

        val service = LocationService(context)

        val fusedLocationClient: FusedLocationProviderClient? =
            service.getPrivateProperty("fusedLocationClient")
        val cancellationTokenSource: CancellationTokenSource? =
            service.getPrivateProperty("cancellationTokenSource")

        assertNull(fusedLocationClient)
        assertNull(cancellationTokenSource)

        val locationManager: LocationManager? = service.getPrivateProperty("locationManager")
        assertNotNull(locationManager)
        service.setPrivateProperty("locationManager", null)

        val listener = mockk<LocationService.OnCurrentLocationListener>()
        service.getCurrentLocation(listener)

        verify { listener wasNot Called }
    }

    @Test
    fun cancelCurrentLocation_whenNoPreviousTokenOrSignal_makesNoAction() {
        val context = spyk(ApplicationProvider.getApplicationContext())
        val service = LocationService(context)

        val cancellationTokenSource: CancellationTokenSource? =
            service.getPrivateProperty("cancellationTokenSource")
        assertNull(cancellationTokenSource)
        val cancellationSignal1: CancellationSignal? =
            service.getPrivateProperty("cancellationSignal")
        assertNull(cancellationSignal1)

        service.cancelCurrentLocation()

        val cancellationSignal2: CancellationSignal? =
            service.getPrivateProperty("cancellationSignal")
        assertNull(cancellationSignal2)
    }

    @Test
    fun cancelCurrentLocation_whenCancellationTokenSource_callsCancelMethod() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val service = LocationService(context)

        val cancellationTokenSource = mockk<CancellationTokenSource>()
        justRun { cancellationTokenSource.cancel() }
        service.setPrivateProperty("cancellationTokenSource", cancellationTokenSource)

        service.cancelCurrentLocation()

        verify(exactly = 1) { cancellationTokenSource.cancel() }
    }

    @Test
    fun cancelCurrentLocation_whenCancellationSignal_callsCancelMethod() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val service = LocationService(context)

        val cancellationSignal = mockk<CancellationSignal>()
        justRun { cancellationSignal.cancel() }
        service.setPrivateProperty("cancellationSignal", cancellationSignal)

        service.cancelCurrentLocation()

        verify(exactly = 1) { cancellationSignal.cancel() }
    }

    @Test
    fun smallestDisplacement_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val service = LocationService(context)

        // check default value
        assertEquals(LocationService.DEFAULT_SMALLEST_DISPLACEMENT, service.smallestDisplacement)

        // set new value
        service.smallestDisplacement = 1.0f

        // check
        assertEquals(1.0f, service.smallestDisplacement)
    }

    @Test(expected = IllegalArgumentException::class)
    fun smallestDisplacement_whenNegative_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val service = LocationService(context)

        service.smallestDisplacement = -1.0f
    }

    @Test
    fun updateInterval_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val service = LocationService(context)

        // check default value
        assertEquals(LocationService.DEFAULT_UPDATE_INTERVAL, service.updateInterval)

        // set new value
        service.updateInterval = 2000L

        // check
        assertEquals(2000L, service.updateInterval)
    }

    @Test(expected = IllegalArgumentException::class)
    fun updateInterval_whenNegative_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val service = LocationService(context)

        service.updateInterval = -1L
    }

    @Test
    fun locationUpdateListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val service = LocationService(context)

        // check default value
        assertNull(service.locationUpdateListener)

        // set new value
        val listener = mockk<LocationService.OnLocationUpdateListener>()
        service.locationUpdateListener = listener

        // check
        assertSame(listener, service.locationUpdateListener)
    }

    @Test
    fun requestLocationUpdates_whenFusedLocationClientAvailableAndNoListener_callsExpectedMethods() {
        val context = ApplicationProvider.getApplicationContext<Context>()

        val googleApiAvailability = mockk<GoogleApiAvailability>()
        every { googleApiAvailability.isGooglePlayServicesAvailable(context) }.returns(
            ConnectionResult.SUCCESS
        )
        mockkStatic(GoogleApiAvailability::class)
        every {
            GoogleApiAvailability.getInstance()
        }.returns(googleApiAvailability)

        val service = LocationService(context)

        val fusedLocationClient: FusedLocationProviderClient? =
            service.getPrivateProperty("fusedLocationClient")
        val cancellationTokenSource: CancellationTokenSource? =
            service.getPrivateProperty("cancellationTokenSource")

        requireNotNull(fusedLocationClient)
        requireNotNull(cancellationTokenSource)

        val fusedLocationClientSpy = spyk(fusedLocationClient)
        service.setPrivateProperty("fusedLocationClient", fusedLocationClientSpy)

        val requestTask = mockk<Task<Void>>()
        every { fusedLocationClientSpy.requestLocationUpdates(any(), any(), any()) }.returns(
            requestTask
        )

        service.requestLocationUpdates()

        // check
        val locationRequestSlot = slot<LocationRequest>()
        val callbackSlot = slot<LocationCallback>()
        verify(exactly = 1) {
            fusedLocationClientSpy.requestLocationUpdates(
                capture(locationRequestSlot),
                capture(callbackSlot),
                any()
            )
        }

        val locationRequest = locationRequestSlot.captured
        assertEquals(service.smallestDisplacement, locationRequest.smallestDisplacement)
        assertEquals(service.updateInterval, locationRequest.interval)

        // execute callback methods
        val callback = callbackSlot.captured
        val locationAvailability = mockk<LocationAvailability>()
        callback.onLocationAvailability(locationAvailability)

        val locationResult = mockk<LocationResult>()
        callback.onLocationResult(locationResult)

        unmockkStatic(GoogleApiAvailability::class)
    }

    @Test
    fun requestLocationUpdates_whenFusedLocationClientAvailableAndListener_callsExpectedMethods() {
        val context = ApplicationProvider.getApplicationContext<Context>()

        val googleApiAvailability = mockk<GoogleApiAvailability>()
        every { googleApiAvailability.isGooglePlayServicesAvailable(context) }.returns(
            ConnectionResult.SUCCESS
        )
        mockkStatic(GoogleApiAvailability::class)
        every {
            GoogleApiAvailability.getInstance()
        }.returns(googleApiAvailability)

        val service = LocationService(context)
        val listener = mockk<LocationService.OnLocationUpdateListener>()
        justRun { listener.onLocationChanged(any()) }
        justRun { listener.onLocationAvailability(any()) }
        service.locationUpdateListener = listener

        val fusedLocationClient: FusedLocationProviderClient? =
            service.getPrivateProperty("fusedLocationClient")
        val cancellationTokenSource: CancellationTokenSource? =
            service.getPrivateProperty("cancellationTokenSource")

        requireNotNull(fusedLocationClient)
        requireNotNull(cancellationTokenSource)

        val fusedLocationClientSpy = spyk(fusedLocationClient)
        service.setPrivateProperty("fusedLocationClient", fusedLocationClientSpy)

        val requestTask = mockk<Task<Void>>()
        every { fusedLocationClientSpy.requestLocationUpdates(any(), any(), any()) }
            .returns(requestTask)

        service.requestLocationUpdates()

        // check
        val locationRequestSlot = slot<LocationRequest>()
        val callbackSlot = slot<LocationCallback>()
        verify(exactly = 1) {
            fusedLocationClientSpy.requestLocationUpdates(
                capture(locationRequestSlot),
                capture(callbackSlot),
                any()
            )
        }

        val locationRequest = locationRequestSlot.captured
        assertEquals(service.smallestDisplacement, locationRequest.smallestDisplacement)
        assertEquals(service.updateInterval, locationRequest.interval)

        // execute callback methods
        val callback = callbackSlot.captured
        val locationAvailability = mockk<LocationAvailability>()
        every { locationAvailability.isLocationAvailable }.returns(true)
        callback.onLocationAvailability(locationAvailability)

        val locationResult = mockk<LocationResult>()
        val location = mockk<Location>()
        every { locationResult.lastLocation }.returns(location)
        callback.onLocationResult(locationResult)

        verify(exactly = 1) { listener.onLocationChanged(location) }
        verify(exactly = 1) { listener.onLocationAvailability(true) }

        unmockkStatic(GoogleApiAvailability::class)
    }

    @Test
    fun requestLocationUpdates_whenNoFusedLocationClientAndLocationManagerAvailable_callsExpectedMethods() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val service = LocationService(context)

        val fusedLocationClient: FusedLocationProviderClient? =
            service.getPrivateProperty("fusedLocationClient")
        val cancellationTokenSource: CancellationTokenSource? =
            service.getPrivateProperty("cancellationTokenSource")

        assertNull(fusedLocationClient)
        assertNull(cancellationTokenSource)

        val locationManager: LocationManager? = service.getPrivateProperty("locationManager")
        requireNotNull(locationManager)
        val locationManagerSpy = spyk(locationManager)
        service.setPrivateProperty("locationManager", locationManagerSpy)

        service.requestLocationUpdates()

        // check
        verify(exactly = 1) { locationManagerSpy.removeUpdates(any<LocationListener>()) }
        verify(exactly = 1) {
            locationManagerSpy.requestLocationUpdates(
                LocationService.FUSED_PROVIDER,
                service.updateInterval,
                service.smallestDisplacement,
                any<LocationListener>(),
                any()
            )
        }
    }

    @Test
    fun requestLocationUpdates_whenNoFusedLocationClientAndNoLocationManager_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val service = LocationService(context)

        val fusedLocationClient: FusedLocationProviderClient? =
            service.getPrivateProperty("fusedLocationClient")
        val cancellationTokenSource: CancellationTokenSource? =
            service.getPrivateProperty("cancellationTokenSource")

        assertNull(fusedLocationClient)
        assertNull(cancellationTokenSource)

        val locationManager: LocationManager? = service.getPrivateProperty("locationManager")
        assertNotNull(locationManager)
        service.setPrivateProperty("locationManager", null)

        service.requestLocationUpdates()
    }

    @Test
    fun cancelLocationUpdates_whenFusedClient_executesExpectedMethod() {
        val context = ApplicationProvider.getApplicationContext<Context>()

        val googleApiAvailability = mockk<GoogleApiAvailability>()
        every { googleApiAvailability.isGooglePlayServicesAvailable(context) }.returns(
            ConnectionResult.SUCCESS
        )
        mockkStatic(GoogleApiAvailability::class)
        every {
            GoogleApiAvailability.getInstance()
        }.returns(googleApiAvailability)

        val service = LocationService(context)

        val fusedLocationClient: FusedLocationProviderClient? =
            service.getPrivateProperty("fusedLocationClient")
        requireNotNull(fusedLocationClient)
        val fusedLocationClientSpy = spyk(fusedLocationClient)
        service.setPrivateProperty("fusedLocationClient", fusedLocationClientSpy)

        val locationManager: LocationManager? = service.getPrivateProperty("locationManager")
        requireNotNull(locationManager)
        val locationManagerSpy = spyk(locationManager)
        service.setPrivateProperty("locationManager", locationManagerSpy)

        service.cancelLocationUpdates()

        // check
        verify(exactly = 1) { fusedLocationClientSpy.removeLocationUpdates(any<LocationCallback>()) }
        verify(exactly = 1) { locationManagerSpy.removeUpdates(any<LocationListener>()) }

        unmockkStatic(GoogleApiAvailability::class)
    }

    @Test
    fun cancelLocationUpdates_whenNoFusedClient_executesExpectedMethod() {
        val context = ApplicationProvider.getApplicationContext<Context>()

        val service = LocationService(context)

        val fusedLocationClient: FusedLocationProviderClient? =
            service.getPrivateProperty("fusedLocationClient")
        assertNull(fusedLocationClient)

        val locationManager: LocationManager? = service.getPrivateProperty("locationManager")
        requireNotNull(locationManager)
        val locationManagerSpy = spyk(locationManager)
        service.setPrivateProperty("locationManager", locationManagerSpy)

        service.cancelLocationUpdates()

        // check
        verify(exactly = 1) { locationManagerSpy.removeUpdates(any<LocationListener>()) }
    }

    @Test
    fun cancelLocationUpdates_whenNoLocationManager_executesExpectedMethod() {
        val context = ApplicationProvider.getApplicationContext<Context>()

        val service = LocationService(context)

        val fusedLocationClient: FusedLocationProviderClient? =
            service.getPrivateProperty("fusedLocationClient")
        assertNull(fusedLocationClient)

        service.setPrivateProperty("locationManager", null)

        service.cancelLocationUpdates()
    }

    @Test
    fun googlePlayStatus_fromValues_returnsExpectedValues() {
        assertEquals(6, LocationService.GooglePlayStatus.values().size)
        assertEquals(
            LocationService.GooglePlayStatus.SUCCESS,
            LocationService.GooglePlayStatus.from(ConnectionResult.SUCCESS)
        )
        assertEquals(
            LocationService.GooglePlayStatus.SERVICE_MISSING,
            LocationService.GooglePlayStatus.from(ConnectionResult.SERVICE_MISSING)
        )
        assertEquals(
            LocationService.GooglePlayStatus.SERVICE_UPDATING,
            LocationService.GooglePlayStatus.from(ConnectionResult.SERVICE_UPDATING)
        )
        assertEquals(
            LocationService.GooglePlayStatus.SERVICE_VERSION_UPDATE_REQUIRED,
            LocationService.GooglePlayStatus.from(ConnectionResult.SERVICE_VERSION_UPDATE_REQUIRED)
        )
        assertEquals(
            LocationService.GooglePlayStatus.SERVICE_DISABLED,
            LocationService.GooglePlayStatus.from(ConnectionResult.SERVICE_DISABLED)
        )
        assertEquals(
            LocationService.GooglePlayStatus.SERVICE_INVALID,
            LocationService.GooglePlayStatus.from(ConnectionResult.SERVICE_INVALID)
        )
    }

    @Test
    fun updatesListener_whenNoListener_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()

        val service = LocationService(context)

        val updatesListener: LocationListener? = service.getPrivateProperty("updatesListener")
        requireNotNull(updatesListener)

        assertNull(service.locationUpdateListener)

        val location = mockk<Location>()
        updatesListener.onLocationChanged(location)
    }

    @Test
    fun updatesListener_whenListener_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()

        val service = LocationService(context)

        val updatesListener: LocationListener? = service.getPrivateProperty("updatesListener")
        requireNotNull(updatesListener)

        assertNull(service.locationUpdateListener)

        val listener = mockk<LocationService.OnLocationUpdateListener>()
        justRun { listener.onLocationChanged(any()) }
        service.locationUpdateListener = listener

        val location = mockk<Location>()
        updatesListener.onLocationChanged(location)

        verify(exactly = 1) { listener.onLocationChanged(location) }
    }

    @Test
    fun getLooper_whenNoLooperOnCurrentThread_getsMainLooper() {
        mockkStatic(Looper::class)
        every { Looper.myLooper() }.returns(null)

        val context = ApplicationProvider.getApplicationContext<Context>()
        val service = LocationService(context)

        val fusedLocationClient: FusedLocationProviderClient? =
            service.getPrivateProperty("fusedLocationClient")
        val cancellationTokenSource: CancellationTokenSource? =
            service.getPrivateProperty("cancellationTokenSource")

        assertNull(fusedLocationClient)
        assertNull(cancellationTokenSource)

        val locationManager: LocationManager? = service.getPrivateProperty("locationManager")
        requireNotNull(locationManager)
        val locationManagerSpy = spyk(locationManager)
        service.setPrivateProperty("locationManager", locationManagerSpy)

        service.requestLocationUpdates()

        verify(exactly = 1) { Looper.myLooper() }
        verify(exactly = 1) { Looper.getMainLooper() }

        unmockkStatic(Looper::class)
    }

    @Test
    fun getLooper_whenLooperOnCurrentThread_getsMainLooper() {
        mockkStatic(Looper::class)
        val looper = mockk<Looper>()
        every { Looper.myLooper() }.returns(looper)

        val context = ApplicationProvider.getApplicationContext<Context>()
        val service = LocationService(context)

        val fusedLocationClient: FusedLocationProviderClient? =
            service.getPrivateProperty("fusedLocationClient")
        val cancellationTokenSource: CancellationTokenSource? =
            service.getPrivateProperty("cancellationTokenSource")

        assertNull(fusedLocationClient)
        assertNull(cancellationTokenSource)

        val locationManager: LocationManager? = service.getPrivateProperty("locationManager")
        requireNotNull(locationManager)
        val locationManagerSpy = spyk(locationManager)
        service.setPrivateProperty("locationManager", locationManagerSpy)

        service.requestLocationUpdates()

        verify(exactly = 1) { Looper.myLooper() }
        verify(exactly = 0) { Looper.getMainLooper() }

        unmockkStatic(Looper::class)
    }
}