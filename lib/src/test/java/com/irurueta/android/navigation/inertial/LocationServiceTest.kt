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
import com.irurueta.android.testutils.getPrivateProperty
import com.irurueta.android.testutils.setPrivateProperty
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
import org.mockito.Mockito.mockStatic
import org.mockito.Spy
import org.mockito.junit.MockitoJUnit
import org.mockito.junit.MockitoRule
import org.mockito.kotlin.any
import org.mockito.kotlin.capture
import org.mockito.kotlin.doAnswer
import org.mockito.kotlin.doNothing
import org.mockito.kotlin.doReturn
import org.mockito.kotlin.eq
import org.mockito.kotlin.never
import org.mockito.kotlin.only
import org.mockito.kotlin.spy
import org.mockito.kotlin.times
import org.mockito.kotlin.verify
import org.mockito.kotlin.verifyNoInteractions
import org.mockito.kotlin.whenever
import org.robolectric.RobolectricTestRunner
import org.robolectric.annotation.Config
import java.util.function.Consumer

//@Ignore("Possible memory leak when running this test")
@Suppress("DEPRECATION")
@RunWith(RobolectricTestRunner::class)
class LocationServiceTest {

    @get:Rule
    val mockitoRule: MockitoRule = MockitoJUnit.rule()

//    @get:Rule
//    val mockkRule = MockKRule(this)

//    @MockK
    @Mock
    private lateinit var googleApiAvailability: GoogleApiAvailability

//    @MockK
    @Mock
    private lateinit var location: Location

//    @MockK
    @Mock
    private lateinit var task: Task<Location>

//    @MockK
    @Mock
    private lateinit var currentLocationListener: LocationService.OnCurrentLocationListener

//    @MockK
    @Mock
    private lateinit var cancellationTokenSource: CancellationTokenSource

//    @MockK
    @Mock
    private lateinit var cancellationSignal: CancellationSignal

//    @MockK
    @Mock
    private lateinit var locationUpdateListener: LocationService.OnLocationUpdateListener

//    @MockK
    @Mock
    private lateinit var requestTask: Task<Void>

//    @MockK
    @Mock
    private lateinit var locationAvailability: LocationAvailability

//    @MockK
    @Mock
    private lateinit var locationResult: LocationResult

//    @MockK
    @Mock
    private lateinit var looper: Looper

//    @SpyK
    @Spy
    private var context: Context = ApplicationProvider.getApplicationContext()

//    @SpyK
    @Spy
    private var cancellationSignalSpy = CancellationSignal()

    @Captor
    private lateinit var locationRequestCaptor: ArgumentCaptor<LocationRequest>

    @Captor
    private lateinit var locationCallbackCaptor: ArgumentCaptor<LocationCallback>

    /*@After
    fun tearDown() {
        unmockkAll()
        clearAllMocks()
        System.gc()
    }*/

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
        doReturn(null as LocationManager?).whenever(context).getSystemService(Context.LOCATION_SERVICE)
/*        every { context.getSystemService(Context.LOCATION_SERVICE) }
            .returns(null as LocationManager?)*/
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

        whenever(googleApiAvailability.isGooglePlayServicesAvailable(context)).thenReturn(ConnectionResult.SUCCESS)
/*        every { googleApiAvailability.isGooglePlayServicesAvailable(context) }.returns(
            ConnectionResult.SUCCESS
        )*/
        mockStatic(GoogleApiAvailability::class.java).use { mock ->
            mock.`when`<GoogleApiAvailability> {
                GoogleApiAvailability.getInstance()
            }.thenReturn(googleApiAvailability)

            assertEquals(LocationService.GooglePlayStatus.SUCCESS, service.googlePlayServicesStatus)

            mock.verify({
                GoogleApiAvailability.getInstance()
            }, only())
            verify(googleApiAvailability, only()).isGooglePlayServicesAvailable(context)
        }
/*        mockkStatic(GoogleApiAvailability::class) {
            every {
                GoogleApiAvailability.getInstance()
            }.returns(googleApiAvailability)

            assertEquals(LocationService.GooglePlayStatus.SUCCESS, service.googlePlayServicesStatus)

            verify(exactly = 1) { GoogleApiAvailability.getInstance() }
            verify(exactly = 1) { googleApiAvailability.isGooglePlayServicesAvailable(context) }
        }*/
    }

    @Test
    fun googlePlayServicesAvailable_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val service = LocationService(context)

        whenever(googleApiAvailability.isGooglePlayServicesAvailable(context)).thenReturn(ConnectionResult.SUCCESS)
/*        every { googleApiAvailability.isGooglePlayServicesAvailable(context) }.returns(
            ConnectionResult.SUCCESS
        )*/
        mockStatic(GoogleApiAvailability::class.java).use { mock ->
            mock.`when`<GoogleApiAvailability> {
                GoogleApiAvailability.getInstance()
            }.thenReturn(googleApiAvailability)

            assertTrue(service.googlePlayServicesAvailable)

            mock.verify({
                GoogleApiAvailability.getInstance()
            }, only())
            verify(googleApiAvailability, only()).isGooglePlayServicesAvailable(context)
        }
/*        mockkStatic(GoogleApiAvailability::class) {
            every {
                GoogleApiAvailability.getInstance()
            }.returns(googleApiAvailability)

            assertTrue(service.googlePlayServicesAvailable)

            verify(exactly = 1) { GoogleApiAvailability.getInstance() }
            verify(exactly = 1) { googleApiAvailability.isGooglePlayServicesAvailable(context) }
        }*/
    }

    @Config(sdk = [Build.VERSION_CODES.P])
    @Test
    fun locationEnabled_whenLocationManagerAvailableAndSdkP_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val service = LocationService(context)

        val locationManager: LocationManager? = service.getPrivateProperty("locationManager")
        requireNotNull(locationManager)
        val locationManagerSpy = spy(locationManager)
//        val locationManagerSpy = spyk(locationManager)
        doReturn(true).whenever(locationManagerSpy).isLocationEnabled
//        every { locationManagerSpy.isLocationEnabled }.returns(true)
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
        val locationManagerSpy = spy(locationManager)
//        val locationManagerSpy = spyk(locationManager)
        doReturn(true).whenever(locationManagerSpy).isProviderEnabled(LocationService.FUSED_PROVIDER)
//        every { locationManagerSpy.isProviderEnabled(LocationService.FUSED_PROVIDER) }.returns(true)
        doReturn(location).whenever(locationManagerSpy).getLastKnownLocation(LocationService.FUSED_PROVIDER)
/*        every { locationManagerSpy.getLastKnownLocation(LocationService.FUSED_PROVIDER) }.returns(
            location
        )*/
        service.setPrivateProperty("locationManager", locationManagerSpy)

        assertSame(location, service.getLastKnownLocation())
    }

    @Test
    fun getLastKnownLocation_whenProviderDisabled_returnsNull() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val service = LocationService(context)

        val locationManager: LocationManager? = service.getPrivateProperty("locationManager")
        requireNotNull(locationManager)
        val locationManagerSpy = spy(locationManager)
//        val locationManagerSpy = spyk(locationManager)
        doReturn(false).whenever(locationManagerSpy).isProviderEnabled(LocationService.FUSED_PROVIDER)
//        every { locationManagerSpy.isProviderEnabled(LocationService.FUSED_PROVIDER) }.returns(false)
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
        whenever(googleApiAvailability.isGooglePlayServicesAvailable(context)).thenReturn(ConnectionResult.SUCCESS)
/*        every { googleApiAvailability.isGooglePlayServicesAvailable(context) }.returns(
            ConnectionResult.SUCCESS
        )*/
        mockStatic(GoogleApiAvailability::class.java).use { mock ->
            mock.`when`<GoogleApiAvailability> {
                GoogleApiAvailability.getInstance()
            }.thenReturn(googleApiAvailability)

            val service = LocationService(context)

            val fusedLocationClient: FusedLocationProviderClient? =
                service.getPrivateProperty("fusedLocationClient")
            val cancellationTokenSource: CancellationTokenSource? =
                service.getPrivateProperty("cancellationTokenSource")

            requireNotNull(fusedLocationClient)
            requireNotNull(cancellationTokenSource)

            val fusedLocationClientSpy = spy(fusedLocationClient)
            val cancellationTokenSourceSpy = spy(cancellationTokenSource)
            service.setPrivateProperty("fusedLocationClient", fusedLocationClientSpy)
            service.setPrivateProperty("cancellationTokenSource", cancellationTokenSourceSpy)

            whenever(task.addOnSuccessListener(any())).thenAnswer { invocation ->
                val listener = invocation.getArgument<OnSuccessListener<Location>>(0)
                listener.onSuccess(location)
                return@thenAnswer task
            }
            doReturn(task).whenever(fusedLocationClientSpy).getCurrentLocation(
                eq(LocationRequest.PRIORITY_HIGH_ACCURACY), any()
            )

            doNothing().whenever(currentLocationListener).onCurrentLocation(any())
            service.getCurrentLocation(currentLocationListener)

            verify(fusedLocationClientSpy, only()).getCurrentLocation(
                eq(LocationRequest.PRIORITY_HIGH_ACCURACY),
                any()
            )
            verify(task, only()).addOnSuccessListener(any())
            verify(currentLocationListener, only()).onCurrentLocation(location)
        }
/*        mockkStatic(GoogleApiAvailability::class) {
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

            justRun { currentLocationListener.onCurrentLocation(any()) }
            service.getCurrentLocation(currentLocationListener)

            verify(exactly = 1) {
                fusedLocationClientSpy.getCurrentLocation(
                    LocationRequest.PRIORITY_HIGH_ACCURACY,
                    any()
                )
            }
            verify(exactly = 1) { task.addOnSuccessListener(any()) }
            verify(exactly = 1) { currentLocationListener.onCurrentLocation(location) }
        }*/
    }

    @Config(sdk = [Build.VERSION_CODES.R])
    @Test
    fun getCurrentLocation_whenNoFusedClientAndNoPreviousRequestAndSdkR_callsExpectedMethods() {
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

        val locationManager: LocationManager? = service.getPrivateProperty("locationManager")
        requireNotNull(locationManager)
        val locationManagerSpy = spy(locationManager)
//        val locationManagerSpy = spyk(locationManager)
        service.setPrivateProperty("locationManager", locationManagerSpy)
        doAnswer { invocation ->
            val cancellationSignal = invocation.arguments[1]
            assertNotNull(cancellationSignal)
            val consumer = invocation.getArgument<Consumer<Location>>(3)
            consumer.accept(location)
        }.whenever(locationManagerSpy).getCurrentLocation(
            eq(LocationService.FUSED_PROVIDER),
            any(),
            any(),
            any<Consumer<Location>>()
        )
/*        every {
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
        }*/

        doNothing().whenever(currentLocationListener).onCurrentLocation(any())
//        justRun { currentLocationListener.onCurrentLocation(any()) }
        service.getCurrentLocation(currentLocationListener)

        val cancellationSignal2: CancellationSignal? =
            service.getPrivateProperty("cancellationSignal")
        assertNull(cancellationSignal2)

        verify(locationManagerSpy, only()).getCurrentLocation(
            eq(LocationService.FUSED_PROVIDER),
            any(),
            any(),
            any<Consumer<Location>>()
        )
/*        verify(exactly = 1) {
            locationManagerSpy.getCurrentLocation(
                LocationService.FUSED_PROVIDER,
                any(),
                any(),
                any<Consumer<Location>>()
            )
        }*/
        verify(currentLocationListener, only()).onCurrentLocation(location)
//        verify(exactly = 1) { currentLocationListener.onCurrentLocation(location) }
    }

    @Config(sdk = [Build.VERSION_CODES.R])
    @Test
    fun getCurrentLocation_whenNoFusedClientAndPreviousRequestAndSdkR_callsExpectedMethods() {
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
        service.setPrivateProperty("cancellationSignal", cancellationSignalSpy)

        val locationManager: LocationManager? = service.getPrivateProperty("locationManager")
        requireNotNull(locationManager)
        val locationManagerSpy = spy(locationManager)
//        val locationManagerSpy = spyk(locationManager)
        service.setPrivateProperty("locationManager", locationManagerSpy)
        doAnswer { invocation ->
            val cancellationSignal = invocation.arguments[1]
            assertNotNull(cancellationSignal)
            assertNotSame(cancellationSignalSpy, cancellationSignal)
            val consumer = invocation.getArgument<Consumer<Location>>(3)
            consumer.accept(location)
        }.whenever(locationManagerSpy).getCurrentLocation(
            eq(LocationService.FUSED_PROVIDER),
            any(),
            any(),
            any<Consumer<Location>>()
        )
/*        every {
            locationManagerSpy.getCurrentLocation(
                LocationService.FUSED_PROVIDER,
                any(),
                any(),
                any<Consumer<Location>>()
            )
        }.answers { answer ->
            val cancellationSignal = answer.invocation.args[1]
            assertNotNull(cancellationSignal)
            assertNotSame(cancellationSignalSpy, cancellationSignal)
            @Suppress("UNCHECKED_CAST")
            val consumer = answer.invocation.args[3] as Consumer<Location>
            consumer.accept(location)
        }*/

        doNothing().whenever(currentLocationListener).onCurrentLocation(any())
//        justRun { currentLocationListener.onCurrentLocation(any()) }
        service.getCurrentLocation(currentLocationListener)

        val cancellationSignal3: CancellationSignal? =
            service.getPrivateProperty("cancellationSignal")
        assertNull(cancellationSignal3)

        verify(cancellationSignalSpy, only()).cancel()
//        verify(exactly = 1) { cancellationSignalSpy.cancel() }
        verify(locationManagerSpy, only()).getCurrentLocation(
            eq(LocationService.FUSED_PROVIDER),
            any(),
            any(),
            any<Consumer<Location>>()
        )
/*        verify(exactly = 1) {
            locationManagerSpy.getCurrentLocation(
                LocationService.FUSED_PROVIDER,
                any(),
                any(),
                any<Consumer<Location>>()
            )
        }*/
        verify(currentLocationListener, only()).onCurrentLocation(location)
//        verify(exactly = 1) { currentLocationListener.onCurrentLocation(location) }
    }

    @Config(sdk = [Build.VERSION_CODES.R])
    @Test
    fun getCurrentLocation_whenNoFusedClientNoLocationManagerAndSdkR_callsExpectedMethods() {
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
        service.setPrivateProperty("cancellationSignal", cancellationSignalSpy)

        val locationManager: LocationManager? = service.getPrivateProperty("locationManager")
        assertNotNull(locationManager)
        service.setPrivateProperty("locationManager", null)

        doNothing().whenever(currentLocationListener).onCurrentLocation(any())
//        justRun { currentLocationListener.onCurrentLocation(any()) }
        service.getCurrentLocation(currentLocationListener)

        val cancellationSignal3: CancellationSignal? =
            service.getPrivateProperty("cancellationSignal")
        assertNotNull(cancellationSignal3)
        assertNotSame(cancellationSignalSpy, cancellationSignal3)

        verifyNoInteractions(currentLocationListener)
//        verify { currentLocationListener wasNot Called }
    }

    @Config(sdk = [Build.VERSION_CODES.Q])
    @Test
    fun getCurrentLocation_whenNoFusedClientAndPreviousRequestAndSdkQ_callsExpectedMethods() {
        val service = LocationService(context)

        val fusedLocationClient: FusedLocationProviderClient? =
            service.getPrivateProperty("fusedLocationClient")
        val cancellationTokenSource: CancellationTokenSource? =
            service.getPrivateProperty("cancellationTokenSource")

        assertNull(fusedLocationClient)
        assertNull(cancellationTokenSource)

        val locationManager: LocationManager? = service.getPrivateProperty("locationManager")
        requireNotNull(locationManager)
        val locationManagerSpy = spy(locationManager)
//        val locationManagerSpy = spyk(locationManager)
        service.setPrivateProperty("locationManager", locationManagerSpy)
        doAnswer { invocation ->
            val listener = invocation.getArgument<LocationListener>(1)
            listener.onLocationChanged(location)
        }.whenever(locationManagerSpy).requestSingleUpdate(eq(LocationService.FUSED_PROVIDER), any(), any())
/*        every {
            @Suppress("DEPRECATION")
            locationManagerSpy.requestSingleUpdate(LocationService.FUSED_PROVIDER, any(), any())
        }.answers { answer ->
            val listener = answer.invocation.args[1] as LocationListener
            listener.onLocationChanged(location)
        }*/

        doNothing().whenever(currentLocationListener).onCurrentLocation(any())
//        justRun { currentLocationListener.onCurrentLocation(any()) }
        service.getCurrentLocation(currentLocationListener)

        verify(locationManagerSpy, only()).requestSingleUpdate(eq(LocationService.FUSED_PROVIDER), any(), any())
/*        verify(exactly = 1) {
            @Suppress("DEPRECATION")
            locationManagerSpy.requestSingleUpdate(LocationService.FUSED_PROVIDER, any(), any())
        }*/
        verify(currentLocationListener, only()).onCurrentLocation(location)
//        verify(exactly = 1) { currentLocationListener.onCurrentLocation(location) }
    }

    @Test
    fun getCurrentLocation_whenNoLocationManagerAndSdkQ_makesNoAction() {
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

        doNothing().whenever(currentLocationListener).onCurrentLocation(any())
//        justRun { currentLocationListener.onCurrentLocation(any()) }
        service.getCurrentLocation(currentLocationListener)

        verifyNoInteractions(currentLocationListener)
//        verify { currentLocationListener wasNot Called }
    }

    @Config(sdk = [Build.VERSION_CODES.R])
    @Test
    fun getCurrentLocation_whenFusedClientAndNoCancellationTokenSource() {
        whenever(googleApiAvailability.isGooglePlayServicesAvailable(context)).thenReturn(ConnectionResult.SUCCESS)
/*        every { googleApiAvailability.isGooglePlayServicesAvailable(context) }.returns(
            ConnectionResult.SUCCESS
        )*/
        mockStatic(GoogleApiAvailability::class.java).use { mock ->
            mock.`when`<GoogleApiAvailability> {
                GoogleApiAvailability.getInstance()
            }.thenReturn(googleApiAvailability)

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

            val locationManager: LocationManager? = service.getPrivateProperty("locationManager")
            requireNotNull(locationManager)
            val locationManagerSpy = spy(locationManager)
            service.setPrivateProperty("locationManager", locationManagerSpy)
            doAnswer { invocation ->
                val cancellationSignal = invocation.arguments[1]
                assertNotNull(cancellationSignal)
                val consumer = invocation.getArgument<Consumer<Location>>(3)
                consumer.accept(location)
            }.whenever(locationManagerSpy).getCurrentLocation(
                eq(LocationService.FUSED_PROVIDER),
                any(),
                any(),
                any<Consumer<Location>>()
            )

            doNothing().whenever(currentLocationListener).onCurrentLocation(any())
            service.getCurrentLocation(currentLocationListener)

            val cancellationSignal2: CancellationSignal? =
                service.getPrivateProperty("cancellationSignal")
            assertNull(cancellationSignal2)

            verify(locationManagerSpy, only()).getCurrentLocation(
                eq(LocationService.FUSED_PROVIDER),
                any(),
                any(),
                any<Consumer<Location>>()
            )
            verify(currentLocationListener, only()).onCurrentLocation(location)
        }
/*        mockkStatic(GoogleApiAvailability::class) {
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

            justRun { currentLocationListener.onCurrentLocation(any()) }
            service.getCurrentLocation(currentLocationListener)

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
            verify(exactly = 1) { currentLocationListener.onCurrentLocation(location) }
        }*/
    }

    @Config(sdk = [Build.VERSION_CODES.Q])
    @Test
    fun getCurrentLocation_whenNoLocationManagerSdkQ_makesNoAction() {
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

        service.getCurrentLocation(currentLocationListener)

        verifyNoInteractions(currentLocationListener)
//        verify { currentLocationListener wasNot Called }
    }

    @Test
    fun cancelCurrentLocation_whenNoPreviousTokenOrSignal_makesNoAction() {
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

        doNothing().whenever(cancellationTokenSource).cancel()
//        justRun { cancellationTokenSource.cancel() }
        service.setPrivateProperty("cancellationTokenSource", cancellationTokenSource)

        service.cancelCurrentLocation()

        verify(cancellationTokenSource, only()).cancel()
//        verify(exactly = 1) { cancellationTokenSource.cancel() }
    }

    @Test
    fun cancelCurrentLocation_whenCancellationSignal_callsCancelMethod() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val service = LocationService(context)

        doNothing().whenever(cancellationSignal).cancel()
//        justRun { cancellationSignal.cancel() }
        service.setPrivateProperty("cancellationSignal", cancellationSignal)

        service.cancelCurrentLocation()

        verify(cancellationSignal, only()).cancel()
//        verify(exactly = 1) { cancellationSignal.cancel() }
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
        service.locationUpdateListener = locationUpdateListener

        // check
        assertSame(locationUpdateListener, service.locationUpdateListener)
    }

    @Test
    fun requestLocationUpdates_whenFusedLocationClientAvailableAndNoListener_callsExpectedMethods() {
        val context = ApplicationProvider.getApplicationContext<Context>()

        whenever(googleApiAvailability.isGooglePlayServicesAvailable(context)).thenReturn(ConnectionResult.SUCCESS)
/*        every { googleApiAvailability.isGooglePlayServicesAvailable(context) }.returns(
            ConnectionResult.SUCCESS
        )*/
        mockStatic(GoogleApiAvailability::class.java).use { mock ->
            mock.`when`<GoogleApiAvailability> {
                GoogleApiAvailability.getInstance()
            }.thenReturn(googleApiAvailability)

            val service = LocationService(context)

            val fusedLocationClient: FusedLocationProviderClient? =
                service.getPrivateProperty("fusedLocationClient")
            val cancellationTokenSource: CancellationTokenSource? =
                service.getPrivateProperty("cancellationTokenSource")

            requireNotNull(fusedLocationClient)
            requireNotNull(cancellationTokenSource)

            val fusedLocationClientSpy = spy(fusedLocationClient)
            service.setPrivateProperty("fusedLocationClient", fusedLocationClientSpy)

            doReturn(requestTask).whenever(fusedLocationClientSpy).requestLocationUpdates(
                any(),
                any<LocationCallback>(),
                any()
            )

            service.requestLocationUpdates()

            // check
            verify(fusedLocationClientSpy, only()).requestLocationUpdates(
                capture(locationRequestCaptor),
                capture(locationCallbackCaptor),
                any()
            )

            val locationRequest = locationRequestCaptor.value
            assertEquals(service.smallestDisplacement, locationRequest.smallestDisplacement)
            assertEquals(service.updateInterval, locationRequest.interval)

            // execute callback methods
            val callback = locationCallbackCaptor.value
            callback.onLocationAvailability(locationAvailability)

            whenever(locationResult.lastLocation).thenReturn(null)
            callback.onLocationResult(locationResult)
        }

        /*mockkStatic(GoogleApiAvailability::class) {
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

            every {
                fusedLocationClientSpy.requestLocationUpdates(
                    any(),
                    any<LocationCallback>(),
                    any()
                )
            }.returns(
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
            callback.onLocationAvailability(locationAvailability)

            every { locationResult.lastLocation }.returns(null)
            callback.onLocationResult(locationResult)
        }*/
    }

    @Test
    fun requestLocationUpdates_whenFusedLocationClientAvailableAndListener_callsExpectedMethods() {
        val context = ApplicationProvider.getApplicationContext<Context>()

        whenever(googleApiAvailability.isGooglePlayServicesAvailable(context)).thenReturn(ConnectionResult.SUCCESS)
/*        every { googleApiAvailability.isGooglePlayServicesAvailable(context) }.returns(
            ConnectionResult.SUCCESS
        )*/
        mockStatic(GoogleApiAvailability::class.java).use { mock ->
            mock.`when`<GoogleApiAvailability> {
                GoogleApiAvailability.getInstance()
            }.thenReturn(googleApiAvailability)

            val service = LocationService(context)
            doNothing().whenever(locationUpdateListener).onLocationChanged(any())
            doNothing().whenever(locationUpdateListener).onLocationAvailability(any())
            service.locationUpdateListener = locationUpdateListener

            val fusedLocationClient: FusedLocationProviderClient? =
                service.getPrivateProperty("fusedLocationClient")
            val cancellationTokenSource: CancellationTokenSource? =
                service.getPrivateProperty("cancellationTokenSource")

            requireNotNull(fusedLocationClient)
            requireNotNull(cancellationTokenSource)

            val fusedLocationClientSpy = spy(fusedLocationClient)
            service.setPrivateProperty("fusedLocationClient", fusedLocationClientSpy)

            doReturn(requestTask).whenever(fusedLocationClientSpy).requestLocationUpdates(
                any(),
                any<LocationCallback>(),
                any()
            )

            service.requestLocationUpdates()

            // check
            verify(fusedLocationClientSpy, only()).requestLocationUpdates(
                capture(locationRequestCaptor),
                capture(locationCallbackCaptor),
                any()
            )

            val locationRequest = locationRequestCaptor.value
            assertEquals(service.smallestDisplacement, locationRequest.smallestDisplacement)
            assertEquals(service.updateInterval, locationRequest.interval)

            // execute callback methods
            val callback = locationCallbackCaptor.value
            whenever(locationAvailability.isLocationAvailable).thenReturn(true)
            callback.onLocationAvailability(locationAvailability)

            whenever(locationResult.lastLocation).thenReturn(location)
            callback.onLocationResult(locationResult)

            verify(locationUpdateListener, times(1)).onLocationChanged(location)
            verify(locationUpdateListener, times(1)).onLocationAvailability(true)
        }

        /*mockkStatic(GoogleApiAvailability::class) {
            every {
                GoogleApiAvailability.getInstance()
            }.returns(googleApiAvailability)

            val service = LocationService(context)
            justRun { locationUpdateListener.onLocationChanged(any()) }
            justRun { locationUpdateListener.onLocationAvailability(any()) }
            service.locationUpdateListener = locationUpdateListener

            val fusedLocationClient: FusedLocationProviderClient? =
                service.getPrivateProperty("fusedLocationClient")
            val cancellationTokenSource: CancellationTokenSource? =
                service.getPrivateProperty("cancellationTokenSource")

            requireNotNull(fusedLocationClient)
            requireNotNull(cancellationTokenSource)

            val fusedLocationClientSpy = spyk(fusedLocationClient)
            service.setPrivateProperty("fusedLocationClient", fusedLocationClientSpy)

            every {
                fusedLocationClientSpy.requestLocationUpdates(
                    any(),
                    any<LocationCallback>(),
                    any()
                )
            }.returns(requestTask)

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
            every { locationAvailability.isLocationAvailable }.returns(true)
            callback.onLocationAvailability(locationAvailability)

            every { locationResult.lastLocation }.returns(location)
            callback.onLocationResult(locationResult)

            verify(exactly = 1) { locationUpdateListener.onLocationChanged(location) }
            verify(exactly = 1) { locationUpdateListener.onLocationAvailability(true) }
        }*/
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
        val locationManagerSpy = spy(locationManager)
//        val locationManagerSpy = spyk(locationManager)
        doNothing().whenever(locationManagerSpy).requestLocationUpdates(
            eq("fused"),
            eq(service.updateInterval),
            eq(service.smallestDisplacement),
            any<LocationListener>(),
            any()
        )
/*        justRun {
            locationManagerSpy.requestLocationUpdates(
                "fused",
                service.updateInterval,
                service.smallestDisplacement,
                any<LocationListener>(),
                any()
            )
        }*/
        service.setPrivateProperty("locationManager", locationManagerSpy)

        service.requestLocationUpdates()

        // check
        verify(locationManagerSpy, times(1)).removeUpdates(any<LocationListener>())
//        verify(exactly = 1) { locationManagerSpy.removeUpdates(any<LocationListener>()) }
        verify(locationManagerSpy, times(1)).requestLocationUpdates(
            eq(LocationService.FUSED_PROVIDER),
            eq(service.updateInterval),
            eq(service.smallestDisplacement),
            any<LocationListener>(),
            any()
        )
/*        verify(exactly = 1) {
            locationManagerSpy.requestLocationUpdates(
                LocationService.FUSED_PROVIDER,
                service.updateInterval,
                service.smallestDisplacement,
                any<LocationListener>(),
                any()
            )
        }*/
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

        whenever(googleApiAvailability.isGooglePlayServicesAvailable(context)).thenReturn(ConnectionResult.SUCCESS)
/*        every { googleApiAvailability.isGooglePlayServicesAvailable(context) }.returns(
            ConnectionResult.SUCCESS
        )*/
        mockStatic(GoogleApiAvailability::class.java).use { mock ->
            mock.`when`<GoogleApiAvailability> {
                GoogleApiAvailability.getInstance()
            }.thenReturn(googleApiAvailability)

            val service = LocationService(context)

            val fusedLocationClient: FusedLocationProviderClient? =
                service.getPrivateProperty("fusedLocationClient")
            requireNotNull(fusedLocationClient)
            val fusedLocationClientSpy = spy(fusedLocationClient)
            service.setPrivateProperty("fusedLocationClient", fusedLocationClientSpy)

            val locationManager: LocationManager? = service.getPrivateProperty("locationManager")
            requireNotNull(locationManager)
            val locationManagerSpy = spy(locationManager)
            service.setPrivateProperty("locationManager", locationManagerSpy)

            service.cancelLocationUpdates()

            // check
            verify(fusedLocationClientSpy, times(1)).removeLocationUpdates(any<LocationCallback>())
            verify(locationManagerSpy, times(1)).removeUpdates(any<LocationListener>())
        }

        /*mockkStatic(GoogleApiAvailability::class) {
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
        }*/
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
        val locationManagerSpy = spy(locationManager)
//        val locationManagerSpy = spyk(locationManager)
        service.setPrivateProperty("locationManager", locationManagerSpy)

        service.cancelLocationUpdates()

        // check
        verify(locationManagerSpy, times(1)).removeUpdates(any<LocationListener>())
//        verify(exactly = 1) { locationManagerSpy.removeUpdates(any<LocationListener>()) }
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
        assertEquals(6, LocationService.GooglePlayStatus.entries.size)
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

        updatesListener.onLocationChanged(location)
    }

    @Test
    fun updatesListener_whenListener_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()

        val service = LocationService(context)

        val updatesListener: LocationListener? = service.getPrivateProperty("updatesListener")
        requireNotNull(updatesListener)

        assertNull(service.locationUpdateListener)

        doNothing().whenever(locationUpdateListener).onLocationChanged(any())
//        justRun { locationUpdateListener.onLocationChanged(any()) }
        service.locationUpdateListener = locationUpdateListener

        updatesListener.onLocationChanged(location)

        verify(locationUpdateListener, only()).onLocationChanged(location)
//        verify(exactly = 1) { locationUpdateListener.onLocationChanged(location) }
    }

    @Test
    fun getLooper_whenNoLooperOnCurrentThread_getsMainLooper() {
        mockStatic(Looper::class.java).use { mock ->
            mock.`when`<Looper> {
                Looper.myLooper()
            }.thenReturn(null)
            mock.`when`<Looper> {
                Looper.getMainLooper()
            }.thenReturn(looper)

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
            val locationManagerSpy = spy(locationManager)
            service.setPrivateProperty("locationManager", locationManagerSpy)
            doNothing().whenever(locationManagerSpy).requestLocationUpdates(
                eq("fused"),
                eq(service.updateInterval),
                eq(service.smallestDisplacement),
                any<LocationListener>(),
                any()
            )
            service.setPrivateProperty("locationManager", locationManagerSpy)

            service.requestLocationUpdates()

            mock.verify( {
                Looper.myLooper()
            }, times(1))
            mock.verify( {
                Looper.getMainLooper()
            }, times(1))
        }

        /*        mockkStatic(Looper::class) {
                    every { Looper.myLooper() }.returns(null)
                    every { Looper.getMainLooper() }.returns(looper)

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
                    justRun {
                        locationManagerSpy.requestLocationUpdates(
                            "fused",
                            service.updateInterval,
                            service.smallestDisplacement,
                            any<LocationListener>(),
                            any()
                        )
                    }

                    service.requestLocationUpdates()

                    verify(exactly = 1) { Looper.myLooper() }
                    verify(exactly = 1) { Looper.getMainLooper() }
                }*/
    }

    @Test
    fun getLooper_whenLooperOnCurrentThread_getsMainLooper() {
        mockStatic(Looper::class.java).use { mock ->
            mock.`when`<Looper> {
                Looper.myLooper()
            }.thenReturn(looper)

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
            val locationManagerSpy = spy(locationManager)
            doNothing().whenever(locationManagerSpy).requestLocationUpdates(
                eq("fused"),
                eq(service.updateInterval),
                eq(service.smallestDisplacement),
                any<LocationListener>(),
                any()
            )
            service.setPrivateProperty("locationManager", locationManagerSpy)

            service.requestLocationUpdates()

            mock.verify( {
                Looper.myLooper()
            }, times(1))
            mock.verify( {
                Looper.getMainLooper()
            }, never())
        }

/*        mockkStatic(Looper::class) {
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
            justRun {
                locationManagerSpy.requestLocationUpdates(
                    "fused",
                    service.updateInterval,
                    service.smallestDisplacement,
                    any<LocationListener>(),
                    any()
                )
            }
            service.setPrivateProperty("locationManager", locationManagerSpy)

            service.requestLocationUpdates()

            verify(exactly = 1) { Looper.myLooper() }
            verify(exactly = 0) { Looper.getMainLooper() }
        }*/
    }
}