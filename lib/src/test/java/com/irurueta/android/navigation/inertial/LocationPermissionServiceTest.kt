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

import android.Manifest
import android.annotation.SuppressLint
import android.content.Context
import android.content.pm.PackageManager
import android.os.Build
import androidx.activity.result.ActivityResultCallback
import androidx.activity.result.ActivityResultLauncher
import androidx.activity.result.contract.ActivityResultContracts
import androidx.appcompat.app.AppCompatActivity
import androidx.core.app.ActivityCompat
import androidx.test.core.app.ApplicationProvider
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
//import org.mockito.kotlin.mock
import org.mockito.kotlin.only
import org.mockito.kotlin.times
import org.mockito.kotlin.verify
import org.mockito.kotlin.whenever
import org.robolectric.RobolectricTestRunner
import org.robolectric.annotation.Config

//@Ignore("Possible memory leak when running this test")
@Suppress("UseCheckPermission")
@RunWith(RobolectricTestRunner::class)
class LocationPermissionServiceTest {

    @get:Rule
    val mockitoRule: MockitoRule = MockitoJUnit.rule()

//    @get:Rule
//    val mockkRule = MockKRule(this)

//    @MockK
    @Mock
    private lateinit var listener:
            LocationPermissionService.OnLocationPermissionRequestResultListener

//    @MockK
    @Mock
    private lateinit var launcher: ActivityResultLauncher<Array<String>>

//    @SpyK
    @Spy
    private var context: Context = ApplicationProvider.getApplicationContext()

//    @SpyK
    @Spy
    private var activity = AppCompatActivity()

    @Captor
    private lateinit var locationPermissionResultCaptor: ArgumentCaptor<LocationPermissionService.LocationPermissionResult>

    /*@After
    fun tearDown() {
        unmockkAll()
        clearAllMocks()
        System.gc()
    }*/

    @Test
    fun constructor_whenActivity_setsBothContextAndActivity() {
        val activity = AppCompatActivity()
        val service = LocationPermissionService(activity)

        // check
        assertSame(activity, service.context)
        assertSame(activity, service.activity)
        assertNull(service.onLocationPermissionRequestResultListener)
    }

    @Test
    fun constructor_whenActivityAndListener_setsContextActivityAndListener() {
        val activity = AppCompatActivity()
        val service = LocationPermissionService(activity, listener)

        // check
        assertSame(activity, service.context)
        assertSame(activity, service.activity)
        assertSame(listener, service.onLocationPermissionRequestResultListener)
    }

    @Test
    fun constructor_whenContext_setsContextOnly() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val service = LocationPermissionService(context)

        // check
        assertSame(context, service.context)
        assertNull(service.activity)
        assertNull(service.onLocationPermissionRequestResultListener)
    }

    @Test
    fun constructor_whenContextAndListener_setsContextAndListener() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val service = LocationPermissionService(context, listener)

        // check
        assertSame(context, service.context)
        assertNull(service.activity)
        assertSame(listener, service.onLocationPermissionRequestResultListener)
    }

    @Test
    fun onLocationPermissionRequestResultListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val service = LocationPermissionService(context)

        // check default value
        assertNull(service.onLocationPermissionRequestResultListener)

        // set new value
        service.onLocationPermissionRequestResultListener = listener

        // check
        assertSame(listener, service.onLocationPermissionRequestResultListener)
    }

    @SuppressLint("UseCheckPermission")
    @Test
    fun hasFineLocationPermission_whenPermissionGranted_callsExpectedMethod() {
        doReturn(PackageManager.PERMISSION_GRANTED).whenever(context).checkPermission(
            eq(Manifest.permission.ACCESS_FINE_LOCATION),
            any(),
            any()
        )
/*        every {
            context.checkPermission(
                Manifest.permission.ACCESS_FINE_LOCATION,
                any(),
                any()
            )
        }.returns(PackageManager.PERMISSION_GRANTED)*/
        val service = LocationPermissionService(context)

        assertTrue(service.hasFineLocationPermission())
        verify(context, only()).checkPermission(
            eq(Manifest.permission.ACCESS_FINE_LOCATION),
            any(),
            any()
        )
/*        verify(exactly = 1) {
            context.checkPermission(
                Manifest.permission.ACCESS_FINE_LOCATION,
                any(),
                any()
            )
        }*/
    }

    @SuppressLint("UseCheckPermission")
    @Test
    fun hasFineLocationPermission_whenPermissionDenied_callsExpectedMethod() {
        doReturn(PackageManager.PERMISSION_DENIED).whenever(context).checkPermission(
            eq(Manifest.permission.ACCESS_FINE_LOCATION),
            any(),
            any()
        )
/*        every {
            context.checkPermission(
                Manifest.permission.ACCESS_FINE_LOCATION,
                any(),
                any()
            )
        }.returns(PackageManager.PERMISSION_DENIED)*/
        val service = LocationPermissionService(context)

        assertFalse(service.hasFineLocationPermission())
        verify(context, times(1)).checkPermission(
            eq(Manifest.permission.ACCESS_FINE_LOCATION),
            any(),
            any()
        )
/*        verify(exactly = 1) {
            context.checkPermission(
                Manifest.permission.ACCESS_FINE_LOCATION,
                any(),
                any()
            )
        }*/
    }

    @Test
    fun hasCoarseLocationPermission_whenPermissionGranted_callsExpectedMethod() {
        doReturn(PackageManager.PERMISSION_GRANTED).whenever(context).checkPermission(
            eq(Manifest.permission.ACCESS_COARSE_LOCATION),
            any(),
            any()
        )
/*        every {
            context.checkPermission(
                Manifest.permission.ACCESS_COARSE_LOCATION,
                any(),
                any()
            )
        }.returns(PackageManager.PERMISSION_GRANTED)*/
        val service = LocationPermissionService(context)

        assertTrue(service.hasCoarseLocationPermission())
        verify(context, only()).checkPermission(
            eq(Manifest.permission.ACCESS_COARSE_LOCATION),
            any(),
            any()
        )
/*        verify(exactly = 1) {
            context.checkPermission(
                Manifest.permission.ACCESS_COARSE_LOCATION,
                any(),
                any()
            )
        }*/
    }

    @Test
    fun hasCoarseLocationPermission_whenPermissionDenied_callsExpectedMethod() {
        doReturn(PackageManager.PERMISSION_DENIED).whenever(context).checkPermission(
            eq(Manifest.permission.ACCESS_COARSE_LOCATION),
            any(),
            any()
        )
/*        every {
            context.checkPermission(
                Manifest.permission.ACCESS_COARSE_LOCATION,
                any(),
                any()
            )
        }.returns(PackageManager.PERMISSION_DENIED)*/
        val service = LocationPermissionService(context)

        assertFalse(service.hasCoarseLocationPermission())
        verify(context, only()).checkPermission(
            eq(Manifest.permission.ACCESS_COARSE_LOCATION),
            any(),
            any()
        )
/*        verify(exactly = 1) {
            context.checkPermission(
                Manifest.permission.ACCESS_COARSE_LOCATION,
                any(),
                any()
            )
        }*/
    }

    @Config(sdk = [Build.VERSION_CODES.P])
    @Test
    fun hasBackgroundLocationPermission_whenSdkPAndFinePermissionGranted_callsExpectedMethod() {
        doReturn(PackageManager.PERMISSION_GRANTED).whenever(context).checkPermission(
            eq(Manifest.permission.ACCESS_FINE_LOCATION),
            any(),
            any()
        )
/*        every {
            context.checkPermission(
                Manifest.permission.ACCESS_FINE_LOCATION,
                any(),
                any()
            )
        }.returns(PackageManager.PERMISSION_GRANTED)*/

        val service = LocationPermissionService(context)

        assertTrue(service.hasBackgroundLocationPermission())
        verify(context, times(1)).checkPermission(
            eq(Manifest.permission.ACCESS_FINE_LOCATION),
            any(),
            any()
        )
/*        verify(exactly = 1) {
            context.checkPermission(
                Manifest.permission.ACCESS_FINE_LOCATION,
                any(),
                any()
            )
        }*/
    }

    @Config(sdk = [Build.VERSION_CODES.P])
    @Test
    fun hasBackgroundLocationPermission_whenSdkPAndCoarsePermissionGranted_callsExpectedMethod() {
        doReturn(PackageManager.PERMISSION_DENIED).whenever(context).checkPermission(
            eq(Manifest.permission.ACCESS_FINE_LOCATION),
            any(),
            any()
        )
/*        every {
            context.checkPermission(
                Manifest.permission.ACCESS_FINE_LOCATION,
                any(),
                any()
            )
        }.returns(PackageManager.PERMISSION_DENIED)*/
        doReturn(PackageManager.PERMISSION_GRANTED).whenever(context).checkPermission(
            eq(Manifest.permission.ACCESS_COARSE_LOCATION),
            any(),
            any()
        )
/*        every {
            context.checkPermission(
                Manifest.permission.ACCESS_COARSE_LOCATION,
                any(),
                any()
            )
        }.returns(PackageManager.PERMISSION_GRANTED)*/

        val service = LocationPermissionService(context)

        assertTrue(service.hasBackgroundLocationPermission())
        verify(context, times(1)).checkPermission(
            eq(Manifest.permission.ACCESS_FINE_LOCATION),
            any(),
            any()
        )
/*        verify(exactly = 1) {
            context.checkPermission(
                Manifest.permission.ACCESS_FINE_LOCATION,
                any(),
                any()
            )
        }*/
        verify(context, times(1)).checkPermission(
            eq(Manifest.permission.ACCESS_COARSE_LOCATION),
            any(),
            any()
        )
/*        verify(exactly = 1) {
            context.checkPermission(
                Manifest.permission.ACCESS_COARSE_LOCATION,
                any(),
                any()
            )
        }*/
    }

    @Config(sdk = [Build.VERSION_CODES.P])
    @Test
    fun hasBackgroundLocationPermission_whenSdkPAndPermissionDenied_callsExpectedMethod() {
        doReturn(PackageManager.PERMISSION_DENIED).whenever(context).checkPermission(
            eq(Manifest.permission.ACCESS_FINE_LOCATION),
            any(),
            any()
        )
/*        every {
            context.checkPermission(
                Manifest.permission.ACCESS_FINE_LOCATION,
                any(),
                any()
            )
        }.returns(PackageManager.PERMISSION_DENIED)*/
        doReturn(PackageManager.PERMISSION_DENIED).whenever(context).checkPermission(
            eq(Manifest.permission.ACCESS_COARSE_LOCATION),
            any(),
            any()
        )
/*        every {
            context.checkPermission(
                Manifest.permission.ACCESS_COARSE_LOCATION,
                any(),
                any()
            )
        }.returns(PackageManager.PERMISSION_DENIED)*/

        val service = LocationPermissionService(context)

        assertFalse(service.hasBackgroundLocationPermission())
        verify(context, times(1)).checkPermission(
            eq(Manifest.permission.ACCESS_FINE_LOCATION),
            any(),
            any()
        )
/*        verify(exactly = 1) {
            context.checkPermission(
                Manifest.permission.ACCESS_FINE_LOCATION,
                any(),
                any()
            )
        }*/
        verify(context, times(1)).checkPermission(
            eq(Manifest.permission.ACCESS_COARSE_LOCATION),
            any(),
            any()
        )
/*        verify(exactly = 1) {
            context.checkPermission(
                Manifest.permission.ACCESS_COARSE_LOCATION,
                any(),
                any()
            )
        }*/
    }

    @Config(sdk = [Build.VERSION_CODES.Q])
    @Test
    fun hasBackgroundLocationPermission_whenSdkQAndPermissionGranted_callsExpectedMethod() {
        doReturn(PackageManager.PERMISSION_GRANTED).whenever(context).checkPermission(
            eq(Manifest.permission.ACCESS_BACKGROUND_LOCATION),
            any(),
            any()
        )
/*        every {
            context.checkPermission(
                Manifest.permission.ACCESS_BACKGROUND_LOCATION,
                any(),
                any()
            )
        }.returns(PackageManager.PERMISSION_GRANTED)*/
        val service = LocationPermissionService(context)

        assertTrue(service.hasBackgroundLocationPermission())
        verify(context, only()).checkPermission(
            eq(Manifest.permission.ACCESS_BACKGROUND_LOCATION),
            any(),
            any()
        )
/*        verify(exactly = 1) {
            context.checkPermission(
                Manifest.permission.ACCESS_BACKGROUND_LOCATION,
                any(),
                any()
            )
        }*/
    }

    @Config(sdk = [Build.VERSION_CODES.Q])
    @Test
    fun hasBackgroundLocationPermission_whenSdkQAndPermissionDenied_callsExpectedMethod() {
        doReturn(PackageManager.PERMISSION_DENIED).whenever(context).checkPermission(
            eq(Manifest.permission.ACCESS_BACKGROUND_LOCATION),
            any(),
            any()
        )
/*        every {
            context.checkPermission(
                Manifest.permission.ACCESS_BACKGROUND_LOCATION,
                any(),
                any()
            )
        }.returns(PackageManager.PERMISSION_DENIED)*/
        val service = LocationPermissionService(context)

        assertFalse(service.hasBackgroundLocationPermission())
        verify(context, only()).checkPermission(
            eq(Manifest.permission.ACCESS_BACKGROUND_LOCATION),
            any(),
            any()
        )
/*        verify(exactly = 1) {
            context.checkPermission(
                Manifest.permission.ACCESS_BACKGROUND_LOCATION,
                any(),
                any()
            )
        }*/
    }

    @Test(expected = IllegalStateException::class)
    fun shouldShowRequestFineLocationPermissionRationale_whenNoActivity_throwsIllegalStateException() {
        val service = LocationPermissionService(context)

        service.shouldShowRequestFineLocationPermissionRationale()
    }

    @Test
    fun shouldShowRequestFineLocationPermissionRationale_whenActivity_callsExpectedMethod() {
        val activity = AppCompatActivity()

        mockStatic(ActivityCompat::class.java).use { mock ->
            mock.`when`<Boolean> {
                ActivityCompat.shouldShowRequestPermissionRationale(
                    activity,
                    Manifest.permission.ACCESS_FINE_LOCATION
                )
            }.thenReturn(true)

            val service = LocationPermissionService(activity)
            assertTrue(service.shouldShowRequestFineLocationPermissionRationale())

            mock.verify({
                ActivityCompat.shouldShowRequestPermissionRationale(
                    activity,
                    Manifest.permission.ACCESS_FINE_LOCATION
                )
            }, only())
        }
/*        mockkStatic(ActivityCompat::class) {
            every {
                ActivityCompat.shouldShowRequestPermissionRationale(
                    activity,
                    Manifest.permission.ACCESS_FINE_LOCATION
                )
            }.returns(true)

            val service = LocationPermissionService(activity)
            assertTrue(service.shouldShowRequestFineLocationPermissionRationale())
            verify(exactly = 1) {
                ActivityCompat.shouldShowRequestPermissionRationale(
                    activity,
                    Manifest.permission.ACCESS_FINE_LOCATION
                )
            }
        }*/
    }

    @Test(expected = IllegalStateException::class)
    fun shouldShowRequestCoarseLocationPermissionRationale_whenNoActivity_throwsIllegalStateException() {
        val service = LocationPermissionService(context)

        service.shouldShowRequestCoarseLocationPermissionRationale()
    }

    @Test
    fun shouldShowRequestCoarseLocationPermissionRationale_whenActivity_callsExpectedMethod() {
        val activity = AppCompatActivity()

        mockStatic(ActivityCompat::class.java).use { mock ->
            mock.`when`<Boolean> {
                ActivityCompat.shouldShowRequestPermissionRationale(
                    activity,
                    Manifest.permission.ACCESS_COARSE_LOCATION
                )
            }.thenReturn(true)

            val service = LocationPermissionService(activity)
            assertTrue(service.shouldShowRequestCoarseLocationPermissionRationale())

            mock.verify({
                ActivityCompat.shouldShowRequestPermissionRationale(
                    activity,
                    Manifest.permission.ACCESS_COARSE_LOCATION
                )
            }, only())
        }
/*        mockkStatic(ActivityCompat::class) {
            every {
                ActivityCompat.shouldShowRequestPermissionRationale(
                    activity,
                    Manifest.permission.ACCESS_COARSE_LOCATION
                )
            }.returns(true)

            val service = LocationPermissionService(activity)
            assertTrue(service.shouldShowRequestCoarseLocationPermissionRationale())
            verify(exactly = 1) {
                ActivityCompat.shouldShowRequestPermissionRationale(
                    activity,
                    Manifest.permission.ACCESS_COARSE_LOCATION
                )
            }
        }*/
    }

    @Config(sdk = [Build.VERSION_CODES.Q])
    @Test(expected = IllegalStateException::class)
    fun shouldShowRequestBackgroundLocationPermissionRationale_whenNoActivityAndSdkQ_throwsIllegalStateException() {
        val service = LocationPermissionService(context)

        service.shouldShowRequestBackgroundLocationPermissionRationale()
    }

    @Config(sdk = [Build.VERSION_CODES.Q])
    @Test
    fun shouldShowRequestBackgroundLocationPermissionRationale_whenActivityAndSdkQ_callsExpectedMethod() {
        val activity = AppCompatActivity()

        mockStatic(ActivityCompat::class.java).use { mock ->
            mock.`when`<Boolean> {
                ActivityCompat.shouldShowRequestPermissionRationale(
                    activity,
                    Manifest.permission.ACCESS_BACKGROUND_LOCATION
                )
            }.thenReturn(true)

            val service = LocationPermissionService(activity)
            assertTrue(service.shouldShowRequestBackgroundLocationPermissionRationale())

            mock.verify({
                ActivityCompat.shouldShowRequestPermissionRationale(
                    activity,
                    Manifest.permission.ACCESS_BACKGROUND_LOCATION
                )
            }, only())
        }
/*        mockkStatic(ActivityCompat::class) {
            every {
                ActivityCompat.shouldShowRequestPermissionRationale(
                    activity,
                    Manifest.permission.ACCESS_BACKGROUND_LOCATION
                )
            }.returns(true)

            val service = LocationPermissionService(activity)
            assertTrue(service.shouldShowRequestBackgroundLocationPermissionRationale())
            verify(exactly = 1) {
                ActivityCompat.shouldShowRequestPermissionRationale(
                    activity,
                    Manifest.permission.ACCESS_BACKGROUND_LOCATION
                )
            }
        }*/
    }

    @SuppressLint("UseCheckPermission")
    @Config(sdk = [Build.VERSION_CODES.P])
    @Test
    fun shouldShowRequestBackgroundLocationPermissionRationale_whenNoActivityAndSdkR_callsExpectedMethod() {
        doReturn(PackageManager.PERMISSION_GRANTED).whenever(context).checkPermission(
            any(),
            any(),
            any()
        )
/*        every {
            context.checkPermission(
                any(),
                any(),
                any()
            )
        }.returns(PackageManager.PERMISSION_GRANTED)*/

        val service = LocationPermissionService(context)
        assertFalse(service.shouldShowRequestBackgroundLocationPermissionRationale())
        verify(context, only()).checkPermission(
            eq(Manifest.permission.ACCESS_FINE_LOCATION),
            any(),
            any()
        )
/*        verify(exactly = 1) {
            context.checkPermission(
                Manifest.permission.ACCESS_FINE_LOCATION,
                any(),
                any()
            )
        }*/
    }

    @Config(sdk = [Build.VERSION_CODES.P])
    @Test
    fun shouldShowRequestBackgroundLocationPermissionRationale_whenActivityAndSdkR_callsExpectedMethod() {
        doReturn(PackageManager.PERMISSION_DENIED).whenever(activity).checkPermission(
            any(),
            any(),
            any()
        )
/*        every {
            activity.checkPermission(
                any(),
                any(),
                any()
            )
        }.returns(PackageManager.PERMISSION_DENIED)*/

        val service = LocationPermissionService(activity)
        assertTrue(service.shouldShowRequestBackgroundLocationPermissionRationale())
        verify(activity, times(1)).checkPermission(
            eq(Manifest.permission.ACCESS_FINE_LOCATION),
            any(),
            any()
        )
/*        verify(exactly = 1) {
            activity.checkPermission(
                Manifest.permission.ACCESS_FINE_LOCATION,
                any(),
                any()
            )
        }*/
        verify(activity, times(1)).checkPermission(
            eq(Manifest.permission.ACCESS_COARSE_LOCATION),
            any(),
            any()
        )
/*        verify(exactly = 1) {
            activity.checkPermission(
                Manifest.permission.ACCESS_COARSE_LOCATION,
                any(),
                any()
            )
        }*/
    }

    @Test(expected = IllegalStateException::class)
    fun requestFineLocationPermission_whenNoActivity_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val service = LocationPermissionService(context)

        service.requestFineLocationPermission()
    }

    @Test
    fun requestFineLocationPermission_whenActivityAndNoListener_callsExpectedMethod() {
        doNothing().whenever(launcher).launch(arrayOf(Manifest.permission.ACCESS_FINE_LOCATION))
//        justRun { launcher.launch(arrayOf(Manifest.permission.ACCESS_FINE_LOCATION)) }

        doAnswer { invocation ->
            val permissions = mapOf(Manifest.permission.ACCESS_FINE_LOCATION to true)

            val callback = invocation.getArgument<ActivityResultCallback<Map<String, Boolean>>>(1)
            callback.onActivityResult(permissions)
            return@doAnswer launcher
        }.whenever(activity).registerForActivityResult(
            any<ActivityResultContracts.RequestMultiplePermissions>(),
            any()
        )
/*        every {
            activity.registerForActivityResult(
                any<ActivityResultContracts.RequestMultiplePermissions>(),
                any()
            )
        }.answers { answer ->
            val permissions = mapOf(Manifest.permission.ACCESS_FINE_LOCATION to true)

            @Suppress("UNCHECKED_CAST")
            val callback = answer.invocation.args[1] as ActivityResultCallback<Map<String, Boolean>>
            callback.onActivityResult(permissions)
            return@answers launcher
        }*/

        val service = LocationPermissionService(activity)
        service.requestFineLocationPermission()

        verify(activity, only()).registerForActivityResult(
            any<ActivityResultContracts.RequestMultiplePermissions>(),
            any()
        )
/*        verify(exactly = 1) {
            activity.registerForActivityResult(
                any<ActivityResultContracts.RequestMultiplePermissions>(),
                any()
            )
        }*/
        verify(launcher, only()).launch(arrayOf(Manifest.permission.ACCESS_FINE_LOCATION))
//        verify(exactly = 1) { launcher.launch(arrayOf(Manifest.permission.ACCESS_FINE_LOCATION)) }
    }

    @Test
    fun requestFineLocationPermission_whenActivityAndListener_notifiesResultOfRequest() {
        doNothing().whenever(launcher).launch(arrayOf(Manifest.permission.ACCESS_FINE_LOCATION))
//        justRun { launcher.launch(arrayOf(Manifest.permission.ACCESS_FINE_LOCATION)) }

        doAnswer { invocation ->
            val permissions = mapOf(Manifest.permission.ACCESS_FINE_LOCATION to true)

            val callback = invocation.getArgument<ActivityResultCallback<Map<String, Boolean>>>(1)
            callback.onActivityResult(permissions)
            return@doAnswer launcher
        }.whenever(activity).registerForActivityResult(
            any<ActivityResultContracts.RequestMultiplePermissions>(),
            any()
        )
/*        every {
            activity.registerForActivityResult(
                any<ActivityResultContracts.RequestMultiplePermissions>(),
                any()
            )
        }.answers { answer ->
            val permissions = mapOf(Manifest.permission.ACCESS_FINE_LOCATION to true)

            @Suppress("UNCHECKED_CAST")
            val callback = answer.invocation.args[1] as ActivityResultCallback<Map<String, Boolean>>
            callback.onActivityResult(permissions)
            return@answers launcher
        }*/

        doNothing().whenever(listener).onLocationPermissionRequestResult(any())
//        justRun { listener.onLocationPermissionRequestResult(any()) }

        val service = LocationPermissionService(activity, listener)
        service.requestFineLocationPermission()

        verify(activity, only()).registerForActivityResult(
            any<ActivityResultContracts.RequestMultiplePermissions>(),
            any()
        )
/*        verify(exactly = 1) {
            activity.registerForActivityResult(
                any<ActivityResultContracts.RequestMultiplePermissions>(),
                any()
            )
        }*/
        verify(launcher, only()).launch(arrayOf(Manifest.permission.ACCESS_FINE_LOCATION))
//        verify(exactly = 1) { launcher.launch(arrayOf(Manifest.permission.ACCESS_FINE_LOCATION)) }

//        val slot = slot<LocationPermissionService.LocationPermissionResult>()
        verify(listener, only()).onLocationPermissionRequestResult(capture(locationPermissionResultCaptor))
//        verify(exactly = 1) { listener.onLocationPermissionRequestResult(capture(slot)) }

        val result = locationPermissionResultCaptor.value
//        val result = slot.captured
        assertFalse(result.cancelled)
        assertTrue(result.finePermissionGranted)
        assertFalse(result.coarsePermissionGranted)
        assertFalse(result.backgroundPermissionGranted)
        assertTrue(result.finePermissionRequested)
        assertFalse(result.coarsePermissionRequested)
        assertFalse(result.backgroundPermissionRequested)
    }

    @Test
    fun requestFineLocationPermission_whenCancelled_notifiesResultOfRequest() {
        doNothing().whenever(launcher).launch(arrayOf(Manifest.permission.ACCESS_FINE_LOCATION))
//        justRun { launcher.launch(arrayOf(Manifest.permission.ACCESS_FINE_LOCATION)) }

        doAnswer { invocation ->
            val permissions = emptyMap<String, Boolean>()
            val callback = invocation.getArgument<ActivityResultCallback<Map<String, Boolean>>>(1)
            callback.onActivityResult(permissions)
            return@doAnswer launcher
        }.whenever(activity).registerForActivityResult(
            any<ActivityResultContracts.RequestMultiplePermissions>(),
            any()
        )
/*        every {
            activity.registerForActivityResult(
                any<ActivityResultContracts.RequestMultiplePermissions>(),
                any()
            )
        }.answers { answer ->
            val permissions = emptyMap<String, Boolean>()

            @Suppress("UNCHECKED_CAST")
            val callback = answer.invocation.args[1] as ActivityResultCallback<Map<String, Boolean>>
            callback.onActivityResult(permissions)
            return@answers launcher
        }*/

        doNothing().whenever(listener).onLocationPermissionRequestResult(any())
//        justRun { listener.onLocationPermissionRequestResult(any()) }

        val service = LocationPermissionService(activity, listener)
        service.requestFineLocationPermission()

        verify(activity, only()).registerForActivityResult(
            any<ActivityResultContracts.RequestMultiplePermissions>(),
            any()
        )
/*        verify(exactly = 1) {
            activity.registerForActivityResult(
                any<ActivityResultContracts.RequestMultiplePermissions>(),
                any()
            )
        }*/
        verify(launcher, only()).launch(arrayOf(Manifest.permission.ACCESS_FINE_LOCATION))
//        verify(exactly = 1) { launcher.launch(arrayOf(Manifest.permission.ACCESS_FINE_LOCATION)) }

//        val slot = slot<LocationPermissionService.LocationPermissionResult>()
        verify(listener, times(1)).onLocationPermissionRequestResult(capture(locationPermissionResultCaptor))
//        verify(exactly = 1) { listener.onLocationPermissionRequestResult(capture(slot)) }

        val result = locationPermissionResultCaptor.value
//        val result = slot.captured
        assertTrue(result.cancelled)
        assertFalse(result.finePermissionGranted)
        assertFalse(result.coarsePermissionGranted)
        assertFalse(result.backgroundPermissionGranted)
        assertFalse(result.finePermissionRequested)
        assertFalse(result.coarsePermissionRequested)
        assertFalse(result.backgroundPermissionRequested)
    }

    @Test(expected = IllegalStateException::class)
    fun requestCoarseLocationPermission_whenNoActivity_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val service = LocationPermissionService(context)

        service.requestCoarseLocationPermission()
    }

    @Test
    fun requestCoarseLocationPermission_whenActivityAndNoListener_callsExpectedMethod() {
        doNothing().whenever(launcher).launch(arrayOf(Manifest.permission.ACCESS_COARSE_LOCATION))
//        justRun { launcher.launch(arrayOf(Manifest.permission.ACCESS_COARSE_LOCATION)) }

        doAnswer { invocation ->
            val permissions = mapOf(Manifest.permission.ACCESS_COARSE_LOCATION to true)
            val callback = invocation.getArgument<ActivityResultCallback<Map<String, Boolean>>>(1)
            callback.onActivityResult(permissions)
            return@doAnswer launcher
        }.whenever(activity).registerForActivityResult(
            any<ActivityResultContracts.RequestMultiplePermissions>(),
            any()
        )
/*        every {
            activity.registerForActivityResult(
                any<ActivityResultContracts.RequestMultiplePermissions>(),
                any()
            )
        }.answers { answer ->
            val permissions = mapOf(Manifest.permission.ACCESS_COARSE_LOCATION to true)

            @Suppress("UNCHECKED_CAST")
            val callback = answer.invocation.args[1] as ActivityResultCallback<Map<String, Boolean>>
            callback.onActivityResult(permissions)
            return@answers launcher
        }*/

        val service = LocationPermissionService(activity)
        service.requestCoarseLocationPermission()

        verify(activity, only()).registerForActivityResult(
            any<ActivityResultContracts.RequestMultiplePermissions>(),
            any()
        )
/*        verify(exactly = 1) {
            activity.registerForActivityResult(
                any<ActivityResultContracts.RequestMultiplePermissions>(),
                any()
            )
        }*/
        verify(launcher, only()).launch(arrayOf(Manifest.permission.ACCESS_COARSE_LOCATION))
//        verify(exactly = 1) { launcher.launch(arrayOf(Manifest.permission.ACCESS_COARSE_LOCATION)) }
    }

    @Test
    fun requestCoarseLocationPermission_whenActivityAndListener_notifiesResultOfRequest() {
        doNothing().whenever(launcher).launch(arrayOf(Manifest.permission.ACCESS_COARSE_LOCATION))
//        justRun { launcher.launch(arrayOf(Manifest.permission.ACCESS_COARSE_LOCATION)) }

        doAnswer { invocation ->
            val permissions = mapOf(Manifest.permission.ACCESS_COARSE_LOCATION to true)
            val callback = invocation.getArgument<ActivityResultCallback<Map<String, Boolean>>>(1)
            callback.onActivityResult(permissions)
            return@doAnswer launcher
        }.whenever(activity).registerForActivityResult(
            any<ActivityResultContracts.RequestMultiplePermissions>(),
            any()
        )
/*        every {
            activity.registerForActivityResult(
                any<ActivityResultContracts.RequestMultiplePermissions>(),
                any()
            )
        }.answers { answer ->
            val permissions = mapOf(Manifest.permission.ACCESS_COARSE_LOCATION to true)

            @Suppress("UNCHECKED_CAST")
            val callback = answer.invocation.args[1] as ActivityResultCallback<Map<String, Boolean>>
            callback.onActivityResult(permissions)
            return@answers launcher
        }*/

        doNothing().whenever(listener).onLocationPermissionRequestResult(any())
//        justRun { listener.onLocationPermissionRequestResult(any()) }

        val service = LocationPermissionService(activity, listener)
        service.requestCoarseLocationPermission()

        verify(activity, only()).registerForActivityResult(
            any<ActivityResultContracts.RequestMultiplePermissions>(),
            any()
        )
/*        verify(exactly = 1) {
            activity.registerForActivityResult(
                any<ActivityResultContracts.RequestMultiplePermissions>(),
                any()
            )
        }*/
        verify(launcher, only()).launch(arrayOf(Manifest.permission.ACCESS_COARSE_LOCATION))
//        verify(exactly = 1) { launcher.launch(arrayOf(Manifest.permission.ACCESS_COARSE_LOCATION)) }

//        val slot = slot<LocationPermissionService.LocationPermissionResult>()
        verify(listener, only()).onLocationPermissionRequestResult(capture(locationPermissionResultCaptor))
//        verify(exactly = 1) { listener.onLocationPermissionRequestResult(capture(slot)) }

        val result = locationPermissionResultCaptor.value
//        val result = slot.captured
        assertFalse(result.cancelled)
        assertFalse(result.finePermissionGranted)
        assertTrue(result.coarsePermissionGranted)
        assertFalse(result.backgroundPermissionGranted)
        assertFalse(result.finePermissionRequested)
        assertTrue(result.coarsePermissionRequested)
        assertFalse(result.backgroundPermissionRequested)
    }

    @Test
    fun requestCoarseLocationPermission_whenCancelled_notifiesResultOfRequest() {
        doNothing().whenever(launcher).launch(arrayOf(Manifest.permission.ACCESS_COARSE_LOCATION))
//        justRun { launcher.launch(arrayOf(Manifest.permission.ACCESS_COARSE_LOCATION)) }

        doAnswer { invocation ->
            val permissions = emptyMap<String, Boolean>()
            val callback = invocation.getArgument<ActivityResultCallback<Map<String, Boolean>>>(1)
            callback.onActivityResult(permissions)
            return@doAnswer launcher
        }.whenever(activity).registerForActivityResult(
            any<ActivityResultContracts.RequestMultiplePermissions>(),
            any()
        )
/*        every {
            activity.registerForActivityResult(
                any<ActivityResultContracts.RequestMultiplePermissions>(),
                any()
            )
        }.answers { answer ->
            val permissions = emptyMap<String, Boolean>()

            @Suppress("UNCHECKED_CAST")
            val callback = answer.invocation.args[1] as ActivityResultCallback<Map<String, Boolean>>
            callback.onActivityResult(permissions)
            return@answers launcher
        }*/

        doNothing().whenever(listener).onLocationPermissionRequestResult(any())
//        justRun { listener.onLocationPermissionRequestResult(any()) }

        val service = LocationPermissionService(activity, listener)
        service.requestCoarseLocationPermission()

        verify(activity, only()).registerForActivityResult(
            any<ActivityResultContracts.RequestMultiplePermissions>(),
            any()
        )
/*        verify(exactly = 1) {
            activity.registerForActivityResult(
                any<ActivityResultContracts.RequestMultiplePermissions>(),
                any()
            )
        }*/
        verify(launcher, only()).launch(arrayOf(Manifest.permission.ACCESS_COARSE_LOCATION))
//        verify(exactly = 1) { launcher.launch(arrayOf(Manifest.permission.ACCESS_COARSE_LOCATION)) }

//        val slot = slot<LocationPermissionService.LocationPermissionResult>()
        verify(listener, only()).onLocationPermissionRequestResult(capture(locationPermissionResultCaptor))
//        verify(exactly = 1) { listener.onLocationPermissionRequestResult(capture(slot)) }

        val result = locationPermissionResultCaptor.value
//        val result = slot.captured
        assertTrue(result.cancelled)
        assertFalse(result.finePermissionGranted)
        assertFalse(result.coarsePermissionGranted)
        assertFalse(result.backgroundPermissionGranted)
        assertFalse(result.finePermissionRequested)
        assertFalse(result.coarsePermissionRequested)
        assertFalse(result.backgroundPermissionRequested)
    }

    @Test(expected = IllegalStateException::class)
    fun requestBackgroundFineLocationPermission_whenNoActivity_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val service = LocationPermissionService(context)

        service.requestBackgroundFineLocationPermission()
    }

    @Config(sdk = [Build.VERSION_CODES.Q])
    @Test
    fun requestBackgroundFineLocationPermission_whenActivityNoListenerAndSdkQ_callsExpectedMethod() {
        doNothing().whenever(launcher).launch(
            arrayOf(
                Manifest.permission.ACCESS_FINE_LOCATION,
                Manifest.permission.ACCESS_BACKGROUND_LOCATION
            )
        )
/*        justRun {
            launcher.launch(
                arrayOf(
                    Manifest.permission.ACCESS_FINE_LOCATION,
                    Manifest.permission.ACCESS_BACKGROUND_LOCATION
                )
            )
        }*/

        doAnswer { invocation ->
            val permissions = mapOf(
                Manifest.permission.ACCESS_FINE_LOCATION to true,
                Manifest.permission.ACCESS_BACKGROUND_LOCATION to true
            )
            val callback = invocation.getArgument<ActivityResultCallback<Map<String, Boolean>>>(1)
            callback.onActivityResult(permissions)
            return@doAnswer launcher
        }.whenever(activity).registerForActivityResult(
            any<ActivityResultContracts.RequestMultiplePermissions>(),
            any()
        )
/*        every {
            activity.registerForActivityResult(
                any<ActivityResultContracts.RequestMultiplePermissions>(),
                any()
            )
        }.answers { answer ->
            val permissions = mapOf(
                Manifest.permission.ACCESS_FINE_LOCATION to true,
                Manifest.permission.ACCESS_BACKGROUND_LOCATION to true
            )

            @Suppress("UNCHECKED_CAST")
            val callback = answer.invocation.args[1] as ActivityResultCallback<Map<String, Boolean>>
            callback.onActivityResult(permissions)
            return@answers launcher
        }*/

        val service = LocationPermissionService(activity)
        service.requestBackgroundFineLocationPermission()

        verify(activity, only()).registerForActivityResult(
            any<ActivityResultContracts.RequestMultiplePermissions>(),
            any()
        )
/*        verify(exactly = 1) {
            activity.registerForActivityResult(
                any<ActivityResultContracts.RequestMultiplePermissions>(),
                any()
            )
        }*/
        verify(launcher, only()).launch(
            arrayOf(
                Manifest.permission.ACCESS_FINE_LOCATION,
                Manifest.permission.ACCESS_BACKGROUND_LOCATION
            )
        )
/*        verify(exactly = 1) {
            launcher.launch(
                arrayOf(
                    Manifest.permission.ACCESS_FINE_LOCATION,
                    Manifest.permission.ACCESS_BACKGROUND_LOCATION
                )
            )
        }*/
    }

    @Config(sdk = [Build.VERSION_CODES.Q])
    @Test
    fun requestBackgroundFineLocationPermission_whenActivityListenerAndSdkQ_notifiesResultOfRequest() {
        doNothing().whenever(launcher).launch(
            arrayOf(
                Manifest.permission.ACCESS_FINE_LOCATION,
                Manifest.permission.ACCESS_BACKGROUND_LOCATION
            )
        )
/*        justRun {
            launcher.launch(
                arrayOf(
                    Manifest.permission.ACCESS_FINE_LOCATION,
                    Manifest.permission.ACCESS_BACKGROUND_LOCATION
                )
            )
        }*/

        doAnswer { invocation ->
            val permissions = mapOf(
                Manifest.permission.ACCESS_FINE_LOCATION to true,
                Manifest.permission.ACCESS_BACKGROUND_LOCATION to true
            )
            val callback = invocation.getArgument<ActivityResultCallback<Map<String, Boolean>>>(1)
            callback.onActivityResult(permissions)
            return@doAnswer launcher
        }.whenever(activity).registerForActivityResult(
            any<ActivityResultContracts.RequestMultiplePermissions>(),
            any()
        )
/*        every {
            activity.registerForActivityResult(
                any<ActivityResultContracts.RequestMultiplePermissions>(),
                any()
            )
        }.answers { answer ->
            val permissions = mapOf(
                Manifest.permission.ACCESS_FINE_LOCATION to true,
                Manifest.permission.ACCESS_BACKGROUND_LOCATION to true
            )

            @Suppress("UNCHECKED_CAST")
            val callback = answer.invocation.args[1] as ActivityResultCallback<Map<String, Boolean>>
            callback.onActivityResult(permissions)
            return@answers launcher
        }*/

        doNothing().whenever(listener).onLocationPermissionRequestResult(any())
//        justRun { listener.onLocationPermissionRequestResult(any()) }

        val service = LocationPermissionService(activity, listener)
        service.requestBackgroundFineLocationPermission()

        verify(activity, only()).registerForActivityResult(
            any<ActivityResultContracts.RequestMultiplePermissions>(),
            any()
        )
/*        verify(exactly = 1) {
            activity.registerForActivityResult(
                any<ActivityResultContracts.RequestMultiplePermissions>(),
                any()
            )
        }*/
        verify(launcher, only()).launch(
            arrayOf(
                Manifest.permission.ACCESS_FINE_LOCATION,
                Manifest.permission.ACCESS_BACKGROUND_LOCATION
            )
        )
/*        verify(exactly = 1) {
            launcher.launch(
                arrayOf(
                    Manifest.permission.ACCESS_FINE_LOCATION,
                    Manifest.permission.ACCESS_BACKGROUND_LOCATION
                )
            )
        }*/

//        val slot = slot<LocationPermissionService.LocationPermissionResult>()
        verify(listener, only()).onLocationPermissionRequestResult(capture(locationPermissionResultCaptor))
//        verify(exactly = 1) { listener.onLocationPermissionRequestResult(capture(slot)) }

        val result = locationPermissionResultCaptor.value
//        val result = slot.captured
        assertFalse(result.cancelled)
        assertTrue(result.finePermissionGranted)
        assertFalse(result.coarsePermissionGranted)
        assertTrue(result.backgroundPermissionGranted)
        assertTrue(result.finePermissionRequested)
        assertFalse(result.coarsePermissionRequested)
        assertTrue(result.backgroundPermissionRequested)
    }

    @Config(sdk = [Build.VERSION_CODES.Q])
    @Test
    fun requestBackgroundFineLocationPermission_whenCancelledAndSdkQ_notifiesResultOfRequest() {
        doNothing().whenever(launcher).launch(
            arrayOf(
                Manifest.permission.ACCESS_FINE_LOCATION,
                Manifest.permission.ACCESS_BACKGROUND_LOCATION
            )
        )
/*        justRun {
            launcher.launch(
                arrayOf(
                    Manifest.permission.ACCESS_FINE_LOCATION,
                    Manifest.permission.ACCESS_BACKGROUND_LOCATION
                )
            )
        }*/

        doAnswer { invocation ->
            val permissions = emptyMap<String, Boolean>()
            val callback = invocation.getArgument<ActivityResultCallback<Map<String, Boolean>>>(1)
            callback.onActivityResult(permissions)
            return@doAnswer launcher
        }.whenever(activity).registerForActivityResult(
            any<ActivityResultContracts.RequestMultiplePermissions>(),
            any()
        )
/*        every {
            activity.registerForActivityResult(
                any<ActivityResultContracts.RequestMultiplePermissions>(),
                any()
            )
        }.answers { answer ->
            val permissions = emptyMap<String, Boolean>()

            @Suppress("UNCHECKED_CAST")
            val callback = answer.invocation.args[1] as ActivityResultCallback<Map<String, Boolean>>
            callback.onActivityResult(permissions)
            return@answers launcher
        }*/

        doNothing().whenever(listener).onLocationPermissionRequestResult(any())
//        justRun { listener.onLocationPermissionRequestResult(any()) }

        val service = LocationPermissionService(activity, listener)
        service.requestBackgroundFineLocationPermission()

        verify(activity, only()).registerForActivityResult(
            any<ActivityResultContracts.RequestMultiplePermissions>(),
            any()
        )
/*        verify(exactly = 1) {
            activity.registerForActivityResult(
                any<ActivityResultContracts.RequestMultiplePermissions>(),
                any()
            )
        }*/
        verify(launcher, only()).launch(
            arrayOf(
                Manifest.permission.ACCESS_FINE_LOCATION,
                Manifest.permission.ACCESS_BACKGROUND_LOCATION
            )
        )
/*        verify(exactly = 1) {
            launcher.launch(
                arrayOf(
                    Manifest.permission.ACCESS_FINE_LOCATION,
                    Manifest.permission.ACCESS_BACKGROUND_LOCATION
                )
            )
        }*/

//        val slot = slot<LocationPermissionService.LocationPermissionResult>()
        verify(listener, only()).onLocationPermissionRequestResult(capture(locationPermissionResultCaptor))
//        verify(exactly = 1) { listener.onLocationPermissionRequestResult(capture(slot)) }

        val result = locationPermissionResultCaptor.value
//        val result = slot.captured
        assertTrue(result.cancelled)
        assertFalse(result.finePermissionGranted)
        assertFalse(result.coarsePermissionGranted)
        assertFalse(result.backgroundPermissionGranted)
        assertFalse(result.finePermissionRequested)
        assertFalse(result.coarsePermissionRequested)
        assertFalse(result.backgroundPermissionRequested)
    }

    @Config(sdk = [Build.VERSION_CODES.P])
    @Test
    fun requestBackgroundFineLocationPermission_whenActivityNoListenerAndSdkP_callsExpectedMethod() {
        doNothing().whenever(launcher).launch(arrayOf(Manifest.permission.ACCESS_FINE_LOCATION))
//        justRun { launcher.launch(arrayOf(Manifest.permission.ACCESS_FINE_LOCATION)) }

        doAnswer { invocation ->
            val permissions = mapOf(Manifest.permission.ACCESS_FINE_LOCATION to true)
            val callback = invocation.getArgument<ActivityResultCallback<Map<String, Boolean>>>(1)
            callback.onActivityResult(permissions)
            return@doAnswer launcher
        }.whenever(activity).registerForActivityResult(
            any<ActivityResultContracts.RequestMultiplePermissions>(),
            any()
        )
/*        every {
            activity.registerForActivityResult(
                any<ActivityResultContracts.RequestMultiplePermissions>(),
                any()
            )
        }.answers { answer ->
            val permissions = mapOf(Manifest.permission.ACCESS_FINE_LOCATION to true)

            @Suppress("UNCHECKED_CAST")
            val callback = answer.invocation.args[1] as ActivityResultCallback<Map<String, Boolean>>
            callback.onActivityResult(permissions)
            return@answers launcher
        }*/

        val service = LocationPermissionService(activity)
        service.requestBackgroundFineLocationPermission()

        verify(activity, only()).registerForActivityResult(
            any<ActivityResultContracts.RequestMultiplePermissions>(),
            any()
        )
/*        verify(exactly = 1) {
            activity.registerForActivityResult(
                any<ActivityResultContracts.RequestMultiplePermissions>(),
                any()
            )
        }*/
        verify(launcher, only()).launch(arrayOf(Manifest.permission.ACCESS_FINE_LOCATION))
/*        verify(exactly = 1) {
            launcher.launch(arrayOf(Manifest.permission.ACCESS_FINE_LOCATION))
        }*/
    }

    @Config(sdk = [Build.VERSION_CODES.P])
    @Test
    fun requestBackgroundFineLocationPermission_whenActivityListenerAndSdkP_notifiesResultOfRequest() {
        doNothing().whenever(launcher).launch(arrayOf(Manifest.permission.ACCESS_FINE_LOCATION))
//        justRun { launcher.launch(arrayOf(Manifest.permission.ACCESS_FINE_LOCATION)) }

        doAnswer { invocation ->
            val permissions = mapOf(Manifest.permission.ACCESS_FINE_LOCATION to true)
            val callback = invocation.getArgument<ActivityResultCallback<Map<String, Boolean>>>(1)
            callback.onActivityResult(permissions)
            return@doAnswer launcher
        }.whenever(activity).registerForActivityResult(
            any<ActivityResultContracts.RequestMultiplePermissions>(),
            any()
        )
/*        every {
            activity.registerForActivityResult(
                any<ActivityResultContracts.RequestMultiplePermissions>(),
                any()
            )
        }.answers { answer ->
            val permissions = mapOf(Manifest.permission.ACCESS_FINE_LOCATION to true)

            @Suppress("UNCHECKED_CAST")
            val callback = answer.invocation.args[1] as ActivityResultCallback<Map<String, Boolean>>
            callback.onActivityResult(permissions)
            return@answers launcher
        }*/

        doNothing().whenever(listener).onLocationPermissionRequestResult(any())
//        justRun { listener.onLocationPermissionRequestResult(any()) }

        val service = LocationPermissionService(activity, listener)
        service.requestBackgroundFineLocationPermission()

        verify(activity, only()).registerForActivityResult(
            any<ActivityResultContracts.RequestMultiplePermissions>(),
            any()
        )
/*        verify(exactly = 1) {
            activity.registerForActivityResult(
                any<ActivityResultContracts.RequestMultiplePermissions>(),
                any()
            )
        }*/
        verify(launcher, only()).launch(arrayOf(Manifest.permission.ACCESS_FINE_LOCATION))
//        verify(exactly = 1) { launcher.launch(arrayOf(Manifest.permission.ACCESS_FINE_LOCATION)) }

//        val slot = slot<LocationPermissionService.LocationPermissionResult>()
        verify(listener, only()).onLocationPermissionRequestResult(capture(locationPermissionResultCaptor))
//        verify(exactly = 1) { listener.onLocationPermissionRequestResult(capture(slot)) }

        val result = locationPermissionResultCaptor.value
//        val result = slot.captured
        assertFalse(result.cancelled)
        assertTrue(result.finePermissionGranted)
        assertFalse(result.coarsePermissionGranted)
        assertFalse(result.backgroundPermissionGranted)
        assertTrue(result.finePermissionRequested)
        assertFalse(result.coarsePermissionRequested)
        assertFalse(result.backgroundPermissionRequested)
    }

    @Config(sdk = [Build.VERSION_CODES.P])
    @Test
    fun requestBackgroundFineLocationPermission_whenCancelledAndSdkP_notifiesResultOfRequest() {
        doNothing().whenever(launcher).launch(arrayOf(Manifest.permission.ACCESS_FINE_LOCATION))
//        justRun { launcher.launch(arrayOf(Manifest.permission.ACCESS_FINE_LOCATION)) }

        doAnswer { invocation ->
            val permissions = emptyMap<String, Boolean>()
            val callback = invocation.getArgument<ActivityResultCallback<Map<String, Boolean>>>(1)
            callback.onActivityResult(permissions)
            return@doAnswer launcher
        }.whenever(activity).registerForActivityResult(
            any<ActivityResultContracts.RequestMultiplePermissions>(),
            any()
        )
/*        every {
            activity.registerForActivityResult(
                any<ActivityResultContracts.RequestMultiplePermissions>(),
                any()
            )
        }.answers { answer ->
            val permissions = emptyMap<String, Boolean>()

            @Suppress("UNCHECKED_CAST")
            val callback = answer.invocation.args[1] as ActivityResultCallback<Map<String, Boolean>>
            callback.onActivityResult(permissions)
            return@answers launcher
        }*/

        doNothing().whenever(listener).onLocationPermissionRequestResult(any())
//        justRun { listener.onLocationPermissionRequestResult(any()) }

        val service = LocationPermissionService(activity, listener)
        service.requestBackgroundFineLocationPermission()

        verify(activity, only()).registerForActivityResult(
            any<ActivityResultContracts.RequestMultiplePermissions>(),
            any()
        )
/*        verify(exactly = 1) {
            activity.registerForActivityResult(
                any<ActivityResultContracts.RequestMultiplePermissions>(),
                any()
            )
        }*/
        verify(launcher, only()).launch(arrayOf(Manifest.permission.ACCESS_FINE_LOCATION))
//        verify(exactly = 1) { launcher.launch(arrayOf(Manifest.permission.ACCESS_FINE_LOCATION)) }

//        val slot = slot<LocationPermissionService.LocationPermissionResult>()
        verify(listener, only()).onLocationPermissionRequestResult(capture(locationPermissionResultCaptor))
//        verify(exactly = 1) { listener.onLocationPermissionRequestResult(capture(slot)) }

        val result = locationPermissionResultCaptor.value
//        val result = slot.captured
        assertTrue(result.cancelled)
        assertFalse(result.finePermissionGranted)
        assertFalse(result.coarsePermissionGranted)
        assertFalse(result.backgroundPermissionGranted)
        assertFalse(result.finePermissionRequested)
        assertFalse(result.coarsePermissionRequested)
        assertFalse(result.backgroundPermissionRequested)
    }

    @Test(expected = IllegalStateException::class)
    fun requestBackgroundCoarseLocationPermission_whenNoActivity_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val service = LocationPermissionService(context)

        service.requestBackgroundCoarseLocationPermission()
    }

    @Config(sdk = [Build.VERSION_CODES.Q])
    @Test
    fun requestBackgroundCoarseLocationPermission_whenActivityNoListenerAndSdkQ_callsExpectedMethod() {
        doNothing().whenever(launcher).launch(
            arrayOf(
                Manifest.permission.ACCESS_COARSE_LOCATION,
                Manifest.permission.ACCESS_BACKGROUND_LOCATION
            )
        )
/*        justRun {
            launcher.launch(
                arrayOf(
                    Manifest.permission.ACCESS_COARSE_LOCATION,
                    Manifest.permission.ACCESS_BACKGROUND_LOCATION
                )
            )
        }*/

        doAnswer { invocation ->
            val permissions = emptyMap<String, Boolean>()
            val callback = invocation.getArgument<ActivityResultCallback<Map<String, Boolean>>>(1)
            callback.onActivityResult(permissions)
            return@doAnswer launcher
        }.whenever(activity).registerForActivityResult(
            any<ActivityResultContracts.RequestMultiplePermissions>(),
            any()
        )
/*        every {
            activity.registerForActivityResult(
                any<ActivityResultContracts.RequestMultiplePermissions>(),
                any()
            )
        }.answers { answer ->
            val permissions = emptyMap<String, Boolean>()

            @Suppress("UNCHECKED_CAST")
            val callback = answer.invocation.args[1] as ActivityResultCallback<Map<String, Boolean>>
            callback.onActivityResult(permissions)
            return@answers launcher
        }*/

        val service = LocationPermissionService(activity)
        service.requestBackgroundCoarseLocationPermission()

        verify(activity, only()).registerForActivityResult(
            any<ActivityResultContracts.RequestMultiplePermissions>(),
            any()
        )
/*        verify(exactly = 1) {
            activity.registerForActivityResult(
                any<ActivityResultContracts.RequestMultiplePermissions>(),
                any()
            )
        }*/
        verify(launcher, only()).launch(
            arrayOf(
                Manifest.permission.ACCESS_COARSE_LOCATION,
                Manifest.permission.ACCESS_BACKGROUND_LOCATION
            )
        )
/*        verify(exactly = 1) {
            launcher.launch(
                arrayOf(
                    Manifest.permission.ACCESS_COARSE_LOCATION,
                    Manifest.permission.ACCESS_BACKGROUND_LOCATION
                )
            )
        }*/
    }

    @Config(sdk = [Build.VERSION_CODES.Q])
    @Test
    fun requestBackgroundCoarseLocationPermission_whenActivityListenerAndSdkQ_notifiesResultOfRequest() {
        doNothing().whenever(launcher).launch(
            arrayOf(
                Manifest.permission.ACCESS_COARSE_LOCATION,
                Manifest.permission.ACCESS_BACKGROUND_LOCATION
            )
        )
/*        justRun {
            launcher.launch(
                arrayOf(
                    Manifest.permission.ACCESS_COARSE_LOCATION,
                    Manifest.permission.ACCESS_BACKGROUND_LOCATION
                )
            )
        }*/

        doAnswer { invocation ->
            val permissions = mapOf(
                Manifest.permission.ACCESS_COARSE_LOCATION to true,
                Manifest.permission.ACCESS_BACKGROUND_LOCATION to true
            )
            val callback = invocation.getArgument<ActivityResultCallback<Map<String, Boolean>>>(1)
            callback.onActivityResult(permissions)
            return@doAnswer launcher
        }.whenever(activity).registerForActivityResult(
            any<ActivityResultContracts.RequestMultiplePermissions>(),
            any()
        )
/*        every {
            activity.registerForActivityResult(
                any<ActivityResultContracts.RequestMultiplePermissions>(),
                any()
            )
        }.answers { answer ->
            val permissions = mapOf(
                Manifest.permission.ACCESS_COARSE_LOCATION to true,
                Manifest.permission.ACCESS_BACKGROUND_LOCATION to true
            )

            @Suppress("UNCHECKED_CAST")
            val callback = answer.invocation.args[1] as ActivityResultCallback<Map<String, Boolean>>
            callback.onActivityResult(permissions)
            return@answers launcher
        }*/

        doNothing().whenever(listener).onLocationPermissionRequestResult(any())
//        justRun { listener.onLocationPermissionRequestResult(any()) }

        val service = LocationPermissionService(activity, listener)
        service.requestBackgroundCoarseLocationPermission()

        verify(activity, only()).registerForActivityResult(
            any<ActivityResultContracts.RequestMultiplePermissions>(),
            any()
        )
/*        verify(exactly = 1) {
            activity.registerForActivityResult(
                any<ActivityResultContracts.RequestMultiplePermissions>(),
                any()
            )
        }*/
        verify(launcher, only()).launch(
            arrayOf(
                Manifest.permission.ACCESS_COARSE_LOCATION,
                Manifest.permission.ACCESS_BACKGROUND_LOCATION
            )
        )
/*        verify(exactly = 1) {
            launcher.launch(
                arrayOf(
                    Manifest.permission.ACCESS_COARSE_LOCATION,
                    Manifest.permission.ACCESS_BACKGROUND_LOCATION
                )
            )
        }*/

//        val slot = slot<LocationPermissionService.LocationPermissionResult>()
        verify(listener, only()).onLocationPermissionRequestResult(capture(locationPermissionResultCaptor))
//        verify(exactly = 1) { listener.onLocationPermissionRequestResult(capture(slot)) }

        val result = locationPermissionResultCaptor.value
//        val result = slot.captured
        assertFalse(result.cancelled)
        assertFalse(result.finePermissionGranted)
        assertTrue(result.coarsePermissionGranted)
        assertTrue(result.backgroundPermissionGranted)
        assertFalse(result.finePermissionRequested)
        assertTrue(result.coarsePermissionRequested)
        assertTrue(result.backgroundPermissionRequested)
    }

    @Config(sdk = [Build.VERSION_CODES.Q])
    @Test
    fun requestBackgroundCoarseLocationPermission_whenCancelledAndSdkQ_notifiesResultOfRequest() {
        doNothing().whenever(launcher).launch(
            arrayOf(
                Manifest.permission.ACCESS_COARSE_LOCATION,
                Manifest.permission.ACCESS_BACKGROUND_LOCATION
            )
        )
/*        justRun {
            launcher.launch(
                arrayOf(
                    Manifest.permission.ACCESS_COARSE_LOCATION,
                    Manifest.permission.ACCESS_BACKGROUND_LOCATION
                )
            )
        }*/

        doAnswer { invocation ->
            val permissions = emptyMap<String, Boolean>()
            val callback = invocation.getArgument<ActivityResultCallback<Map<String, Boolean>>>(1)
            callback.onActivityResult(permissions)
            return@doAnswer launcher
        }.whenever(activity).registerForActivityResult(
            any<ActivityResultContracts.RequestMultiplePermissions>(),
            any()
        )
/*        every {
            activity.registerForActivityResult(
                any<ActivityResultContracts.RequestMultiplePermissions>(),
                any()
            )
        }.answers { answer ->
            val permissions = emptyMap<String, Boolean>()

            @Suppress("UNCHECKED_CAST")
            val callback = answer.invocation.args[1] as ActivityResultCallback<Map<String, Boolean>>
            callback.onActivityResult(permissions)
            return@answers launcher
        }*/

        doNothing().whenever(listener).onLocationPermissionRequestResult(any())
//        justRun { listener.onLocationPermissionRequestResult(any()) }

        val service = LocationPermissionService(activity, listener)
        service.requestBackgroundCoarseLocationPermission()

        verify(activity, only()).registerForActivityResult(
            any<ActivityResultContracts.RequestMultiplePermissions>(),
            any()
        )
/*        verify(exactly = 1) {
            activity.registerForActivityResult(
                any<ActivityResultContracts.RequestMultiplePermissions>(),
                any()
            )
        }*/
        verify(launcher, only()).launch(
            arrayOf(
                Manifest.permission.ACCESS_COARSE_LOCATION,
                Manifest.permission.ACCESS_BACKGROUND_LOCATION
            )
        )
/*        verify(exactly = 1) {
            launcher.launch(
                arrayOf(
                    Manifest.permission.ACCESS_COARSE_LOCATION,
                    Manifest.permission.ACCESS_BACKGROUND_LOCATION
                )
            )
        }*/

//        val slot = slot<LocationPermissionService.LocationPermissionResult>()
        verify(listener, only()).onLocationPermissionRequestResult(capture(locationPermissionResultCaptor))
//        verify(exactly = 1) { listener.onLocationPermissionRequestResult(capture(slot)) }

        val result = locationPermissionResultCaptor.value
//        val result = slot.captured
        assertTrue(result.cancelled)
        assertFalse(result.finePermissionGranted)
        assertFalse(result.coarsePermissionGranted)
        assertFalse(result.backgroundPermissionGranted)
        assertFalse(result.finePermissionRequested)
        assertFalse(result.coarsePermissionRequested)
        assertFalse(result.backgroundPermissionRequested)
    }

    @Config(sdk = [Build.VERSION_CODES.P])
    @Test
    fun requestBackgroundCoarseLocationPermission_whenActivityNoListenerAndSdkP_callsExpectedMethod() {
        doNothing().whenever(launcher).launch(arrayOf(Manifest.permission.ACCESS_COARSE_LOCATION))
//        justRun { launcher.launch(arrayOf(Manifest.permission.ACCESS_COARSE_LOCATION)) }

        doAnswer { invocation ->
            val permissions = emptyMap<String, Boolean>()
            val callback = invocation.getArgument<ActivityResultCallback<Map<String, Boolean>>>(1)
            callback.onActivityResult(permissions)
            return@doAnswer launcher
        }.whenever(activity).registerForActivityResult(
            any<ActivityResultContracts.RequestMultiplePermissions>(),
            any()
        )
/*        every {
            activity.registerForActivityResult(
                any<ActivityResultContracts.RequestMultiplePermissions>(),
                any()
            )
        }.answers { answer ->
            val permissions = emptyMap<String, Boolean>()

            @Suppress("UNCHECKED_CAST")
            val callback = answer.invocation.args[1] as ActivityResultCallback<Map<String, Boolean>>
            callback.onActivityResult(permissions)
            return@answers launcher
        }*/

        val service = LocationPermissionService(activity)
        service.requestBackgroundCoarseLocationPermission()

        verify(activity, only()).registerForActivityResult(
            any<ActivityResultContracts.RequestMultiplePermissions>(),
            any()
        )
/*        verify(exactly = 1) {
            activity.registerForActivityResult(
                any<ActivityResultContracts.RequestMultiplePermissions>(),
                any()
            )
        }*/
        verify(launcher, only()).launch(arrayOf(Manifest.permission.ACCESS_COARSE_LOCATION))
//        verify(exactly = 1) { launcher.launch(arrayOf(Manifest.permission.ACCESS_COARSE_LOCATION)) }
    }

    @Config(sdk = [Build.VERSION_CODES.P])
    @Test
    fun requestBackgroundCoarseLocationPermission_whenActivityListenerAndSdkP_notifiesResultOfRequest() {
        doNothing().whenever(launcher).launch(arrayOf(Manifest.permission.ACCESS_COARSE_LOCATION))
//        justRun { launcher.launch(arrayOf(Manifest.permission.ACCESS_COARSE_LOCATION)) }

        doAnswer { invocation ->
            val permissions = mapOf(Manifest.permission.ACCESS_COARSE_LOCATION to true)
            val callback = invocation.getArgument<ActivityResultCallback<Map<String, Boolean>>>(1)
            callback.onActivityResult(permissions)
            return@doAnswer launcher
        }.whenever(activity).registerForActivityResult(
            any<ActivityResultContracts.RequestMultiplePermissions>(),
            any()
        )
/*        every {
            activity.registerForActivityResult(
                any<ActivityResultContracts.RequestMultiplePermissions>(),
                any()
            )
        }.answers { answer ->
            val permissions = mapOf(Manifest.permission.ACCESS_COARSE_LOCATION to true)

            @Suppress("UNCHECKED_CAST")
            val callback = answer.invocation.args[1] as ActivityResultCallback<Map<String, Boolean>>
            callback.onActivityResult(permissions)
            return@answers launcher
        }*/

        doNothing().whenever(listener).onLocationPermissionRequestResult(any())
//        justRun { listener.onLocationPermissionRequestResult(any()) }

        val service = LocationPermissionService(activity, listener)
        service.requestBackgroundCoarseLocationPermission()

        verify(activity, only()).registerForActivityResult(
            any<ActivityResultContracts.RequestMultiplePermissions>(),
            any()
        )
/*        verify(exactly = 1) {
            activity.registerForActivityResult(
                any<ActivityResultContracts.RequestMultiplePermissions>(),
                any()
            )
        }*/
        verify(launcher, only()).launch(arrayOf(Manifest.permission.ACCESS_COARSE_LOCATION))
/*        verify(exactly = 1) {
            launcher.launch(arrayOf(Manifest.permission.ACCESS_COARSE_LOCATION))
        }*/

//        val slot = slot<LocationPermissionService.LocationPermissionResult>()
        verify(listener, only()).onLocationPermissionRequestResult(capture(locationPermissionResultCaptor))
//        verify(exactly = 1) { listener.onLocationPermissionRequestResult(capture(slot)) }

        val result = locationPermissionResultCaptor.value
//        val result = slot.captured
        assertFalse(result.cancelled)
        assertFalse(result.finePermissionGranted)
        assertTrue(result.coarsePermissionGranted)
        assertFalse(result.backgroundPermissionGranted)
        assertFalse(result.finePermissionRequested)
        assertTrue(result.coarsePermissionRequested)
        assertFalse(result.backgroundPermissionRequested)
    }

    @Config(sdk = [Build.VERSION_CODES.P])
    @Test
    fun requestBackgroundCoarseLocationPermission_whenCancelledAndSdkP_notifiesResultOfRequest() {
        doNothing().whenever(launcher).launch(arrayOf(Manifest.permission.ACCESS_COARSE_LOCATION))
//        justRun { launcher.launch(arrayOf(Manifest.permission.ACCESS_COARSE_LOCATION)) }

        doAnswer { invocation ->
            val permissions = emptyMap<String, Boolean>()
            val callback = invocation.getArgument<ActivityResultCallback<Map<String, Boolean>>>(1)
            callback.onActivityResult(permissions)
            return@doAnswer launcher
        }.whenever(activity).registerForActivityResult(
            any<ActivityResultContracts.RequestMultiplePermissions>(),
            any()
        )
/*        every {
            activity.registerForActivityResult(
                any<ActivityResultContracts.RequestMultiplePermissions>(),
                any()
            )
        }.answers { answer ->
            val permissions = emptyMap<String, Boolean>()

            @Suppress("UNCHECKED_CAST")
            val callback = answer.invocation.args[1] as ActivityResultCallback<Map<String, Boolean>>
            callback.onActivityResult(permissions)
            return@answers launcher
        }*/

        doNothing().whenever(listener).onLocationPermissionRequestResult(any())
//        justRun { listener.onLocationPermissionRequestResult(any()) }

        val service = LocationPermissionService(activity, listener)
        service.requestBackgroundCoarseLocationPermission()

        verify(activity, only()).registerForActivityResult(
            any<ActivityResultContracts.RequestMultiplePermissions>(),
            any()
        )
/*        verify(exactly = 1) {
            activity.registerForActivityResult(
                any<ActivityResultContracts.RequestMultiplePermissions>(),
                any()
            )
        }*/
        verify(launcher, only()).launch(arrayOf(Manifest.permission.ACCESS_COARSE_LOCATION))
//        verify(exactly = 1) { launcher.launch(arrayOf(Manifest.permission.ACCESS_COARSE_LOCATION)) }

//        val slot = slot<LocationPermissionService.LocationPermissionResult>()
        verify(listener, only()).onLocationPermissionRequestResult(capture(locationPermissionResultCaptor))
//        verify(exactly = 1) { listener.onLocationPermissionRequestResult(capture(slot)) }

        val result = locationPermissionResultCaptor.value
//        val result = slot.captured
        assertTrue(result.cancelled)
        assertFalse(result.finePermissionGranted)
        assertFalse(result.coarsePermissionGranted)
        assertFalse(result.backgroundPermissionGranted)
        assertFalse(result.finePermissionRequested)
        assertFalse(result.coarsePermissionRequested)
        assertFalse(result.backgroundPermissionRequested)
    }
}