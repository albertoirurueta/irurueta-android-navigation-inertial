package com.irurueta.android.navigation.inertial.test

import androidx.test.core.app.ActivityScenario
import androidx.test.platform.app.InstrumentationRegistry
import androidx.test.rule.GrantPermissionRule
import androidx.test.runner.lifecycle.ActivityLifecycleCallback
import androidx.test.runner.lifecycle.ActivityLifecycleMonitor
import androidx.test.runner.lifecycle.ActivityLifecycleMonitorRegistry
import androidx.test.runner.lifecycle.Stage
import com.irurueta.android.navigation.inertial.LocationPermissionService
import io.mockk.Called
import io.mockk.mockk
import io.mockk.verify
import org.junit.Assert.*
import org.junit.Rule
import org.junit.Test

class LocationPermissionServiceTest {

    @get:Rule
    val permissionRule: GrantPermissionRule = GrantPermissionRule.grant(
        android.Manifest.permission.ACCESS_COARSE_LOCATION,
        android.Manifest.permission.ACCESS_FINE_LOCATION,
        android.Manifest.permission.ACCESS_BACKGROUND_LOCATION
    )

    private val registry: ActivityLifecycleMonitor = ActivityLifecycleMonitorRegistry.getInstance()

    @Test
    fun hasFineLocationPermission_returnsExpectedValue() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val service = LocationPermissionService(context)

        assertTrue(service.hasFineLocationPermission())
    }

    @Test
    fun hasCoarseLocationPermission_returnsExpectedValue() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val service = LocationPermissionService(context)

        assertTrue(service.hasCoarseLocationPermission())
    }

    @Test
    fun hasBackgroundLocationPermission_returnsExpectedValue() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val service = LocationPermissionService(context)

        assertTrue(service.hasBackgroundLocationPermission())
    }

    @Test
    fun shouldShowRequestFineLocationPermissionRationale_returnsExpectedValue() {
        val scenario = ActivityScenario.launch(LocationPermissionServiceActivity::class.java).use {
            it.onActivity { activity ->
                val service = LocationPermissionService(activity)

                assertFalse(service.shouldShowRequestFineLocationPermissionRationale())
            }
        }
        assertNotNull(scenario)
    }

    @Test
    fun shouldShowRequestCoarseLocationPermissionRationale_returnsExpectedValue() {
        val scenario = ActivityScenario.launch(LocationPermissionServiceActivity::class.java).use {
            it.onActivity { activity ->
                val service = LocationPermissionService(activity)

                assertFalse(service.shouldShowRequestCoarseLocationPermissionRationale())
            }
        }
        assertNotNull(scenario)
    }

    @Test
    fun shouldShowRequestBackgroundLocationPermissionRationale_returnsExpectedValue() {
        val scenario = ActivityScenario.launch(LocationPermissionServiceActivity::class.java).use {
            it.onActivity { activity ->
                val service = LocationPermissionService(activity)

                assertFalse(service.shouldShowRequestBackgroundLocationPermissionRationale())
            }
        }
        assertNotNull(scenario)
    }

    @Test
    fun requestFineLocationPermission_whenPermissionAlreadyGranted_makesNoAction() {
        val listener = mockk<LocationPermissionService.OnLocationPermissionRequestResultListener>()
        val callback = ActivityLifecycleCallback { activity, stage ->
            if (stage == Stage.CREATED) {
                requireNotNull(activity)
                val service = LocationPermissionService(activity, listener)

                assertTrue(service.hasFineLocationPermission())
                service.requestFineLocationPermission()

                verify { listener wasNot Called }
            }
        }
        registry.addLifecycleCallback(callback)

        val scenario = ActivityScenario.launch(LocationPermissionServiceActivity::class.java).use {
            it.onActivity { activity ->
                registry.removeLifecycleCallback(callback)
                val service = LocationPermissionService(activity, listener)
                assertTrue(service.hasFineLocationPermission())
            }
        }
        assertNotNull(scenario)
    }

    @Test
    fun requestCoarseLocationPermission_whenPermissionAlreadyGranted_makesNoAction() {
        val listener = mockk<LocationPermissionService.OnLocationPermissionRequestResultListener>()
        val callback = ActivityLifecycleCallback { activity, stage ->
            if (stage == Stage.CREATED) {
                requireNotNull(activity)
                val service = LocationPermissionService(activity, listener)

                assertTrue(service.hasCoarseLocationPermission())
                service.requestCoarseLocationPermission()

                verify { listener wasNot Called }
            }
        }
        registry.addLifecycleCallback(callback)

        val scenario = ActivityScenario.launch(LocationPermissionServiceActivity::class.java).use {
            it.onActivity { activity ->
                registry.removeLifecycleCallback(callback)
                val service = LocationPermissionService(activity, listener)
                assertTrue(service.hasCoarseLocationPermission())
            }
        }
        assertNotNull(scenario)
    }

    @Test
    fun requestBackgroundFineLocationPermission_whenPermissionAlreadyGranted_makesNoAction() {
        val listener = mockk<LocationPermissionService.OnLocationPermissionRequestResultListener>()
        val callback = ActivityLifecycleCallback { activity, stage ->
            if (stage == Stage.CREATED) {
                requireNotNull(activity)
                val service = LocationPermissionService(activity, listener)

                assertTrue(service.hasFineLocationPermission())
                assertTrue(service.hasBackgroundLocationPermission())
                service.requestBackgroundFineLocationPermission()

                verify { listener wasNot Called }
            }
        }
        registry.addLifecycleCallback(callback)

        val scenario = ActivityScenario.launch(LocationPermissionServiceActivity::class.java).use {
            it.onActivity { activity ->
                registry.removeLifecycleCallback(callback)
                val service = LocationPermissionService(activity, listener)
                assertTrue(service.hasFineLocationPermission())
                assertTrue(service.hasBackgroundLocationPermission())
            }
        }
        assertNotNull(scenario)
    }

    @Test
    fun requestBackgroundCoarseLocationPermission_whenPermissionAlreadyGranted_makesNoAction() {
        val listener = mockk<LocationPermissionService.OnLocationPermissionRequestResultListener>()
        val callback = ActivityLifecycleCallback { activity, stage ->
            if (stage == Stage.CREATED) {
                requireNotNull(activity)
                val service = LocationPermissionService(activity, listener)

                assertTrue(service.hasCoarseLocationPermission())
                assertTrue(service.hasBackgroundLocationPermission())
                service.requestBackgroundCoarseLocationPermission()

                verify { listener wasNot Called }
            }
        }
        registry.addLifecycleCallback(callback)

        val scenario = ActivityScenario.launch(LocationPermissionServiceActivity::class.java).use {
            it.onActivity { activity ->
                registry.removeLifecycleCallback(callback)
                val service = LocationPermissionService(activity, listener)
                assertTrue(service.hasCoarseLocationPermission())
                assertTrue(service.hasBackgroundLocationPermission())
            }
        }
        assertNotNull(scenario)
    }
}