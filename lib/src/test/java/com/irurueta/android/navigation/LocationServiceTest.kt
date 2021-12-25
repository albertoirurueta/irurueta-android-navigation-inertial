package com.irurueta.android.navigation

import android.content.Context
import androidx.test.core.app.ApplicationProvider
import com.google.android.gms.common.ConnectionResult
import org.junit.Assert.*
import org.junit.Test
import org.junit.runner.RunWith
import org.robolectric.RobolectricTestRunner

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
}