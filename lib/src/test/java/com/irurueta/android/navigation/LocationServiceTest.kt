package com.irurueta.android.navigation

import android.content.Context
import android.location.Location
import android.location.LocationManager
import android.os.Build
import androidx.test.core.app.ApplicationProvider
import com.google.android.gms.common.ConnectionResult
import com.google.android.gms.common.GoogleApiAvailability
import io.mockk.*
import org.junit.Assert.*
import org.junit.Test
import org.junit.runner.RunWith
import org.robolectric.RobolectricTestRunner
import org.robolectric.annotation.Config

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
    fun locationEnabled_whenSdkP_returnsExpectedValue() {
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

    @Config(sdk = [Build.VERSION_CODES.O])
    @Test
    fun locationEnabled_whenSdkO_returnsNull() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val service = LocationService(context)

        assertNull(service.locationEnabled)
    }

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