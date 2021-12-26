package com.irurueta.android.navigation

import android.Manifest.permission
import android.content.Context
import android.location.Location
import android.location.LocationListener
import android.location.LocationManager
import android.os.Build
import android.os.CancellationSignal
import android.os.Looper
import androidx.annotation.RequiresPermission
import com.google.android.gms.common.ConnectionResult
import com.google.android.gms.common.GoogleApiAvailability
import com.google.android.gms.location.*
import com.google.android.gms.tasks.CancellationTokenSource
import java.util.concurrent.Executors

/**
 * Service to obtain device location.
 *
 * @property context context where this service is instantiated from.
 */
class LocationService(val context: Context) {

    /**
     * Location provider used by Google Play Services using a fused provider.
     * This provider should be more accurate than the system provider.
     * Only available when device supports Google Play Services.
     */
    private var fusedLocationClient: FusedLocationProviderClient? = null

    /**
     * Location manager.
     * This is the location provider used by the system.
     */
    private var locationManager: LocationManager? = null

    /**
     * Source to obtain tokens to cancel single current location updates when using Google Play
     * Services location provider.
     */
    private var cancellationTokenSource: CancellationTokenSource? = null

    /**
     * Signal to cancel single current location updates when using system location provider.
     */
    private var cancellationSignal: CancellationSignal? = null

    /**
     * Internal callback to receive location updates from Google Play Services location provider.
     */
    private var updatesCallback = object : LocationCallback() {
        /**
         * Called when a location update is received.
         *
         * @param result obtained location result.
         */
        override fun onLocationResult(result: LocationResult) {
            locationUpdateListener?.onLocationChanged(result.lastLocation)
        }

        /**
         * Called when availability of location services changes.
         *
         * @param availability new availability of location services.
         */
        override fun onLocationAvailability(availability: LocationAvailability) {
            locationUpdateListener?.onLocationAvailability(availability.isLocationAvailable)
        }

    }

    /**
     * Internal listener to receive location updates when using system location provider.
     */
    private var updatesListener =
        LocationListener { location -> locationUpdateListener?.onLocationChanged(location) }

    /**
     * Gets status for Google Play Services.
     */
    val googlePlayServicesStatus: GooglePlayStatus?
        get() = GooglePlayStatus.from(
            GoogleApiAvailability.getInstance().isGooglePlayServicesAvailable(context)
        )

    /**
     * Indicates whether Google Play Services is available.
     * More detailed information can be obtained at [googlePlayServicesStatus].
     */
    val googlePlayServicesAvailable
        get() = googlePlayServicesStatus == GooglePlayStatus.SUCCESS

    /**
     * Indicates whether location is enabled on current device.
     * True indicates that location is enabled, false indicates that location is disabled,
     * null indicates that status is unknown.
     * Notice that location status is only available for SDK 28 or later.
     */
    val locationEnabled: Boolean?
        get() {
            return if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.P) {
                locationManager?.isLocationEnabled
            } else {
                null
            }
        }

    /**
     * Gets last known location.
     * This method attempts to retrieve last cached device location, if available.
     * If no previous location is known, null is returned.
     * This method does not attempt to retrieve a new location if no location is known.
     * Location permission is required
     * @see [permission.ACCESS_COARSE_LOCATION]
     * @see [permission.ACCESS_FINE_LOCATION]
     */
    @RequiresPermission(anyOf = [permission.ACCESS_COARSE_LOCATION, permission.ACCESS_FINE_LOCATION])
    fun getLastKnownLocation(): Location? {
        val locationManager = locationManager
        return if (locationManager?.isProviderEnabled(FUSED_PROVIDER) == true) {
            locationManager.getLastKnownLocation(FUSED_PROVIDER)
        } else {
            null
        }
    }

    /**
     * Requests a single location update.
     * Returns a cached last known location if it was recently cached or waits until a new location
     * is received if none is already available.
     * Because this method might use cached locations, returned value in callback may not be the
     * most recent one.
     * This method returns immediately and notifies in provided listener when location is found.
     * Current location request can be cancelled by calling [cancelCurrentLocation].
     * Once the request is cancelled, provider listener is no longer called.
     * There are no guarantees on which thread the listener will be called on. This will depend on
     * Android version on the device and whether Google Play Services location provider is available
     * or not.
     */
    @RequiresPermission(anyOf = [permission.ACCESS_COARSE_LOCATION, permission.ACCESS_FINE_LOCATION])
    fun getCurrentLocation(listener: OnCurrentLocationListener) {
        val fusedLocationClient = this.fusedLocationClient
        val cancellationTokenSource = this.cancellationTokenSource
        if (fusedLocationClient != null && cancellationTokenSource != null) {
            val token = cancellationTokenSource.token
            fusedLocationClient.getCurrentLocation(LocationRequest.PRIORITY_HIGH_ACCURACY, token)
                .addOnSuccessListener { location ->
                    listener.onCurrentLocation(location)
                }
        } else {
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.R) {
                // cancel previous request if any
                cancellationSignal?.cancel()

                // request a new location update
                cancellationSignal = CancellationSignal()
                locationManager?.getCurrentLocation(
                    FUSED_PROVIDER,
                    cancellationSignal,
                    Executors.newCachedThreadPool(),
                    { location ->
                        cancellationSignal = null
                        listener.onCurrentLocation(location)
                    })
            } else {
                @Suppress("DEPRECATION")
                locationManager?.requestSingleUpdate(FUSED_PROVIDER, { location ->
                    listener.onCurrentLocation(location)
                }, getLooper())
            }
        }
    }

    /**
     * Cancels current location request if there is an ongoing request, otherwise makes no action.
     */
    fun cancelCurrentLocation() {
        cancellationTokenSource?.cancel()
        cancellationSignal?.cancel()

        cancellationSignal = null
    }

    /**
     * Minimum distance between location updates in meters.
     * By default this is zero meters.
     * By modifying this value, the frequency to request location updates changes, and consequently
     * so does battery usage.
     * A reasonable value must be used for each use case.
     * Changing this value while updates are already being requested has no effect until location
     * updates are cancelled by calling [cancelLocationUpdates] and new updates are requested again.
     *
     * @throws IllegalArgumentException if provided value is negative
     */
    var smallestDisplacement = DEFAULT_SMALLEST_DISPLACEMENT
        @Throws(IllegalArgumentException::class)
        set(value) {
            require(value >= 0.0f)
            field = value
        }

    /**
     * Minimum time interval between location updates in milliseconds.
     * By default this is one second.
     * By modifying this value, the frequency to request location updates changes, and consequently
     * so does battery usage.
     * A reasonable value must be used for each use case.
     * Changing this value while updates are already being requested has no effect until location
     * updates are cancelled by calling [cancelLocationUpdates] and new updates are requested again.
     *
     * @throws IllegalArgumentException if provided value is negative
     */
    var updateInterval = DEFAULT_UPDATE_INTERVAL
        @Throws(IllegalArgumentException::class)
        set(value) {
            require(value >= 0L)
            field = value
        }

    /**
     * Listener to be notified of location updates.
     */
    var locationUpdateListener: OnLocationUpdateListener? = null

    /**
     * Requests continuous location updates for current [updateInterval] and [smallestDisplacement].
     * By modifying both [updateInterval] and [smallestDisplacement], the frequency to request
     * location updates changes, and consequently so does battery usage.
     * Reasonable values must be used for each use case.
     * Current [locationUpdateListener] will be notified until [cancelLocationUpdates] is called.
     */
    @RequiresPermission(anyOf = [permission.ACCESS_COARSE_LOCATION, permission.ACCESS_FINE_LOCATION])
    fun requestLocationUpdates() {
        val fusedLocationClient = this.fusedLocationClient
        if (fusedLocationClient != null) {
            fusedLocationClient.requestLocationUpdates(
                LocationRequest.create().setSmallestDisplacement(smallestDisplacement)
                    .setInterval(updateInterval),
                updatesCallback,
                getLooper()
            )
        } else {
            locationManager?.removeUpdates(updatesListener)
            locationManager?.requestLocationUpdates(
                FUSED_PROVIDER,
                updateInterval,
                smallestDisplacement,
                updatesListener,
                getLooper()
            )
        }
    }

    /**
     * Cancels periodic location updates.
     * This should be called whenever periodic location updates are no longer needed to preserve
     * battery life.
     */
    fun cancelLocationUpdates() {
        fusedLocationClient?.removeLocationUpdates(updatesCallback)
        locationManager?.removeUpdates(updatesListener)
    }

    /**
     * Initializes internal location clients.
     */
    init {
        locationManager = context.getSystemService(Context.LOCATION_SERVICE) as LocationManager?
        if (googlePlayServicesAvailable) {
            fusedLocationClient = LocationServices.getFusedLocationProviderClient(context)
            cancellationTokenSource = CancellationTokenSource()
        }
    }

    companion object {
        /**
         * Name of fused provider
         */
        const val FUSED_PROVIDER = "fused"

        /**
         * Default minimum displacement expressed in meters to be taken into account to request
         * new location updates.
         * This value is used to request as many location updates as possible for current update
         * interval.
         */
        const val DEFAULT_SMALLEST_DISPLACEMENT = 0.0f

        /**
         * Default update interval expressed in milliseconds to request location updates.
         */
        const val DEFAULT_UPDATE_INTERVAL = 1000L

        /**
         * Gets a looper.
         * First a looper is attempted from current thread, if not available, main thread looper is
         * used instead.
         */
        private fun getLooper(): Looper {
            return Looper.myLooper() ?: Looper.getMainLooper()
        }
    }

    /**
     * Google Play Services status.
     *
     * @param value numerical value representing status.
     */
    enum class GooglePlayStatus(val value: Int) {
        /**
         * The connection was successful.
         */
        SUCCESS(ConnectionResult.SUCCESS),

        /**
         * Google Play services is missing on this device.
         */
        SERVICE_MISSING(ConnectionResult.SERVICE_MISSING),

        /**
         * Google Play service is currently being updated on this device.
         */
        SERVICE_UPDATING(ConnectionResult.SERVICE_UPDATING),

        /**
         * The installed version of Google Play services is out of date.
         */
        SERVICE_VERSION_UPDATE_REQUIRED(ConnectionResult.SERVICE_VERSION_UPDATE_REQUIRED),

        /**
         * The installed version of Google Play services has been disabled on this device.
         */
        SERVICE_DISABLED(ConnectionResult.SERVICE_DISABLED),

        /**
         * The version of Google Play services installed on the device is not authentic.
         */
        SERVICE_INVALID(ConnectionResult.SERVICE_INVALID);

        companion object {
            /**
             * Gets the Google Play services status based on provided code.
             *
             * @param value code returned by Google Play services.
             * @return code expressed as an enum or null if code has no match.
             */
            fun from(value: Int): GooglePlayStatus? {
                return values().find { it.value == value }
            }
        }
    }

    /**
     * Listener to notify current location.
     * When current location is requested using [getCurrentLocation], method returns immediately
     * and listener might be called later if no cached location is readily available, in such cases
     * system must wait until a new location update is received.
     */
    interface OnCurrentLocationListener {
        /**
         * Called when new location is received or a recent cached location is available.
         *
         * @param location current location.
         */
        fun onCurrentLocation(location: Location)
    }

    /**
     * Listener to notify periodic location updates.
     */
    interface OnLocationUpdateListener {
        /**
         * Called when a new location update is received.
         *
         * @param location updated location.
         */
        fun onLocationChanged(location: Location)

        /**
         * Called when location availability changes.
         * This is only called when Google Play Services location provider is used when Google Play
         * Services is available.
         *
         * @param available true if location services become available, false if they become
         * unavailable.
         */
        fun onLocationAvailability(available: Boolean)
    }
}