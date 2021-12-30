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
import android.content.Context
import android.content.pm.PackageManager
import android.os.Build
import androidx.activity.result.ActivityResultCaller
import androidx.activity.result.contract.ActivityResultContracts
import androidx.appcompat.app.AppCompatActivity
import androidx.core.app.ActivityCompat
import androidx.core.content.ContextCompat

/**
 * Service to check and request location permissions.
 *
 * @property activity activity where this service is instantiated from. Might not be available if
 * this is instantiated somewhere else, such as a service. When activity is available, actions
 * requiring a UI, such as requesting permission, can be used.
 * @property context context where this service is instantiated from. This is used to check
 * permissions only.
 * @property onLocationPermissionRequestResultListener listener to notify the result of a permission
 * request.
 */
class LocationPermissionService private constructor(
    val context: Context,
    val activity: AppCompatActivity? = null,
    var onLocationPermissionRequestResultListener: OnLocationPermissionRequestResultListener? = null
) {

    /**
     * Constructor.
     * This constructor must be used when actions requiring a UI are needed, such as when requesting
     * permissions.
     *
     * @param activity activity where this service is instantiated from.
     * @param onLocationPermissionRequestResultListener listener to notify the result of a
     * permission request.
     */
    constructor(
        activity: AppCompatActivity,
        onLocationPermissionRequestResultListener: OnLocationPermissionRequestResultListener? = null
    ) : this(activity, activity, onLocationPermissionRequestResultListener)

    /**
     * Constructor.
     * This constructor can be used if only permissions must be checked but not be requested.
     *
     * @param context context where this service is instantiated from.
     * @param onLocationPermissionRequestResultListener listener to notify the result of a
     * permission request.
     */
    constructor(
        context: Context,
        onLocationPermissionRequestResultListener: OnLocationPermissionRequestResultListener? = null
    ) : this(context, null, onLocationPermissionRequestResultListener)

    /**
     * Indicates whether application has fine location permission.
     * When fine location permission is granted, coarse permission is also
     * implicitly granted.
     *
     * @return true if fine location permission is available, false otherwise.
     */
    fun hasFineLocationPermission(): Boolean {
        return hasPermission(Manifest.permission.ACCESS_FINE_LOCATION)
    }

    /**
     * Indicates whether application has coarse location permission.
     *
     * @return true if coarse location permission is available, false otherwise.
     */
    fun hasCoarseLocationPermission(): Boolean {
        return hasPermission(Manifest.permission.ACCESS_COARSE_LOCATION)
    }

    /**
     * Indicates whether application has background location permission.
     * Background location permission must be requested along with either fine or
     * coarse location permission.
     * Background location permission is only supported for SDK 29 or greater.
     * For older Android versions, whenever fine or location permission is granted,
     * background location permission is also assumed to be granted as well.
     */
    fun hasBackgroundLocationPermission(): Boolean {
        return if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.Q) {
            hasPermission(Manifest.permission.ACCESS_BACKGROUND_LOCATION)
        } else {
            hasFineLocationPermission() || hasCoarseLocationPermission()
        }
    }

    /**
     * Indicates whether a rationale should be displayed to the user indicating why
     * fine location permission is needed.
     * This is true if the user chooses not to be asked again about fine location permission
     * and denies such permission.
     *
     * @return true if a rationale should be displayed, false otherwise.
     * @throws IllegalStateException if no activity has been provided.
     */
    @Throws(IllegalStateException::class)
    fun shouldShowRequestFineLocationPermissionRationale(): Boolean {
        val activity = this.activity
        checkNotNull(activity)
        return ActivityCompat.shouldShowRequestPermissionRationale(
            activity,
            Manifest.permission.ACCESS_FINE_LOCATION
        )
    }

    /**
     * Indicates whether a rationale should be displayed to the user indicating why
     * coarse location permission is needed.
     * This is true if the user chooses not be asked again about coarse location permission
     * and denies such permission.
     *
     * @return true if a rationale should be displayed, false otherwise.
     * @throws IllegalStateException if no activity has been provided.
     */
    @Throws(IllegalStateException::class)
    fun shouldShowRequestCoarseLocationPermissionRationale(): Boolean {
        val activity = this.activity
        checkNotNull(activity)
        return ActivityCompat.shouldShowRequestPermissionRationale(
            activity,
            Manifest.permission.ACCESS_COARSE_LOCATION
        )
    }

    /**
     * Indicates whether a rationale should be displayed to the user indicating why
     * background location permission is needed.
     * This is true if the user chooses not to be asked again about background location permission
     * and denies such permission.
     *
     * @return true if a rationale should be displayed, false otherwise.
     * @throws IllegalStateException if no activity has been provided.
     */
    @Throws(IllegalStateException::class)
    fun shouldShowRequestBackgroundLocationPermissionRationale(): Boolean {
        return if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.Q) {
            val activity = this.activity
            checkNotNull(activity)
            ActivityCompat.shouldShowRequestPermissionRationale(
                activity,
                Manifest.permission.ACCESS_BACKGROUND_LOCATION
            )
        } else {
            !hasBackgroundLocationPermission()
        }
    }

    /**
     * Requests fine location permission in foreground.
     * If background location permission is also needed, then
     * [requestBackgroundFineLocationPermission] must be used instead.
     * Whenever fine location permission is granted, coarse permission is also
     * implicitly granted.
     *
     * @throws IllegalStateException if no activity has been provided.
     */
    @Throws(IllegalStateException::class)
    fun requestFineLocationPermission() {
        val activity = this.activity
        checkNotNull(activity)
        requestFineLocationPermission(activity, onLocationPermissionRequestResultListener)
    }

    /**
     * Requests coarse location permission in foreground.
     * If background location permission is also needed, then
     * [requestBackgroundCoarseLocationPermission] must be used instead.
     *
     * @throws IllegalStateException if no activity has been provided.
     */
    @Throws(IllegalStateException::class)
    fun requestCoarseLocationPermission() {
        val activity = this.activity
        checkNotNull(activity)
        requestCoarseLocationPermission(activity, onLocationPermissionRequestResultListener)
    }

    /**
     * Requests fine location permission in background.
     * Whenever fine location permission is granted, coarse permission is also
     * implicitly granted.
     * Additionally, whenever background permission is granted, foreground permission is
     * also implicitly granted.
     *
     * @throws IllegalStateException if no activity has been provided.
     */
    @Throws(IllegalStateException::class)
    fun requestBackgroundFineLocationPermission() {
        val activity = this.activity
        checkNotNull(activity)
        requestBackgroundFineLocationPermission(activity, onLocationPermissionRequestResultListener)
    }

    /**
     * Requests coarse location permission in background.
     * Whenever background permission is granted, foreground permission is also
     * implicitly granted.
     *
     * @throws IllegalStateException if no activity has been provided.
     */
    @Throws(IllegalStateException::class)
    fun requestBackgroundCoarseLocationPermission() {
        val activity = this.activity
        checkNotNull(activity)
        requestBackgroundCoarseLocationPermission(
            activity,
            onLocationPermissionRequestResultListener
        )
    }

    /**
     * Indicates whether application has provided permission.
     *
     * @return true if provided permission is available, false otherwise.
     */
    private fun hasPermission(permission: String): Boolean {
        val context = this.context
        return ContextCompat.checkSelfPermission(
            context,
            permission
        ) == PackageManager.PERMISSION_GRANTED
    }

    companion object {
        /**
         * Requests fine location permission in foreground.
         * If background location permission is also needed, then
         * [requestBackgroundFineLocationPermission] must be used instead.
         * Whenever fine location permission is granted, coarse permission is also
         * implicitly granted.
         *
         * @param caller an activity, fragment or picker that can handle activity results.
         * @param listener listener to notify the result of permission request.
         */
        fun requestFineLocationPermission(
            caller: ActivityResultCaller,
            listener: OnLocationPermissionRequestResultListener?
        ) {
            val launcher =
                caller.registerForActivityResult(ActivityResultContracts.RequestMultiplePermissions()) { permissions ->
                    val result = processPermissions(permissions)
                    listener?.onLocationPermissionRequestResult(result)
                }
            launcher.launch(arrayOf(Manifest.permission.ACCESS_FINE_LOCATION))
        }

        /**
         * Requests coarse location permission in foreground.
         * If background location permission is also needed, then
         * [requestBackgroundCoarseLocationPermission] must be used instead.
         *
         * @param caller an activity, fragment or picker that can handle activity results.
         * @param listener listener to notify the result of permission request.
         */
        fun requestCoarseLocationPermission(
            caller: ActivityResultCaller,
            listener: OnLocationPermissionRequestResultListener?
        ) {
            val launcher =
                caller.registerForActivityResult(ActivityResultContracts.RequestMultiplePermissions()) { permissions ->
                    val result = processPermissions(permissions)
                    listener?.onLocationPermissionRequestResult(result)
                }
            launcher.launch(arrayOf(Manifest.permission.ACCESS_COARSE_LOCATION))
        }

        /**
         * Requests fine location permission in background.
         * Whenever fine location permission is granted, coarse permission is also
         * implicitly granted.
         * Additionally, whenever background permission is granted, foreground permission is
         * also implicitly granted.
         *
         * @param caller an activity, fragment or picker that can handle activity results.
         * @param listener listener to notify the result of permission request.
         */
        fun requestBackgroundFineLocationPermission(
            caller: ActivityResultCaller,
            listener: OnLocationPermissionRequestResultListener?
        ) {
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.Q) {
                val launcher =
                    caller.registerForActivityResult(ActivityResultContracts.RequestMultiplePermissions()) { permissions ->
                        val result = processPermissions(permissions)
                        listener?.onLocationPermissionRequestResult(result)
                    }
                launcher.launch(
                    arrayOf(
                        Manifest.permission.ACCESS_FINE_LOCATION,
                        Manifest.permission.ACCESS_BACKGROUND_LOCATION
                    )
                )
            } else {
                requestFineLocationPermission(caller, listener)
            }
        }

        /**
         * Requests coarse location permission in background.
         * Whenever background permission is granted, foreground permission is also
         * implicitly granted.
         *
         * @param caller an activity, fragment or picker that can handle activity results.
         * @param listener listener to notify the result of permission request.
         */
        fun requestBackgroundCoarseLocationPermission(
            caller: ActivityResultCaller,
            listener: OnLocationPermissionRequestResultListener?
        ) {
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.Q) {
                val launcher =
                    caller.registerForActivityResult(ActivityResultContracts.RequestMultiplePermissions()) { permissions ->
                        val result = processPermissions(permissions)
                        listener?.onLocationPermissionRequestResult(result)
                    }
                launcher.launch(
                    arrayOf(
                        Manifest.permission.ACCESS_COARSE_LOCATION,
                        Manifest.permission.ACCESS_BACKGROUND_LOCATION
                    )
                )
            } else {
                requestCoarseLocationPermission(caller, listener)
            }
        }

        /**
         * Processes received permissions.
         *
         * @param permissions permission request result. Contains each requested permission and
         * whether it was granted or not. If map is empty it means that permission request was
         * cancelled by the user.
         * @return an instance indicating which permission was requested and which one was granted.
         */
        private fun processPermissions(permissions: Map<String, Boolean>): LocationPermissionResult {
            if (permissions.isEmpty()) {
                return LocationPermissionResult(cancelled = true)
            } else {
                var finePermissionRequested = false
                var finePermissionGranted = false
                var coarsePermissionRequested = false
                var coarsePermissionGranted = false
                var backgroundPermissionRequested = false
                var backgroundPermissionGranted = false
                for ((key, value) in permissions) {
                    when (key) {
                        Manifest.permission.ACCESS_FINE_LOCATION -> {
                            finePermissionRequested = true
                            finePermissionGranted = value
                        }
                        Manifest.permission.ACCESS_COARSE_LOCATION -> {
                            coarsePermissionRequested = true
                            coarsePermissionGranted = value
                        }
                        Manifest.permission.ACCESS_BACKGROUND_LOCATION -> {
                            backgroundPermissionRequested = true
                            backgroundPermissionGranted = value
                        }
                    }
                }

                return LocationPermissionResult(
                    cancelled = false,
                    finePermissionGranted,
                    coarsePermissionGranted,
                    backgroundPermissionGranted,
                    finePermissionRequested,
                    coarsePermissionRequested,
                    backgroundPermissionRequested
                )
            }
        }
    }

    /**
     * Listener to notify the result of permission request.
     */
    interface OnLocationPermissionRequestResultListener {
        /**
         * Called when a permission request result is received.
         *
         * @param result result of location permission request.
         */
        fun onLocationPermissionRequestResult(result: LocationPermissionResult)
    }

    /**
     * Contains information about the result of location permission request.
     *
     * @property cancelled true indicates that permission request was cancelled by the user.
     * @property finePermissionGranted true indicates that fine location permission was granted.
     * When this is true, coarse permission is also implicitly granted.
     * @property coarsePermissionGranted true indicates that coarse location permission was granted.
     * @property backgroundPermissionGranted true indicates that background location permission was
     * granted, either for coarse or fine location. When true, foreground location permission is
     * also implicitly granted.
     * @property finePermissionRequested true indicates that fine location permission was requested
     * to the user.
     * @property coarsePermissionRequested true indicates that coarse location permission was
     * requested to the user.
     * @property backgroundPermissionRequested true indicates that background location permission
     * was requested to the user.
     */
    data class LocationPermissionResult(
        val cancelled: Boolean = true,
        val finePermissionGranted: Boolean = false,
        val coarsePermissionGranted: Boolean = false,
        val backgroundPermissionGranted: Boolean = false,
        val finePermissionRequested: Boolean = false,
        val coarsePermissionRequested: Boolean = false,
        val backgroundPermissionRequested: Boolean = false
    )
}