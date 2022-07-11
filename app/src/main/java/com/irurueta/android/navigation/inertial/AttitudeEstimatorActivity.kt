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
import android.location.Location
import android.os.Bundle
import androidx.appcompat.app.AppCompatActivity
import androidx.appcompat.widget.AppCompatTextView
import com.irurueta.android.gl.cube.CubeRenderer
import com.irurueta.android.gl.cube.CubeTextureView
import com.irurueta.android.navigation.inertial.estimators.AttitudeEstimator
import com.irurueta.geometry.*

class AttitudeEstimatorActivity : AppCompatActivity() {
    private var cubeView: CubeTextureView? = null

    private var rollView: AppCompatTextView? = null

    private var pitchView: AppCompatTextView? = null

    private var yawView: AppCompatTextView? = null

    private var rotation = Quaternion()

    private var eulerAngles = DoubleArray(3)

    private var camera: PinholeCamera? = null

    private var attitudeEstimator: AttitudeEstimator? = null

    private var hasLocationPermission = false

    private var estimatorType: AttitudeEstimator.AttitudeEstimatorType? = null

    private var location: Location? = null

    private val locationPermissionService = LocationPermissionService(this) { locationResult ->
        hasLocationPermission =
            locationResult.finePermissionGranted || locationResult.coarsePermissionGranted || locationResult.backgroundPermissionGranted
    }

    private var locationService: LocationService? = null

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)

        val extras = intent.extras
        val serializable =
            extras?.getSerializable(ESTIMATOR_TYPE) as AttitudeEstimator.AttitudeEstimatorType?
        estimatorType = serializable ?: AttitudeEstimator.AttitudeEstimatorType.LEVELING
        setContentView(R.layout.activity_attitude_estimator)
        cubeView = findViewById(R.id.cube)
        rollView = findViewById(R.id.roll)
        pitchView = findViewById(R.id.pitch)
        yawView = findViewById(R.id.yaw)

        val cubeView = cubeView ?: return
        cubeView.onSurfaceChangedListener = object : CubeTextureView.OnSurfaceChangedListener {
            override fun onSurfaceChanged(width: Int, height: Int) {
                cubeView.cubeSize = 0.5f * CubeRenderer.DEFAULT_CUBE_SIZE
                cubeView.cubeRotation = rotation
                camera = createCamera(cubeView)
                cubeView.camera = camera
            }
        }

        val estimateImuLeveling =
            estimatorType == AttitudeEstimator.AttitudeEstimatorType.LEVELING || estimatorType == AttitudeEstimator.AttitudeEstimatorType.IMPROVED_LEVELING
        attitudeEstimator = AttitudeEstimator(
            this,
            estimateImuLeveling = estimateImuLeveling
        ) { attitude, _, _ ->
            attitude.toQuaternion(rotation)
            cubeView.cubeRotation = rotation

            rotation.toEulerAngles(eulerAngles)
            rollView?.text = getString(R.string.roll_degrees, -Math.toDegrees(eulerAngles[0]))
            pitchView?.text = getString(R.string.pitch_degrees, -Math.toDegrees(eulerAngles[1]))
            yawView?.text = getString(R.string.yaw_degrees, -Math.toDegrees(eulerAngles[2]))
        }

        if (estimatorType != AttitudeEstimator.AttitudeEstimatorType.LEVELING) {
            // check location permission
            if (locationPermissionService.hasFineLocationPermission()) {
                hasLocationPermission = true
            } else {
                locationPermissionService.requestFineLocationPermission()
            }
        }

        locationService = LocationService(this)
    }

    override fun onResume() {
        super.onResume()
        cubeView?.onResume()
        checkLocationAndStart()
    }

    override fun onPause() {
        super.onPause()
        attitudeEstimator?.stop()
        cubeView?.onPause()
    }

    @SuppressLint("MissingPermission")
    private fun checkLocationAndStart() {
        // leveling does not require location
        if (estimatorType == AttitudeEstimator.AttitudeEstimatorType.LEVELING) {
            attitudeEstimator?.start()
            return
        }

        // for all other cases location is needed
        if (!locationPermissionService.hasFineLocationPermission()) {
            return
        }

        val lastLocation = locationService?.getLastKnownLocation()
        if (lastLocation != null) {
            location = lastLocation
            attitudeEstimator?.location = location
        } else {
            locationService?.getCurrentLocation { location ->
                this@AttitudeEstimatorActivity.location = location
                attitudeEstimator?.start()
            }
        }

        if (location != null) {
            attitudeEstimator?.start()
        }
    }

    companion object {
        const val ESTIMATOR_TYPE = "estimatorType"

        val LEVELING = AttitudeEstimator.AttitudeEstimatorType.LEVELING

        val IMPROVED_LEVELING = AttitudeEstimator.AttitudeEstimatorType.IMPROVED_LEVELING

        val GEOMAGNETIC = AttitudeEstimator.AttitudeEstimatorType.GEOMAGNETIC

        private const val HORIZONTAL_FOCAL_LENGTH = 2065.920810699463

        private const val VERTICAL_FOCAL_LENGTH = 2125.98623752594

        private fun createCamera(view: CubeTextureView): PinholeCamera {
            val px = view.width / 2.0
            val py = view.height / 2.0

            val intrinsics = PinholeCameraIntrinsicParameters(
                HORIZONTAL_FOCAL_LENGTH,
                VERTICAL_FOCAL_LENGTH,
                px,
                py,
                0.0
            )

            return PinholeCamera(intrinsics, Rotation3D.create(), Point3D.create())
        }
    }
}