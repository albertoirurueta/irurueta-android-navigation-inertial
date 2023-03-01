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
import android.os.Build
import android.os.Bundle
import androidx.appcompat.app.AppCompatActivity
import androidx.appcompat.widget.AppCompatTextView
import com.irurueta.android.gl.cube.CubeRenderer
import com.irurueta.android.gl.cube.CubeTextureView
import com.irurueta.android.navigation.inertial.collectors.AccelerometerSensorType
import com.irurueta.android.navigation.inertial.collectors.SensorDelay
import com.irurueta.android.navigation.inertial.estimators.attitude.AccurateLevelingEstimator2
import com.irurueta.android.navigation.inertial.estimators.filter.AveragingFilter
import com.irurueta.android.navigation.inertial.estimators.filter.LowPassAveragingFilter
import com.irurueta.android.navigation.inertial.estimators.filter.MeanAveragingFilter
import com.irurueta.android.navigation.inertial.estimators.filter.MedianAveragingFilter
import com.irurueta.geometry.*

class AccurateLevelingEstimatorActivity : AppCompatActivity() {

    private var cubeView: CubeTextureView? = null

    private var rollView: AppCompatTextView? = null

    private var pitchView: AppCompatTextView? = null

    private var rotation = Quaternion()

    private var camera: PinholeCamera? = null

    private var levelingEstimator: AccurateLevelingEstimator2? = null

    private var hasLocationPermission = false

    private var location: Location? = null

    private val locationPermissionService = LocationPermissionService(this) { locationResult ->
        hasLocationPermission = locationResult.finePermissionGranted
        checkLocationAndStart()
    }

    private var locationService: LocationService? = null

    private var useAccelerometer = false

    private var accelerometerSensorType =
        AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED

    private var averagingFilterType: String? = null

    private val displayOrientation = Quaternion()

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)

        val extras = intent.extras
        useAccelerometer = extras?.getBoolean(USE_ACCELEROMETER, false) ?: false
        accelerometerSensorType = if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
            (extras?.getSerializable(ACCELEROMETER_SENSOR_TYPE, AccelerometerSensorType::class.java)
                ?: AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED)
        } else {
            @Suppress("DEPRECATION")
            (extras?.getSerializable(ACCELEROMETER_SENSOR_TYPE) as AccelerometerSensorType?)
                ?: AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED
        }
        averagingFilterType = extras?.getString(AVERAGING_FILTER_TYPE)

        setContentView(R.layout.activity_accurate_leveling_estimator)
        cubeView = findViewById(R.id.cube)
        rollView = findViewById(R.id.roll)
        pitchView = findViewById(R.id.pitch)

        val cubeView = cubeView ?: return
        cubeView.onSurfaceChangedListener = object : CubeTextureView.OnSurfaceChangedListener {
            override fun onSurfaceChanged(width: Int, height: Int) {
                cubeView.cubeSize = 0.5f * CubeRenderer.DEFAULT_CUBE_SIZE
                cubeView.cubeRotation = rotation
                camera = createCamera(cubeView)
                cubeView.camera = camera
            }
        }

        if (locationPermissionService.hasFineLocationPermission()) {
            hasLocationPermission = true
        } else {
            locationPermissionService.requestFineLocationPermission()
        }
    }

    override fun onResume() {
        super.onResume()
        cubeView?.onResume()
        checkLocationAndStart()
    }

    override fun onPause() {
        super.onPause()
        levelingEstimator?.stop()
        cubeView?.onPause()
    }

    @SuppressLint("MissingPermission")
    private fun checkLocationAndStart() {
        if (!hasLocationPermission) {
            return
        }

        val location = this.location

        if (location != null) {
            buildEstimatorAndStart()
        } else {
            locationService = LocationService(this)
            val lastLocation = locationService?.getLastKnownLocation()
            if (lastLocation != null) {
                this.location = lastLocation
                buildEstimatorAndStart()
            } else {
                locationService?.getCurrentLocation { currentLocation ->
                    this@AccurateLevelingEstimatorActivity.location = currentLocation
                    buildEstimatorAndStart()
                }
            }
        }
    }

    private fun buildEstimatorAndStart() {
        val location = this.location ?: return
        if (levelingEstimator?.running == true) return

        if (levelingEstimator != null) {
            levelingEstimator?.location = location
        } else {
            val averagingFilter = buildAveragingFilter(averagingFilterType)

            levelingEstimator = AccurateLevelingEstimator2(
                this,
                location,
                SensorDelay.GAME,
                useAccelerometer = useAccelerometer,
                accelerometerSensorType = accelerometerSensorType,
                accelerometerAveragingFilter = averagingFilter,
                estimateCoordinateTransformation = false,
                estimateEulerAngles = true,
                levelingAvailableListener = { _, attitude, _, roll, pitch, _ ->
                    attitude.toQuaternion(rotation)

                    rollView?.text = getString(R.string.roll_degrees, Math.toDegrees(roll ?: 0.0))
                    pitchView?.text =
                        getString(R.string.pitch_degrees, Math.toDegrees(pitch ?: 0.0))

                    // rotation refers to pinhole camera point of view, to apply rotation to the cube
                    // its inverse must be used.
                    rotation.inverse()

                    // convert attitude from NED to ENU coordinate system to be displayed using OpenGL
                    ENUtoNEDConverter.convert(rotation, rotation)

                    // take into account display orientation
                    val displayRotationRadians =
                        DisplayOrientationHelper.getDisplayRotationRadians(this)
                    displayOrientation.setFromEulerAngles(0.0, 0.0, displayRotationRadians)
                    Quaternion.product(displayOrientation, rotation, rotation)

                    cubeView?.cubeRotation = rotation
                }
            )
        }
        levelingEstimator?.start()
    }

    private fun buildAveragingFilter(averagingFilterType: String?): AveragingFilter {
        return when (averagingFilterType) {
            MEAN_AVERAGING_FILTER -> MeanAveragingFilter()
            MEDIAN_AVERAGING_FILTER -> MedianAveragingFilter()
            else -> LowPassAveragingFilter()
        }
    }

    companion object {
        const val USE_ACCELEROMETER = "useAccelerometer"
        const val ACCELEROMETER_SENSOR_TYPE = "accelerometerSensorType"
        const val AVERAGING_FILTER_TYPE = "averagingFilterType"

        const val LOW_PASS_AVERAGING_FILTER = "losPassAveragingFilter"
        const val MEAN_AVERAGING_FILTER = "meanAveragingFilter"
        const val MEDIAN_AVERAGING_FILTER = "medianAveragingFilter"

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