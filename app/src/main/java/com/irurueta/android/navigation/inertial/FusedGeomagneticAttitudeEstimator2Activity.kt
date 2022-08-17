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
import com.irurueta.android.navigation.inertial.collectors.AccelerometerSensorCollector
import com.irurueta.android.navigation.inertial.collectors.GyroscopeSensorCollector
import com.irurueta.android.navigation.inertial.collectors.MagnetometerSensorCollector
import com.irurueta.android.navigation.inertial.collectors.SensorDelay
import com.irurueta.android.navigation.inertial.estimators.FusedGeomagneticAttitudeEstimator2
import com.irurueta.android.navigation.inertial.estimators.filter.AveragingFilter
import com.irurueta.android.navigation.inertial.estimators.filter.LowPassAveragingFilter
import com.irurueta.android.navigation.inertial.estimators.filter.MeanAveragingFilter
import com.irurueta.android.navigation.inertial.estimators.filter.MedianAveragingFilter
import com.irurueta.geometry.*

class FusedGeomagneticAttitudeEstimator2Activity : AppCompatActivity() {

    private var cubeView: CubeTextureView? = null

    private var rollView: AppCompatTextView? = null

    private var pitchView: AppCompatTextView? = null

    private var yawView: AppCompatTextView? = null

    private var rotation = Quaternion()

    private var eulerAngles = DoubleArray(3)

    private var camera: PinholeCamera? = null

    private var attitudeEstimator: FusedGeomagneticAttitudeEstimator2? = null

    private var hasLocationPermission = false

    private var location: Location? = null

    private val locationPermissionService = LocationPermissionService(this) { locationResult ->
        hasLocationPermission = locationResult.finePermissionGranted
        checkLocationAndStart()
    }

    private var locationService: LocationService? = null

    private var useAccelerometer = false

    private var useWorldMagneticModel = false

    private var accelerometerSensorType = AccelerometerSensorCollector.SensorType.ACCELEROMETER

    private var magnetometerSensorType = MagnetometerSensorCollector.SensorType.MAGNETOMETER

    private var accelerometerAveragingFilterType: String? = null

    private var gyroscopeSensorType = GyroscopeSensorCollector.SensorType.GYROSCOPE

    private var useAccurateLevelingEstimator = false

    private var useAccurateRelativeGyroscopeAttitudeEstimator = false

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)

        val extras = intent.extras
        useAccelerometer =
            extras?.getBoolean(USE_ACCELEROMETER, false)
                ?: false
        accelerometerSensorType =
            (extras?.getSerializable(ACCELEROMETER_SENSOR_TYPE) as AccelerometerSensorCollector.SensorType?)
                ?: AccelerometerSensorCollector.SensorType.ACCELEROMETER
        magnetometerSensorType =
            (extras?.getSerializable(GeomagneticAttitudeEstimatorActivity.MAGNETOMETER_SENSOR_TYPE) as MagnetometerSensorCollector.SensorType?)
                ?: MagnetometerSensorCollector.SensorType.MAGNETOMETER
        accelerometerAveragingFilterType =
            extras?.getString(ACCELEROMETER_AVERAGING_FILTER_TYPE)
        gyroscopeSensorType =
            (extras?.getSerializable(GYROSCOPE_SENSOR_TYPE) as GyroscopeSensorCollector.SensorType?)
                ?: GyroscopeSensorCollector.SensorType.GYROSCOPE
        useAccurateLevelingEstimator =
            extras?.getBoolean(USE_ACCURATE_LEVELING_ESTIMATOR, false) ?: false
        useAccurateRelativeGyroscopeAttitudeEstimator =
            extras?.getBoolean(USE_ACCURATE_RELATIVE_GYROSCOPE_ATTITUDE_ESTIMATOR, false) ?: false
        useWorldMagneticModel = extras?.getBoolean(GeomagneticAttitudeEstimatorActivity.USE_WORLD_MAGNETIC_MODEL, false) ?: false

        setContentView(R.layout.activity_fused_geomagnetic_attitude_estimator2)
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
        attitudeEstimator?.stop()
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
                    this@FusedGeomagneticAttitudeEstimator2Activity.location = currentLocation
                    buildEstimatorAndStart()
                }
            }
        }
    }

    private fun buildEstimatorAndStart() {
        val location = this.location ?: return
        if (attitudeEstimator?.running == true) return

        if (attitudeEstimator != null) {
            attitudeEstimator?.location = location
        } else {
            val accelerometerAveragingFilter =
                buildAveragingFilter(accelerometerAveragingFilterType)

            attitudeEstimator = FusedGeomagneticAttitudeEstimator2(
                this,
                location,
                SensorDelay.GAME,
                useAccelerometer = useAccelerometer,
                accelerometerSensorType = accelerometerSensorType,
                accelerometerAveragingFilter = accelerometerAveragingFilter,
                magnetometerSensorType = magnetometerSensorType,
                gyroscopeSensorType = gyroscopeSensorType,
                useAccurateLevelingEstimator = useAccurateLevelingEstimator,
                useAccurateRelativeGyroscopeAttitudeEstimator = useAccurateRelativeGyroscopeAttitudeEstimator,
                useWorldMagneticModel = useWorldMagneticModel,
                attitudeAvailableListener = { _, attitude, _, _, _, _ ->
                    attitude.toQuaternion(rotation)
                    cubeView?.cubeRotation = rotation

                    rotation.toEulerAngles(eulerAngles)
                    rollView?.text = getString(R.string.roll_degrees, -Math.toDegrees(eulerAngles[0]))
                    pitchView?.text = getString(R.string.pitch_degrees, -Math.toDegrees(eulerAngles[1]))
                    yawView?.text = getString(R.string.yaw_degrees, -Math.toDegrees(eulerAngles[2]))
                }
            )
        }
        attitudeEstimator?.start()
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
        const val MAGNETOMETER_SENSOR_TYPE = "magnetometerSensorType"
        const val ACCELEROMETER_AVERAGING_FILTER_TYPE = "accelerometerAveragingFilterType"
        const val GYROSCOPE_SENSOR_TYPE = "gyroscopeSensorType"
        const val USE_ACCURATE_LEVELING_ESTIMATOR = "useAccurateLevelingEstimator"
        const val USE_ACCURATE_RELATIVE_GYROSCOPE_ATTITUDE_ESTIMATOR =
            "useAccurateRelativeGyroscopeAttitudeEstimator"
        const val USE_WORLD_MAGNETIC_MODEL = "useWorldMagneticModel"

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