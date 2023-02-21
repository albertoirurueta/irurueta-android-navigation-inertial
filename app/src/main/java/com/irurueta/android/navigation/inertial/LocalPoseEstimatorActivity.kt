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
import com.irurueta.android.navigation.inertial.collectors.*
import com.irurueta.android.navigation.inertial.estimators.filter.AveragingFilter
import com.irurueta.android.navigation.inertial.estimators.filter.LowPassAveragingFilter
import com.irurueta.android.navigation.inertial.estimators.filter.MeanAveragingFilter
import com.irurueta.android.navigation.inertial.estimators.filter.MedianAveragingFilter
import com.irurueta.android.navigation.inertial.estimators.pose.LocalPoseEstimator2
import com.irurueta.geometry.*

class LocalPoseEstimatorActivity : AppCompatActivity() {

    private var cubeView: CubeTextureView? = null

    private var rollView: AppCompatTextView? = null

    private var pitchView: AppCompatTextView? = null

    private var yawView: AppCompatTextView? = null

    private var xPosView: AppCompatTextView? = null

    private var yPosView: AppCompatTextView? = null

    private var zPosView: AppCompatTextView? = null

    private var initialAttitudeAvailable = false

    private var previousTimestamp = -1L

    private var initialCamera = PinholeCamera()

    private var camera: PinholeCamera? = null

    private var poseEstimator: LocalPoseEstimator2? = null

    private val conversionRotation = ENUtoNEDTriadConverter.conversionRotation

    private var hasLocationPermission = false

    private var location: Location? = null

    private val locationPermissionService = LocationPermissionService(this) { locationResult ->
        hasLocationPermission = locationResult.finePermissionGranted
        checkLocationAndStart()
    }

    private var locationService: LocationService? = null

    private var useWorldMagneticModel = false

    private var accelerometerSensorType =
        AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED

    private var magnetometerSensorType =
        MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED

    private var accelerometerAveragingFilterType: String? = null

    private var gyroscopeSensorType = GyroscopeSensorType.GYROSCOPE_UNCALIBRATED

    private var useAccurateLevelingEstimator = false

    private var useAccurateRelativeGyroscopeAttitudeEstimator = false

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)

        val extras = intent.extras
        accelerometerSensorType = if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
            extras?.getSerializable(ACCELEROMETER_SENSOR_TYPE, AccelerometerSensorType::class.java)
                ?: AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED
        } else {
            @Suppress("DEPRECATION")
            (extras?.getSerializable(ACCELEROMETER_SENSOR_TYPE) as AccelerometerSensorType?)
                ?: AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED
        }
        magnetometerSensorType = if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
            extras?.getSerializable(MAGNETOMETER_SENSOR_TYPE, MagnetometerSensorType::class.java)
                ?: MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED
        } else {
            @Suppress("DEPRECATION")
            (extras?.getSerializable(MAGNETOMETER_SENSOR_TYPE) as MagnetometerSensorType?)
                ?: MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED
        }
        accelerometerAveragingFilterType =
            extras?.getString(ACCELEROMETER_AVERAGING_FILTER_TYPE)
        gyroscopeSensorType = if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
            extras?.getSerializable(GYROSCOPE_SENSOR_TYPE, GyroscopeSensorType::class.java)
                ?: GyroscopeSensorType.GYROSCOPE_UNCALIBRATED
        } else {
            @Suppress("DEPRECATION")
            (extras?.getSerializable(GYROSCOPE_SENSOR_TYPE) as GyroscopeSensorType?)
                ?: GyroscopeSensorType.GYROSCOPE_UNCALIBRATED
        }
        useAccurateLevelingEstimator =
            extras?.getBoolean(USE_ACCURATE_LEVELING_ESTIMATOR, false) ?: false
        useAccurateRelativeGyroscopeAttitudeEstimator =
            extras?.getBoolean(USE_ACCURATE_RELATIVE_GYROSCOPE_ATTITUDE_ESTIMATOR, false) ?: false
        useWorldMagneticModel =
            extras?.getBoolean(USE_WORLD_MAGNETIC_MODEL, false)
                ?: false

        setContentView(R.layout.activity_local_pose_estimator)
        cubeView = findViewById(R.id.cube)
        rollView = findViewById(R.id.roll)
        pitchView = findViewById(R.id.pitch)
        yawView = findViewById(R.id.yaw)
        xPosView = findViewById(R.id.x_pos)
        yPosView = findViewById(R.id.y_pos)
        zPosView = findViewById(R.id.z_pos)

        val cubeSize = 0.25f
        val cubeDistance = 0.5
        val cubeView = cubeView ?: return
        cubeView.onSurfaceChangedListener = object : CubeTextureView.OnSurfaceChangedListener {
            override fun onSurfaceChanged(width: Int, height: Int) {
                cubeView.cubeSize = 0.5f * CubeRenderer.DEFAULT_CUBE_SIZE
                initialCamera = createCamera(cubeView, Quaternion())
                camera = createCamera(cubeView, Quaternion())
                cubeView.camera = camera
                cubeView.cubeSize = cubeSize
                cubeView.cubePosition =
                    InhomogeneousPoint3D(-cubeDistance, 0.0, 0.0)
                //cubeView.cubePosition =
                //    InhomogeneousPoint3D(-CubeRenderer.DEFAULT_CUBE_DISTANCE, 0.0, 0.0)
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
        poseEstimator?.stop()
        cubeView?.onPause()
        initialAttitudeAvailable = false
        previousTimestamp = -1L
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
                    this@LocalPoseEstimatorActivity.location = currentLocation
                    buildEstimatorAndStart()
                }
            }
        }
    }

    private fun buildEstimatorAndStart() {
        val location = this.location ?: return
        if (poseEstimator?.running == true) return

        if (poseEstimator != null) {
            // TODO: poseEstimator?.initialLocation = location
        } else {
            val accelerometerAveragingFilter =
                buildAveragingFilter(accelerometerAveragingFilterType)

            val refreshRate = if (Build.VERSION.SDK_INT > Build.VERSION_CODES.Q) {
                this.display?.mode?.refreshRate ?: FALLBACK_REFRESH_RATE
            } else {
                FALLBACK_REFRESH_RATE
            }
            val refreshIntervalNanos = (1.0f / refreshRate * 1e9).toLong()

            poseEstimator = LocalPoseEstimator2(
                this,
                location,
                sensorDelay = SensorDelay.FASTEST,
                useAttitudeSensor = true,
                useAccelerometerForAttitudeEstimation = false,
                startOffsetEnabled = false,
                accelerometerSensorType = accelerometerSensorType,
                gyroscopeSensorType = gyroscopeSensorType,
                magnetometerSensorType = magnetometerSensorType,
                accelerometerAveragingFilter = accelerometerAveragingFilter,
                useWorldMagneticModel = useWorldMagneticModel,
                useAccurateLevelingProcessor = true,
                useDoubleFusedAttitudeProcessor = true,
                estimatePoseTransformation = true,
                poseAvailableListener = { _, _, _, _, timestamp, initialTransformation ->

                    if (!initialAttitudeAvailable) {
                        previousTimestamp = timestamp

                        initialAttitudeAvailable = true
                    }

                    // refresh only as much as display allows even though sensors might run at higher refresh rates
                    if (timestamp - previousTimestamp >= refreshIntervalNanos) {
                        previousTimestamp = timestamp

                        val enuRotation = Quaternion()
                        initialTransformation?.rotation?.toQuaternion(enuRotation)
                        val nedRotation = Quaternion()
                        Quaternion.product(enuRotation, conversionRotation, nedRotation)
                        nedRotation.toEulerAngles(eulerAngles)
                        rollView?.text =
                            getString(R.string.roll_degrees, Math.toDegrees(eulerAngles[0]))
                        pitchView?.text =
                            getString(R.string.pitch_degrees, Math.toDegrees(eulerAngles[1]))
                        yawView?.text =
                            getString(R.string.yaw_degrees, Math.toDegrees(eulerAngles[2]))

                        initialTransformation?.inverse()
                        initialTransformation?.transform(initialCamera, camera)

                        cubeView?.camera = camera
                    }
                }
            )
            poseEstimator?.start()
        }
    }

    private fun buildAveragingFilter(averagingFilterType: String?): AveragingFilter {
        return when (averagingFilterType) {
            MEAN_AVERAGING_FILTER -> MeanAveragingFilter()
            MEDIAN_AVERAGING_FILTER -> MedianAveragingFilter()
            else -> LowPassAveragingFilter()
        }
    }

    private val eulerAngles = DoubleArray(Quaternion.N_ANGLES)

    companion object {
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

        private const val FALLBACK_REFRESH_RATE = 60.0f

        private fun createCamera(view: CubeTextureView, attitude: Rotation3D): PinholeCamera {
            val px = view.width / 2.0
            val py = view.height / 2.0

            val intrinsics = PinholeCameraIntrinsicParameters(
                HORIZONTAL_FOCAL_LENGTH,
                VERTICAL_FOCAL_LENGTH,
                px,
                py,
                0.0
            )

            return PinholeCamera(intrinsics, attitude, Point3D.create())
        }
    }
}