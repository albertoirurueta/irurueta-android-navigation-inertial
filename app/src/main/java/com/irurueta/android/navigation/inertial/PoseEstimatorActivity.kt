package com.irurueta.android.navigation.inertial

import android.annotation.SuppressLint
import android.location.Location
import android.os.Build
import android.os.Bundle
import androidx.appcompat.app.AppCompatActivity
import androidx.appcompat.widget.AppCompatTextView
import com.irurueta.android.gl.cube.CubeRenderer
import com.irurueta.android.gl.cube.CubeTextureView
import com.irurueta.android.navigation.inertial.collectors.AccelerometerSensorCollector
import com.irurueta.android.navigation.inertial.collectors.GyroscopeSensorCollector
import com.irurueta.android.navigation.inertial.collectors.MagnetometerSensorCollector
import com.irurueta.android.navigation.inertial.collectors.SensorDelay
import com.irurueta.android.navigation.inertial.estimators.PoseEstimator
import com.irurueta.android.navigation.inertial.estimators.filter.AveragingFilter
import com.irurueta.android.navigation.inertial.estimators.filter.LowPassAveragingFilter
import com.irurueta.android.navigation.inertial.estimators.filter.MeanAveragingFilter
import com.irurueta.android.navigation.inertial.estimators.filter.MedianAveragingFilter
import com.irurueta.geometry.*

class PoseEstimatorActivity : AppCompatActivity() {

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

    private var poseEstimator: PoseEstimator? = null

    private val conversionRotation = ENUtoNEDTriadConverter.conversionRotation

    private var hasLocationPermission = false

    private var location: Location? = null

    private val locationPermissionService = LocationPermissionService(this) { locationResult ->
        hasLocationPermission = locationResult.finePermissionGranted
        checkLocationAndStart()
    }

    private var locationService: LocationService? = null

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
        accelerometerSensorType =
            (extras?.getSerializable(ACCELEROMETER_SENSOR_TYPE) as AccelerometerSensorCollector.SensorType?)
                ?: AccelerometerSensorCollector.SensorType.ACCELEROMETER
        magnetometerSensorType =
            (extras?.getSerializable(MAGNETOMETER_SENSOR_TYPE) as MagnetometerSensorCollector.SensorType?)
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
        useWorldMagneticModel =
            extras?.getBoolean(USE_WORLD_MAGNETIC_MODEL, false)
                ?: false

        setContentView(R.layout.activity_pose_estimator)
        cubeView = findViewById(R.id.cube)
        rollView = findViewById(R.id.roll)
        pitchView = findViewById(R.id.pitch)
        yawView = findViewById(R.id.yaw)
        xPosView = findViewById(R.id.x_pos)
        yPosView = findViewById(R.id.y_pos)
        zPosView = findViewById(R.id.z_pos)

        val cubeView = cubeView ?: return
        cubeView.onSurfaceChangedListener = object : CubeTextureView.OnSurfaceChangedListener {
            override fun onSurfaceChanged(width: Int, height: Int) {
                cubeView.cubeSize = 0.5f * CubeRenderer.DEFAULT_CUBE_SIZE
                initialCamera = createCamera(cubeView, Quaternion())
                camera = createCamera(cubeView, Quaternion())
                cubeView.camera = camera
                cubeView.cubePosition =
                    InhomogeneousPoint3D(-CubeRenderer.DEFAULT_CUBE_DISTANCE, 0.0, 0.0)
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
                    this@PoseEstimatorActivity.location = currentLocation
                    buildEstimatorAndStart()
                }
            }
        }
    }

    private fun buildEstimatorAndStart() {
        val location = this.location ?: return
        if (poseEstimator?.running == true) return

        if (poseEstimator != null) {
            poseEstimator?.location = location
        } else {
            val accelerometerAveragingFilter =
                buildAveragingFilter(accelerometerAveragingFilterType)

            val refreshRate = if (Build.VERSION.SDK_INT > Build.VERSION_CODES.Q) {
                this.display?.mode?.refreshRate ?: FALLBACK_REFRESH_RATE
            } else {
                FALLBACK_REFRESH_RATE
            }
            val refreshIntervalNanos = (1.0f / refreshRate * 1e9).toLong()

            poseEstimator = PoseEstimator(
                this,
                location,
                sensorDelay = SensorDelay.FASTEST,
                accelerometerSensorType = accelerometerSensorType,
                magnetometerSensorType = magnetometerSensorType,
                accelerometerAveragingFilter = accelerometerAveragingFilter,
                gyroscopeSensorType = gyroscopeSensorType,
                useWorldMagneticModel = useWorldMagneticModel,
                estimateInitialTransformation = true,
                poseAvailableListener = { estimator, currentFrame, previousFrame, initialFrame, currentAttitude, previousAttitude, initialAttitude, timestamp, initialTransformation, previousTransformation ->

                    if (!initialAttitudeAvailable) {
                        previousTimestamp = timestamp

                        initialAttitudeAvailable = true

                        // copy camera
                        //initialCamera.internalMatrix = Matrix(camera?.internalMatrix)
                    }

                    // refresh only as much as display allows even though sensors might run at higher refresh rates
                    if (timestamp - previousTimestamp >= refreshIntervalNanos) {
                        previousTimestamp = timestamp

                        updateCamera(initialAttitude, currentAttitude)

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

    private fun updateCamera(initialAttitude: Quaternion, currentAttitude: Quaternion) {
        // P0 = [K*R0    t] = [K*R0 -R0*C], where R0 is camera rotation and C is camera center

        // T1 = [R1   t1]
        //      [0     1]

        // x0 = P0 * X0

        // X1 = T * X0

        // x1 = P0 * X1 = P0 * T * X0 -> P1 = P0 * T

        // P1 = [K*R0   -R0*C] * [R1 t1] = [K*R9*R1     K*R0*t1 - R0*C]
        //                       [0   1]


        // set rotation
        val initRotation = Quaternion(initialAttitude)
        initRotation.toEulerAngles(eulerAngles)
        val initYaw = eulerAngles[2]

        val currentRotation = Quaternion(currentAttitude)
        currentRotation.toEulerAngles(eulerAngles)
        val currentRoll = eulerAngles[0]
        val currentPitch = eulerAngles[1]
        val currentYaw = eulerAngles[2]

        val deltaYaw = currentYaw - initYaw

        val rotation = Quaternion()
        rotation.setFromEulerAngles(currentRoll, currentPitch, deltaYaw)
        Quaternion.product(rotation, conversionRotation, rotation)

        rotation.toEulerAngles(eulerAngles)
        rollView?.text = getString(R.string.roll_degrees, Math.toDegrees(eulerAngles[0]))
        pitchView?.text =
            getString(R.string.pitch_degrees, Math.toDegrees(eulerAngles[1]))
        yawView?.text = getString(R.string.yaw_degrees, Math.toDegrees(eulerAngles[2]))

        val transformation = EuclideanTransformation3D(rotation)
        // transformation needs to be inverted because when camera is transformed, the inverse
        // transformation is used
        transformation.inverse()
        transformation.transform(initialCamera, camera)

        // Lines above are equivalent to:
        // camera?.cameraRotation = rotation
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