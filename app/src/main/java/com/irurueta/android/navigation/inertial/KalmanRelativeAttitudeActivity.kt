/*
 * Copyright (C) 2023 Alberto Irurueta Carro (alberto@irurueta.com)
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

import android.os.Build
import android.os.Bundle
import android.view.View
import androidx.appcompat.app.AppCompatActivity
import androidx.appcompat.widget.AppCompatTextView
import com.irurueta.android.gl.cube.CubeRenderer
import com.irurueta.android.gl.cube.CubeTextureView
import com.irurueta.android.navigation.inertial.collectors.AccelerometerSensorType
import com.irurueta.android.navigation.inertial.collectors.GyroscopeSensorType
import com.irurueta.android.navigation.inertial.collectors.SensorDelay
import com.irurueta.android.navigation.inertial.estimators.attitude.KalmanRelativeAttitudeEstimator
import com.irurueta.geometry.*
import kotlin.math.sqrt

class KalmanRelativeAttitudeActivity : AppCompatActivity() {

    private var cubeView: CubeTextureView? = null

    private var rollView: AppCompatTextView? = null

    private var pitchView: AppCompatTextView? = null

    private var yawView: AppCompatTextView? = null

    private var rollStandardDeviationView: AppCompatTextView? = null

    private var pitchStandardDeviationView: AppCompatTextView? = null

    private var yawStandardDeviationView: AppCompatTextView? = null

    private var rotation = Quaternion()

    private var camera: PinholeCamera? = null

    private var attitudeEstimator: KalmanRelativeAttitudeEstimator? = null

    private val displayOrientation = Quaternion()

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)

        val extras = intent.extras

        val accelerometerSensorType = if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
            extras?.getSerializable(ACCELEROMETER_SENSOR_TYPE, AccelerometerSensorType::class.java)
                ?: AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED
        } else {
            @Suppress("DEPRECATION")
            (extras?.getSerializable(ACCELEROMETER_SENSOR_TYPE) as AccelerometerSensorType?)
                ?: AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED
        }

        val gyroscopeSensorType = if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
            extras?.getSerializable(GYROSCOPE_SENSOR_TYPE, GyroscopeSensorType::class.java)
                ?: GyroscopeSensorType.GYROSCOPE_UNCALIBRATED
        } else {
            @Suppress("DEPRECATION")
            (extras?.getSerializable(GYROSCOPE_SENSOR_TYPE) as GyroscopeSensorType?)
                ?: GyroscopeSensorType.GYROSCOPE_UNCALIBRATED
        }

        setContentView(R.layout.activity_kalman_relative_attitude_estimator)
        cubeView = findViewById(R.id.cube)
        rollView = findViewById(R.id.roll)
        pitchView = findViewById(R.id.pitch)
        yawView = findViewById(R.id.yaw)
        rollStandardDeviationView = findViewById(R.id.roll_accuracy)
        pitchStandardDeviationView = findViewById(R.id.pitch_accuracy)
        yawStandardDeviationView = findViewById(R.id.yaw_accuracy)

        val cubeView = cubeView ?: return
        cubeView.onSurfaceChangedListener = object : CubeTextureView.OnSurfaceChangedListener {
            override fun onSurfaceChanged(width: Int, height: Int) {
                cubeView.cubeSize = 0.5f * CubeRenderer.DEFAULT_CUBE_SIZE
                cubeView.cubeRotation = rotation
                camera = createCamera(cubeView)
                cubeView.camera = camera
            }
        }

        attitudeEstimator = KalmanRelativeAttitudeEstimator(
            this,
            SensorDelay.GAME,
            startOffsetEnabled = false,
            accelerometerSensorType,
            gyroscopeSensorType,
            estimateCoordinateTransformation = false,
            estimateEulerAngles = true,
            estimateCovariances = true,
            attitudeAvailableListener = { _, attitude, _, roll, pitch, yaw, _, _, covariance ->
                attitude.toQuaternion(rotation)

                rollView?.text = getString(R.string.roll_degrees, Math.toDegrees(roll ?: 0.0))
                pitchView?.text = getString(R.string.pitch_degrees, Math.toDegrees(pitch ?: 0.0))
                yawView?.text = getString(R.string.yaw_degrees, Math.toDegrees(yaw ?: 0.0))

                if (covariance != null) {
                    val rollStd = Math.toDegrees(sqrt(covariance.getElementAt(0, 0)))
                    val pitchStd = Math.toDegrees(sqrt(covariance.getElementAt(1, 1)))
                    val yawStd = Math.toDegrees(sqrt(covariance.getElementAt(2, 2)))

                    rollStandardDeviationView?.text =
                        getString(R.string.roll_accuracy_degrees, rollStd)
                    pitchStandardDeviationView?.text =
                        getString(R.string.pitch_accuracy_degrees, pitchStd)
                    yawStandardDeviationView?.text =
                        getString(R.string.yaw_accuracy_degrees, yawStd)
                    rollStandardDeviationView?.visibility = View.VISIBLE
                    pitchStandardDeviationView?.visibility = View.VISIBLE
                    yawStandardDeviationView?.visibility = View.VISIBLE
                } else {
                    rollStandardDeviationView?.visibility = View.INVISIBLE
                    pitchStandardDeviationView?.visibility = View.INVISIBLE
                    yawStandardDeviationView?.visibility = View.INVISIBLE
                }

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

                cubeView.cubeRotation = rotation
            }
        )
    }

    override fun onResume() {
        super.onResume()
        cubeView?.onResume()
        attitudeEstimator?.start()
    }

    override fun onPause() {
        super.onPause()
        attitudeEstimator?.stop()
        cubeView?.onPause()
    }

    companion object {
        const val ACCELEROMETER_SENSOR_TYPE = "accelerometerSensorType"

        const val GYROSCOPE_SENSOR_TYPE = "gyroscopeSensorType"

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