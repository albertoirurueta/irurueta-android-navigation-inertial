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
import com.irurueta.android.navigation.inertial.collectors.AttitudeSensorType
import com.irurueta.android.navigation.inertial.collectors.SensorDelay
import com.irurueta.android.navigation.inertial.estimators.attitude.AttitudeEstimator2
import com.irurueta.geometry.PinholeCamera
import com.irurueta.geometry.PinholeCameraIntrinsicParameters
import com.irurueta.geometry.Point3D
import com.irurueta.geometry.Quaternion
import com.irurueta.geometry.Rotation3D

class AttitudeEstimatorActivity : AppCompatActivity() {

    private var cubeView: CubeTextureView? = null

    private var rollView: AppCompatTextView? = null

    private var pitchView: AppCompatTextView? = null

    private var yawView: AppCompatTextView? = null

    private var headingAccuracyView: AppCompatTextView? = null

    private var rotation = Quaternion()

    private var camera: PinholeCamera? = null

    private var attitudeEstimator: AttitudeEstimator2? = null

    private val displayOrientation = Quaternion()

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)

        val extras = intent.extras
        val attitudeSensorType = if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
            extras?.getSerializable(ATTITUDE_SENSOR_TYPE, AttitudeSensorType::class.java)
                ?: AttitudeSensorType.ABSOLUTE_ATTITUDE
        } else {
            @Suppress("DEPRECATION")
            (extras?.getSerializable(ATTITUDE_SENSOR_TYPE) as AttitudeSensorType?)
                ?: AttitudeSensorType.ABSOLUTE_ATTITUDE
        }
        setContentView(com.irurueta.android.navigation.inertial.app.R.layout.activity_attitude_estimator)
        cubeView = findViewById(com.irurueta.android.navigation.inertial.app.R.id.cube)
        rollView = findViewById(com.irurueta.android.navigation.inertial.app.R.id.roll)
        pitchView = findViewById(com.irurueta.android.navigation.inertial.app.R.id.pitch)
        yawView = findViewById(com.irurueta.android.navigation.inertial.app.R.id.yaw)
        headingAccuracyView =
            findViewById(com.irurueta.android.navigation.inertial.app.R.id.heading_accuracy)

        val cubeView = cubeView ?: return
        cubeView.onSurfaceChangedListener = object : CubeTextureView.OnSurfaceChangedListener {
            override fun onSurfaceChanged(width: Int, height: Int) {
                cubeView.cubeSize = 0.5f * CubeRenderer.DEFAULT_CUBE_SIZE
                cubeView.cubeRotation = rotation
                camera = createCamera(cubeView)
                cubeView.camera = camera
            }
        }

        attitudeEstimator = AttitudeEstimator2(
            this,
            SensorDelay.GAME,
            attitudeSensorType = attitudeSensorType,
            estimateCoordinateTransformation = false,
            estimateEulerAngles = true,
            attitudeAvailableListener = { _, attitude, _, headingAccuracy, roll, pitch, yaw, _ ->
                attitude.toQuaternion(rotation)

                rollView?.text = getString(
                    com.irurueta.android.navigation.inertial.app.R.string.roll_degrees,
                    Math.toDegrees(roll ?: 0.0)
                )
                pitchView?.text = getString(
                    com.irurueta.android.navigation.inertial.app.R.string.pitch_degrees,
                    Math.toDegrees(pitch ?: 0.0)
                )
                yawView?.text = getString(
                    com.irurueta.android.navigation.inertial.app.R.string.yaw_degrees,
                    Math.toDegrees(yaw ?: 0.0)
                )
                if (headingAccuracy != null) {
                    headingAccuracyView?.text = getString(
                        com.irurueta.android.navigation.inertial.app.R.string.heading_accuracy_degrees,
                        Math.toDegrees(headingAccuracy.toDouble())
                    )
                    headingAccuracyView?.visibility = View.VISIBLE
                } else {
                    headingAccuracyView?.visibility = View.INVISIBLE
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
        const val ATTITUDE_SENSOR_TYPE = "attitudeSensorType"

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