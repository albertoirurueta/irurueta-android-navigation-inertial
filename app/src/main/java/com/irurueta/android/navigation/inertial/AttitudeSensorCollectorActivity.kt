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

import android.os.Bundle
import androidx.appcompat.app.AppCompatActivity
import androidx.appcompat.widget.AppCompatTextView
import com.irurueta.android.gl.cube.CubeRenderer
import com.irurueta.android.gl.cube.CubeTextureView
import com.irurueta.android.navigation.inertial.collectors.AttitudeSensorCollector
import com.irurueta.android.navigation.inertial.collectors.SensorDelay
import com.irurueta.geometry.*

class AttitudeSensorCollectorActivity : AppCompatActivity() {

    private var cubeView: CubeTextureView? = null

    private var rollView: AppCompatTextView? = null

    private var pitchView: AppCompatTextView? = null

    private var yawView: AppCompatTextView? = null

    private var accuracyView: AppCompatTextView? = null

    private var rotation = Quaternion()

    private var eulerAngles = DoubleArray(3)

    private var camera: PinholeCamera? = null

    private var attitudeSensorCollector: AttitudeSensorCollector? = null

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)

        val extras = intent.extras
        val serializable =
            extras?.getSerializable(SENSOR_TYPE) as AttitudeSensorCollector.SensorType?
        val sensorType = serializable ?: AttitudeSensorCollector.SensorType.RELATIVE_ATTITUDE
        setContentView(R.layout.activity_attitude_sensor_collector)
        cubeView = findViewById(R.id.cube)
        rollView = findViewById(R.id.roll)
        pitchView = findViewById(R.id.pitch)
        yawView = findViewById(R.id.yaw)
        accuracyView = findViewById(R.id.accuracy)

        val cubeView = cubeView ?: return
        cubeView.onSurfaceChangedListener = object : CubeTextureView.OnSurfaceChangedListener {
            override fun onSurfaceChanged(width: Int, height: Int) {
                cubeView.cubeSize = 0.5f * CubeRenderer.DEFAULT_CUBE_SIZE
                cubeView.cubeRotation = rotation
                camera = createCamera(cubeView)
                cubeView.camera = camera
            }
        }

        attitudeSensorCollector = AttitudeSensorCollector(
            this,
            sensorType,
            SensorDelay.UI,
            measurementListener = { attitude, _, accuracy, _, _ ->
                attitude.toQuaternion(rotation)
                cubeView.cubeRotation = rotation

                rotation.toEulerAngles(eulerAngles)
                rollView?.text = getString(R.string.roll_degrees, -Math.toDegrees(eulerAngles[0]))
                pitchView?.text = getString(R.string.pitch_degrees, -Math.toDegrees(eulerAngles[1]))
                yawView?.text = getString(R.string.yaw_degrees, -Math.toDegrees(eulerAngles[2]))
                if (accuracy != null) {
                    accuracyView?.text =
                        getString(
                            R.string.yaw_standard_deviation_degrees,
                            Math.toDegrees(accuracy.toDouble())
                        )
                }
            }
        )
    }

    override fun onResume() {
        super.onResume()
        cubeView?.onResume()
        attitudeSensorCollector?.start()
    }

    override fun onPause() {
        super.onPause()
        attitudeSensorCollector?.stop()
        cubeView?.onPause()
    }

    companion object {
        const val SENSOR_TYPE = "sensorType"

        val ABSOLUTE_ATTITUDE = AttitudeSensorCollector.SensorType.ABSOLUTE_ATTITUDE

        val RELATIVE_ATTITUDE = AttitudeSensorCollector.SensorType.RELATIVE_ATTITUDE

        val GEOMAGNETIC_ABSOLUTE_ATTITUDE =
            AttitudeSensorCollector.SensorType.GEOMAGNETIC_ABSOLUTE_ATTITUDE

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