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
import com.irurueta.android.gl.cube.CubeRenderer
import com.irurueta.android.gl.cube.CubeTextureView
import com.irurueta.android.glutils.OpenGlToCameraHelper
import com.irurueta.android.navigation.inertial.collectors.AttitudeSensorCollector
import com.irurueta.android.navigation.inertial.collectors.SensorDelay
import com.irurueta.geometry.PinholeCamera
import com.irurueta.geometry.Point3D
import com.irurueta.geometry.Quaternion
import com.irurueta.geometry.Rotation3D

class AttitudeSensorCollectorActivity : AppCompatActivity() {

    private var cubeView: CubeTextureView? = null

    private var rotation = Quaternion()

    private var camera: PinholeCamera? = null

    private var attitudeSensorCollector: AttitudeSensorCollector? = null

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)

        val extras = intent.extras
        val serializable = extras?.getSerializable(SENSOR_TYPE) as AttitudeSensorCollector.SensorType?
        val sensorType = serializable ?: AttitudeSensorCollector.SensorType.RELATIVE_ATTITUDE
        //supportActionBar?.hide()
        setContentView(R.layout.activity_attitud_sensor_collector)
        cubeView = findViewById(R.id.cube)

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
            measurementListener = { attitude, _, _, _, _ ->
                attitude.toQuaternion(rotation)
                cubeView.cubeRotation = rotation
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

        private fun createProjectionMatrix(): FloatArray {
            // Pixel 2 device has the following projection matrix
            // [2.8693345   0.0         -0.004545755        0.0         ]
            // [0.0	        1.5806589   0.009158132         0.0         ]
            // [0.0	        0.0         -1.002002           -0.2002002  ]
            // [0.0         0.0         -1.0                0.0         ]

            // android.opengl.Matrix defines values column-wise
            return floatArrayOf(
                2.8693345f, 0.0f, 0.0f, 0.0f,
                0.0f, 1.5806589f, 0.0f, 0.0f,
                -0.004545755f, 0.009158132f, -1.002002f, -1.0f,
                0.0f, 0.0f, -0.2002002f, 0.0f
            )
        }

        private fun createCamera(view: CubeTextureView): PinholeCamera {
            val projectionMatrix = createProjectionMatrix()
            val intrinsics = OpenGlToCameraHelper.computePinholeCameraIntrinsicsAndReturnNew(
                projectionMatrix,
                view.width,
                view.height
            )
            return PinholeCamera(intrinsics, Rotation3D.create(), Point3D.create())
        }
    }
}