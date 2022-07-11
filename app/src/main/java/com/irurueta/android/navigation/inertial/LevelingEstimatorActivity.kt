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
import com.irurueta.android.navigation.inertial.collectors.AccelerometerSensorCollector
import com.irurueta.android.navigation.inertial.collectors.SensorDelay
import com.irurueta.android.navigation.inertial.estimators.LevelingEstimator
import com.irurueta.android.navigation.inertial.estimators.filter.AveragingFilter
import com.irurueta.android.navigation.inertial.estimators.filter.LowPassAveragingFilter
import com.irurueta.android.navigation.inertial.estimators.filter.MeanAveragingFilter
import com.irurueta.android.navigation.inertial.estimators.filter.MedianAveragingFilter
import com.irurueta.geometry.*

class LevelingEstimatorActivity : AppCompatActivity() {

    private var cubeView: CubeTextureView? = null

    private var rollView: AppCompatTextView? = null

    private var pitchView: AppCompatTextView? = null

    private var rotation = Quaternion()

    private var eulerAngles = DoubleArray(3)

    private var camera: PinholeCamera? = null

    private var levelingEstimator: LevelingEstimator? = null

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)

        val extras = intent.extras
        val useAccelerometer = extras?.getBoolean(USE_ACCELEROMETER, false) ?: false
        val accelerometerSensorType =
            (extras?.getSerializable(ACCELEROMETER_SENSOR_TYPE) as AccelerometerSensorCollector.SensorType?)
                ?: AccelerometerSensorCollector.SensorType.ACCELEROMETER
        val averagingFilterType = extras?.getString(AVERAGING_FILTER_TYPE)
        val averagingFilter = buildAveragingFilter(averagingFilterType)

        setContentView(R.layout.activity_leveling_estimator)
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

        levelingEstimator = LevelingEstimator(
            this,
            SensorDelay.GAME,
            useAccelerometer = useAccelerometer,
            accelerometerSensorType = accelerometerSensorType,
            accelerometerAveragingFilter = averagingFilter
        ) { _, attitude, _, _, _ ->
            attitude.toQuaternion(rotation)
            cubeView.cubeRotation = rotation

            rotation.toEulerAngles(eulerAngles)
            rollView?.text = getString(R.string.roll_degrees, -Math.toDegrees(eulerAngles[0]))
            pitchView?.text = getString(R.string.pitch_degrees, -Math.toDegrees(eulerAngles[1]))
        }
    }

    override fun onResume() {
        super.onResume()
        cubeView?.onResume()
        levelingEstimator?.start()
    }

    override fun onPause() {
        super.onPause()
        levelingEstimator?.stop()
        cubeView?.onPause()
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