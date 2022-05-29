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
package com.irurueta.android.navigation.inertial.collectors

import android.content.Context
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.location.Location
import android.os.Build
import androidx.annotation.RequiresApi
import com.irurueta.algebra.Matrix
import com.irurueta.android.navigation.inertial.PoseHelper
import com.irurueta.android.navigation.inertial.toNEDPosition
import com.irurueta.geometry.*
import com.irurueta.navigation.frames.ECEFPosition
import com.irurueta.navigation.frames.NEDFrame
import com.irurueta.navigation.frames.NEDPosition
import com.irurueta.navigation.frames.NEDVelocity

/**
 * Manages and collects 6 DoF (Degree of Freedom) pose sensor measurements.
 *
 * @property context Android context.
 * @property sensorDelay Delay of sensor between samples.
 * @property measurementListener listener to notify new pose measurements.
 * @property accuracyChangedListener listener to notify changes in pose accuracy.
 */
@RequiresApi(Build.VERSION_CODES.N)
class RelativeDevicePoseSensorCollector private constructor(
    context: Context,
    sensorDelay: SensorDelay = SensorDelay.FASTEST,
    var timeInterval: Double = 0.0,
    var measurementListener: OnMeasurementListener? = null,
    accuracyChangedListener: SensorCollector.OnAccuracyChangedListener? = null
) : SensorCollector(context, sensorDelay, accuracyChangedListener) {

    /**
     * Constructor.
     *
     * @param context Android context.
     * @param sensorDelay Delay of sensor between samples.
     * @param location current location. If provided, frame containing absolute position and
     * attitude will be estimated.
     * @param timeInterval time interval between measurements expressed in seconds (s).
     * @param measurementListener listener to notify new pose measurements.
     * @param accuracyChangedListener listener to notify changes in pose accuracy.
     */
    constructor(
        context: Context,
        sensorDelay: SensorDelay = SensorDelay.FASTEST,
        location: Location? = null,
        timeInterval: Double = 0.0,
        measurementListener: OnMeasurementListener? = null,
        accuracyChangedListener: SensorCollector.OnAccuracyChangedListener? = null
    ) : this(
        context,
        sensorDelay,
        timeInterval,
        measurementListener,
        accuracyChangedListener
    ) {
        this.location = location
    }

    /**
     * Start position to be reused and expressed in NED coordinates.
     */
    private var startPosition: NEDPosition? = null

    /**
     * Frame to be reused and containing current position, attitude and velocity respect Earth.
     */
    private val frame = NEDFrame()

    /**
     * Instance being reused for performance reasons and containing device attitude expressed in
     * NED coordinate system.
     */
    private val attitude = Quaternion()

    /**
     * Relative translation respect an arbitrary position to be reused.
     */
    private val translation = InhomogeneousPoint3D()

    /**
     * Instance being reused for performance reasons and containing delta device attitude.
     */
    private val deltaAttitude = Quaternion()

    /**
     * Delta translation respect previous event.
     */
    private val deltaTranslation = InhomogeneousPoint3D()

    /**
     * Transformation containing attitude respect Earth and relative translation.
     */
    private val transformation = EuclideanTransformation3D()

    /**
     * Transformation containing delta attitude and translation.
     */
    private val deltaTransformation = EuclideanTransformation3D()

    /**
     * Instance being reused for performance reasons and containing device attitude expressed in
     * NED coordinate system.
     */
    private val rotationMatrix = Matrix(
        MatrixRotation3D.ROTATION3D_INHOM_MATRIX_ROWS,
        MatrixRotation3D.ROTATION3D_INHOM_MATRIX_COLS
    )

    /**
     * End position to be reused and expressed in ECEF coordinates.
     */
    private val endEcefPosition = ECEFPosition()

    /**
     * End position to be reused and expressed in NED coordinates.
     */
    private val endNedPosition = NEDPosition()

    /**
     * End velocity to be reused and expressed in NED coordinates.
     */
    private val endVelocity = NEDVelocity()

    /**
     * Internal listener to handle sensor events.
     */
    override val sensorEventListener = object : SensorEventListener {
        override fun onSensorChanged(event: SensorEvent?) {
            if (event == null) {
                return
            }

            if (event.sensor.type != Sensor.TYPE_POSE_6DOF) {
                return
            }

            val sensorAccuracy = SensorAccuracy.from(event.accuracy)
            val timestamp = event.timestamp

            val startPosition = this@RelativeDevicePoseSensorCollector.startPosition
            if (startPosition != null) {
                PoseHelper.convertToNED(
                    event.values,
                    startPosition,
                    frame,
                    attitude,
                    translation,
                    deltaAttitude,
                    deltaTranslation,
                    transformation,
                    deltaTransformation,
                    rotationMatrix,
                    endEcefPosition,
                    endNedPosition,
                    endVelocity,
                    timeInterval
                )
            } else {
                PoseHelper.convertToNED(
                    event.values,
                    attitude,
                    translation,
                    deltaAttitude,
                    deltaTranslation,
                    transformation,
                    deltaTransformation
                )
            }

            measurementListener?.onMeasurement(
                attitude,
                translation,
                if (startPosition != null) frame else null,
                transformation,
                timestamp,
                sensorAccuracy
            )
        }

        override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) {
            if (sensor == null) {
                return
            }

            if (sensor.type != Sensor.TYPE_POSE_6DOF) {
                return
            }

            val sensorAccuracy = SensorAccuracy.from(accuracy)
            accuracyChangedListener?.onAccuracyChanged(sensorAccuracy)
        }
    }

    /**
     * Current location.
     * If provided, frame containing absolute position and attitude will be estimated.
     */
    var location: Location? = null
        set(value) {
            field = value
            startPosition = value?.toNEDPosition()
        }

    /**
     * Sensor being used to obtain measurements or null if not available.
     * This can be used to obtain additional information about the sensor.
     *
     * @see sensorAvailable
     */
    override val sensor: Sensor? by lazy { sensorManager?.getDefaultSensor(Sensor.TYPE_POSE_6DOF) }

    /**
     * Interface to notify when a new device pose is available.
     */
    fun interface OnMeasurementListener {
        /**
         * Called when a new device pose is available.
         *
         * @param attitude measured attitude respect Earth.
         * @param translation relative translation.
         * @param frame if enabled, returns frame containing current absolute position, attitude
         * and velocity respect Earth.
         * @param transformation 3D euclidean transformation containing current attitude and
         * relative translation.
         * @param timestamp time in nanoseconds at which the measurement was made. Each measurement
         * will be monotonically increasing using the same time base as
         * [android.os.SystemClock.elapsedRealtimeNanos].
         * @param accuracy sensor accuracy.
         */
        fun onMeasurement(
            attitude: Quaternion,
            translation: Point3D,
            frame: NEDFrame?,
            transformation: EuclideanTransformation3D?,
            timestamp: Long,
            accuracy: SensorAccuracy?
        )
    }

    /**
     * Interface to notify when sensor accuracy changes.
     */
    fun interface OnAccuracyChangedListener {
        /**
         * Called when accuracy changes.
         *
         * @param accuracy new accuracy.
         */
        fun onAccuracyChanged(accuracy: SensorAccuracy?)
    }
}