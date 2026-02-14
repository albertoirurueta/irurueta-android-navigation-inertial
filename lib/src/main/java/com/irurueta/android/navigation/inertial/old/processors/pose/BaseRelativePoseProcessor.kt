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
package com.irurueta.android.navigation.inertial.old.processors.pose

import android.location.Location
import com.irurueta.algebra.Matrix
import com.irurueta.android.navigation.inertial.ENUtoNEDConverter
import com.irurueta.android.navigation.inertial.collectors.measurements.AccelerometerSensorMeasurement
import com.irurueta.android.navigation.inertial.old.estimators.pose.SpeedTriad
import com.irurueta.geometry.EuclideanTransformation3D
import com.irurueta.geometry.InhomogeneousPoint3D
import com.irurueta.geometry.Quaternion
import com.irurueta.navigation.inertial.calibration.AccelerationTriad
import com.irurueta.units.TimeConverter

/**
 * Base class to estimate relative pose from an unknown location.
 *
 * @property initialSpeed initial device speed in body coordinates.
 * @property processorListener listener to notify new poses.
 */
abstract class BaseRelativePoseProcessor(
    val initialSpeed: SpeedTriad,
    var processorListener: OnProcessedListener?
) {
    /**
     * Specific force measured by accelerometer sensor containing both device acceleration and
     * gravity component.
     */
    private val specificForce = AccelerationTriad()

    /**
     * Indicates whether estimator has been initialized.
     */
    private var initialized = false

    /**
     * Previous attitude in last sample.
     */
    private val previousAttitude = Quaternion()

    /**
     * Attitude when estimator starts.
     */
    private val initialAttitude = Quaternion()

    /**
     * Average attitude between previous and current attitudes.
     */
    private val averageAttitude = Quaternion()

    /**
     * Contains device acceleration in NED body coordinates. This is reused for performance reasons.
     */
    private val abb = Matrix(AccelerationTriad.COMPONENTS, 1)

    /**
     * Contains average attitude expressed in matrix form. This is reused for performance reasons.
     */
    private val avgAttitudeMatrix = Matrix(Quaternion.N_ANGLES, Quaternion.N_ANGLES)

    /**
     * Contains device acceleration in NED coordinates. This is reused for performance reasons.
     */
    private val abn = Matrix(AccelerationTriad.COMPONENTS, 1)

    /**
     * Previous speed in last sample.
     */
    private val previousSpeed = SpeedTriad()

    /**
     * Current speed.
     */
    private val currentSpeed = SpeedTriad()

    /**
     * Previous position in last sample.
     */
    private val previousPosition = InhomogeneousPoint3D()

    /**
     * Current position.
     */
    private val currentPosition = InhomogeneousPoint3D()

    /**
     * Timestamp of previous sample expressed in nanoseconds.
     */
    private var previousTimestamp: Long = -1L

    /**
     * Current attitude.
     */
    protected val currentAttitude = Quaternion()

    /**
     * Current estimated specific force related to gravity at current location.
     */
    protected val gravity = AccelerationTriad()

    /**
     * Time interval expressed in seconds between consecutive gyroscope measurements
     */
    var timeIntervalSeconds = 0.0
        private set

    /**
     * Contains current pose respect to the start of this estimator.
     */
    val poseTransformation = EuclideanTransformation3D()

    /**
     * Gets or sets device location
     */
    abstract var location: Location?

    /**
     * Indicates whether gravity norm must be adjusted to either Earth
     * standard norm, or norm at provided location. If no location is provided, this should only be
     * enabled when device is close to sea level.
     */
    abstract var adjustGravityNorm: Boolean

    /**
     * Resets internal parameters.
     */
    open fun reset() {
        initialized = false
        previousTimestamp = -1L
        timeIntervalSeconds = 0.0
    }

    /**
     * Processes pose by taking into account estimated current attitude, accelerometer and
     * gyroscope measurements to estimate new position and velocity from previous ones.
     *
     * @param accelerometerMeasurement accelerometer measurement.
     * @param timestamp timestamp when all measurements are assumed to occur.
     * @return true if new pose is processed, false otherwise.
     */
    protected fun processPose(
        accelerometerMeasurement: AccelerometerSensorMeasurement,
        timestamp: Long
    ): Boolean {
        if (!processTimeInterval(timestamp)) {
            return false
        }

        initializeIfNeeded(currentAttitude)

        processAccelerometer(accelerometerMeasurement)

        // obtain average attitude between current and previous attitude
        Quaternion.slerp(previousAttitude, currentAttitude, 0.5, averageAttitude)
        averageAttitude.normalize()

        // transform specific force, gravity and acceleration to NED frame
        val fbx = specificForce.valueX
        val fby = specificForce.valueY
        val fbz = specificForce.valueZ
        val gbx = gravity.valueX
        val gby = gravity.valueY
        val gbz = gravity.valueZ
        val abx = fbx - gbx
        val aby = fby - gby
        val abz = fbz - gbz
        abb.setElementAtIndex(0, abx)
        abb.setElementAtIndex(1, aby)
        abb.setElementAtIndex(2, abz)

        averageAttitude.asInhomogeneousMatrix(avgAttitudeMatrix)
        avgAttitudeMatrix.multiply(abb, abn)

        val ax = abn.getElementAtIndex(0)
        val ay = abn.getElementAtIndex(1)
        val az = abn.getElementAtIndex(2)

        // Update velocity
        val oldVx = previousSpeed.valueX
        val oldVy = previousSpeed.valueY
        val oldVz = previousSpeed.valueZ

        val newVx = oldVx + ax * timeIntervalSeconds
        val newVy = oldVy + ay * timeIntervalSeconds
        val newVz = oldVz + az * timeIntervalSeconds
        currentSpeed.setValueCoordinates(newVx, newVy, newVz)

        // Update position
        val oldX = previousPosition.inhomX
        val oldY = previousPosition.inhomY
        val oldZ = previousPosition.inhomZ

        val newX = oldX + 0.5 * (oldVx + newVx) * timeIntervalSeconds
        val newY = oldY + 0.5 * (oldVy + newVy) * timeIntervalSeconds
        val newZ = oldZ + 0.5 * (oldVz + newVz) * timeIntervalSeconds
        currentPosition.setCoordinates(newX, newY, newZ)

        // compute transformation
        computeTransformation()

        // notify
        processorListener?.onProcessed(this, timestamp, poseTransformation)

        // update previous values
        currentAttitude.copyTo(previousAttitude)
        currentSpeed.copyTo(previousSpeed)
        previousPosition.setCoordinates(newX, newY, newZ)

        return true
    }

    /**
     * Processes current accelerometer measurement to obtain specific force expressed in NED
     * coordinates.
     *
     * @param measurement accelerometer measurement to be processed.
     */
    private fun processAccelerometer(measurement: AccelerometerSensorMeasurement) {
        val ax = measurement.ax.toDouble()
        val ay = measurement.ay.toDouble()
        val az = measurement.az.toDouble()
        val bx = measurement.bx?.toDouble()
        val by = measurement.by?.toDouble()
        val bz = measurement.bz?.toDouble()

        val currentAx = if (bx != null) ax - bx else ax
        val currentAy = if (by != null) ay - by else ay
        val currentAz = if (bz != null) az - bz else az

        ENUtoNEDConverter.convert(currentAx, currentAy, currentAz, specificForce)
    }

    /**
     * Updates current time interval estimation between gyroscope measurements.
     *
     * @param timestamp optional timestamp that can be provided to override timestamp associated to
     * gyroscope measurement. If null, the timestamp from gyroscope measurement is used.
     * @return true if it is NOT the 1st measurement and time interval has been estimated, false
     * otherwise.
     */
    private fun processTimeInterval(timestamp: Long): Boolean {
        val isNotFirst = previousTimestamp > 0
        if (isNotFirst) {
            val diff = timestamp - previousTimestamp
            timeIntervalSeconds = TimeConverter.nanosecondToSecond(diff.toDouble())
        }
        previousTimestamp = timestamp

        return isNotFirst
    }

    /**
     * Keeps initial attitude.
     *
     * @param attitude first obtained attitude.
     */
    private fun initializeIfNeeded(attitude: Quaternion) {
        if (!initialized) {
            attitude.copyTo(initialAttitude)
            attitude.copyTo(previousAttitude)

            initialSpeed.copyTo(previousSpeed)

            // initial position is assumed to be at origin (zero).
            previousPosition.setCoordinates(0.0, 0.0, 0.0)

            initialized = true
        }
    }

    /**
     * Computes pose transformation using current attitude and position.
     */
    private fun computeTransformation() {
        // set transformation in NED coordinates
        poseTransformation.rotation.fromRotation(currentAttitude)
        poseTransformation.setTranslation(currentPosition)

        // convert transformation to ENU coordinates
        ENUtoNEDConverter.convert(poseTransformation, poseTransformation)
    }

    /**
     * Interface to notify when a new pose has been processed.
     */
    fun interface OnProcessedListener {
        /**
         * Called when a new pose is processed.
         *
         * @param processor processor that raised this event.
         * @param timestamp time in nanoseconds at which the measurement was made. Each measurement
         * will be monotonically increasing using the same time base as
         * [android.os.SystemClock.elapsedRealtimeNanos].
         * @param poseTransformation 3D metric transformation containing leveled attitude and
         * translation variation since this processor started expressed in ENU system of
         * coordinates.
         */
        fun onProcessed(
            processor: BaseRelativePoseProcessor,
            timestamp: Long,
            poseTransformation: EuclideanTransformation3D?
        )
    }
}