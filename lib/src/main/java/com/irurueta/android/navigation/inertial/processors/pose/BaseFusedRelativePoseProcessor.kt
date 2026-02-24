/*
 * Copyright (C) 2026 Alberto Irurueta Carro (alberto@irurueta.com)
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

package com.irurueta.android.navigation.inertial.processors.pose

import android.location.Location
import com.irurueta.android.navigation.inertial.collectors.measurements.GyroscopeSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.SensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.SyncedSensorMeasurement
import com.irurueta.android.navigation.inertial.processors.attitude.BaseLeveledRelativeAttitudeProcessor
import com.irurueta.navigation.inertial.calibration.AccelerationTriad
import com.irurueta.navigation.inertial.calibration.SpeedTriad

/**
 * Base class to estimate relative pose from an unknown location.
 * This class estimates device attitude by fusing gravity/accelerometer, gyroscope and magnetometer
 * measurements.
 * Accelerometer and gyroscope are then taken into account to update device position.
 *
 * @property initialSpeed initial device speed in body coordinates.
 * @property processorListener listener to notify new poses.
 */
abstract class BaseFusedRelativePoseProcessor<M : SensorMeasurement<M>, S: SyncedSensorMeasurement<S>>(
    initialSpeed: SpeedTriad,
    processorListener: OnProcessedListener?
) : BaseRelativePoseProcessor(initialSpeed, processorListener) {

    /**
     * Attitude processor in charge of fusing accelerometer/gravity + gyroscope and magnetometer
     * measurements to estimate current device attitude.
     */
    protected abstract val attitudeProcessor: BaseLeveledRelativeAttitudeProcessor<M, *>

    /**
     * X-coordinates of last sensed gravity component of specific force expressed in NED coordinates
     * and in meters per squared second (m/s^2).
     */
    val gx: Double
        get() = attitudeProcessor.gx

    /**
     * Y-coordinate of last sensed gravity component of specific force expressed in NED coordinates
     * and in meters per squared second (m/s^2).
     */
    val gy: Double
        get() = attitudeProcessor.gy

    /**
     * Z-coordinate of last sensed gravity component of specific force expressed in NED coordinates
     * and in meters per squared second (m/s^2).
     */
    val gz: Double
        get() = attitudeProcessor.gz

    /**
     * Updates provided triad to contain gravity component of specific force expressed in NED
     * coordinates and in meters per squared second (m/s^2).
     */
    fun getGravity(result: AccelerationTriad) {
        attitudeProcessor.getGravity(result)
    }

    /**
     * Indicates whether accurate leveling is used or not.
     */
    val useAccurateLevelingProcessor: Boolean
        get() = attitudeProcessor.useAccurateLevelingProcessor

    /**
     * Indicates whether accurate non-leveled relative attitude processor must be used or not.
     */
    var useAccurateRelativeGyroscopeAttitudeProcessor: Boolean
        get() = attitudeProcessor.useAccurateRelativeGyroscopeAttitudeProcessor
        set(value) {
            attitudeProcessor.useAccurateRelativeGyroscopeAttitudeProcessor = value
        }

    /**
     * Indicates whether fusion between leveling and relative attitudes occurs based
     * on changing interpolation value that depends on actual relative attitude rotation
     * velocity.
     */
    var useIndirectAttitudeInterpolation: Boolean
        get() = attitudeProcessor.useIndirectInterpolation
        set(value) {
            attitudeProcessor.useIndirectInterpolation = value
        }

    /**
     * Interpolation value to be used to combine both leveling and relative attitudes.
     * Must be between 0.0 and 1.0 (both included).
     * The closer to 0.0 this value is, the more resemblance the result will have to a pure
     * leveling (which feels more jerky). On the contrary, the closer to 1.0 this value is,
     * the more resemblance the result will have to a pure non-leveled relative attitude (which
     * feels softer but might have arbitrary roll and pitch Euler angles).
     *
     * @throws IllegalArgumentException if value is not between 0.0 and 1.0 (both included).
     */
    var attitudeInterpolationValue: Double
        get() = attitudeProcessor.interpolationValue
        @Throws(IllegalArgumentException::class)
        set(value) {
            attitudeProcessor.interpolationValue = value
        }

    /**
     * Factor to take into account when interpolation value is computed and
     * [useIndirectAttitudeInterpolation] is enabled to determine actual interpolation value based
     * on current relative attitude rotation velocity.
     *
     * @throws IllegalArgumentException if value is zero or negative.
     */
    var attitudeIndirectInterpolationWeight: Double
        get() = attitudeProcessor.indirectInterpolationWeight
        @Throws(IllegalArgumentException::class)
        set(value) {
            attitudeProcessor.indirectInterpolationWeight = value
        }

    /**
     * Threshold to determine that current geomagnetic attitude appears to be an outlier respect
     * to estimated fused attitude.
     * When geomagnetic attitude and fused attitudes diverge, fusion is not performed, and instead
     * only gyroscope relative attitude is used for fusion estimation.
     *
     * @throws IllegalArgumentException if value is not between 0.0 and 1.0 (both included).
     */
    var attitudeOutlierThreshold: Double
        get() = attitudeProcessor.outlierThreshold
        @Throws(IllegalArgumentException::class)
        set(value) {
            attitudeProcessor.outlierThreshold = value
        }

    /**
     * Threshold to determine that geomagnetic attitude has largely diverged and if situation is not
     * reverted soon, attitude will be reset to geomagnetic one.
     *
     * @throws IllegalArgumentException if value is not between 0.0 and 1.0 (both included).
     */
    var attitudeOutlierPanicThreshold: Double
        get() = attitudeProcessor.outlierPanicThreshold
        @Throws(IllegalArgumentException::class)
        set(value) {
            attitudeProcessor.outlierPanicThreshold = value
        }

    /**
     * Threshold to determine when fused attitude has largely diverged for a given
     * number of samples and must be reset.
     *
     * @throws IllegalArgumentException if provided value is zero or negative.
     */
    var attitudePanicCounterThreshold: Int
        get() = attitudeProcessor.panicCounterThreshold
        @Throws(IllegalArgumentException::class)
        set(value) {
            attitudeProcessor.panicCounterThreshold = value
        }

    /**
     * Gets or sets device location
     */
    override var location: Location?
        get() = attitudeProcessor.location
        set(value) {
            attitudeProcessor.location = value
        }

    /**
     * Indicates whether gravity norm must be adjusted to either Earth
     * standard norm, or norm at provided location. If no location is provided, this should only be
     * enabled when device is close to sea level.
     */
    override var adjustGravityNorm: Boolean
        get() = attitudeProcessor.adjustGravityNorm
        set(value) {
            attitudeProcessor.adjustGravityNorm = value
        }

    /**
     * Resets internal parameters.
     */
    override fun reset() {
        super.reset()
        attitudeProcessor.reset()
    }

    /**
     * Processes measurements to estimate current attitude.
     *
     * @param accelerometerOrGravityMeasurement accelerometer or gravity measurement.
     * @param gyroscopeMeasurement gyroscope measurement.
     * @param timestamp timestamp when all measurements are assume to occur.
     * @return true if new fused absolute attitude is processed, false otherwise.
     */
    protected fun processAttitude(
        accelerometerOrGravityMeasurement: M,
        gyroscopeMeasurement: GyroscopeSensorMeasurement,
        timestamp: Long
    ): Boolean {
        return if (attitudeProcessor.process(
                accelerometerOrGravityMeasurement,
                gyroscopeMeasurement,
                timestamp
            )
        ) {
            attitudeProcessor.fusedAttitude.copyTo(currentAttitude)
            attitudeProcessor.getGravity(gravity)
            true
        } else {
            false
        }
    }
}