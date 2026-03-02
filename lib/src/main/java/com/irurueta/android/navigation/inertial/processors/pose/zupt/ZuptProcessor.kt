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

package com.irurueta.android.navigation.inertial.processors.pose.zupt

import android.hardware.SensorManager
import android.location.Location
import com.irurueta.android.navigation.inertial.collectors.measurements.AccelerometerSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.GyroscopeSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.SyncedSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.WithAccelerometerSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.WithGyroscopeSensorMeasurement
import com.irurueta.android.navigation.inertial.toNEDPosition
import com.irurueta.navigation.frames.NEDPosition
import com.irurueta.navigation.inertial.NEDGravity
import com.irurueta.navigation.inertial.calibration.AccelerationTriad
import com.irurueta.navigation.inertial.calibration.AngularSpeedTriad
import com.irurueta.navigation.inertial.estimators.NEDGravityEstimator

/**
 * Base class for ZUPT processors.
 * A ZUPT processor determines whether Velocity should be updated to zero depending on
 * certain conditions on measurements (related to accelerometer magnitude and accelerometer and
 * gyroscope variance)
 *
 * @param location current device location. If provided, precise expected gravity is estimated,
 * otherwise default Earth gravity is used.
 * @param windowNanoseconds amount of time to take measurements into account to compute variance
 * and average of accelerometer and gyroscope measurements. Value is expressed in nanoseconds.
 */
abstract class ZuptProcessor<T>(
    var location: Location? = null,
    windowNanoseconds: Long = DEFAULT_WINDOW_NANOSECONDS
) where T : SyncedSensorMeasurement<T>, T : WithAccelerometerSensorMeasurement {
    /**
     * Gravity expressed in NED coordinates.
     */
    private val nedGravity = NEDGravity()

    /**
     * Position expressed in NED coordinates. Only used if location is provided.
     */
    private val nedPosition = NEDPosition()

    /**
     * Estimates accelerometer average and variance on a given time window.
     */
    private val accelerometerAverageAndVarianceProcessor =
        AverageAndVarianceSensorMeasurementProcessor<AccelerometerSensorMeasurement, AccelerationTriad>(
            windowNanoseconds
        )

    /**
     * Estimates gyroscope average and variance on a given time window.
     */
    private val gyroscopeAverageAndVarianceProcessor =
        AverageAndVarianceSensorMeasurementProcessor<GyroscopeSensorMeasurement, AngularSpeedTriad>(
            windowNanoseconds
        )

    /**
     * Amount of time to take measurements into account to compute variance and average of
     * measurements. Value is expressed in nanoseconds.
     *
     * @throws IllegalArgumentException if provided value is negative.
     */
    var windowNanoseconds: Long
        get() = accelerometerAverageAndVarianceProcessor.windowNanoseconds
        @Throws(IllegalArgumentException::class)
        set(value) {
            accelerometerAverageAndVarianceProcessor.windowNanoseconds = value
        }

    /**
     * Average acceleration magnitude.
     */
    var accelerationAverage: Double? = null
        private set

    /**
     * Variance of acceleration.
     */
    var accelerationVariance: Double? = null
        private set

    /**
     * Average gyroscope magnitude.
     */
    var gyroscopeAverage: Double? = null
        private set

    /**
     * Variance of gyroscope.
     */
    var gyroscopeVariance: Double? = null

    /**
     * Gets expected gravity norm at provided location.
     * If No location is provided, average gravity at sea level is returned instead.
     */
    val expectedGravityNorm: Double
        get() {
            val location = this.location
            return if (location != null) {
                location.toNEDPosition(nedPosition)
                NEDGravityEstimator.estimateGravity(nedPosition, nedGravity)
                nedGravity.norm
            } else {
                GRAVITY_EARTH
            }
        }

    /**
     * Estimated ZUPT score for last processed synced measurement.
     */
    var zuptScore: Double = 0.0
        private set

    /**
     * Processes synced measurement to determine whether ZUPT (Zero Velocity Update) should be
     * made or not.
     *
     * @return A value between 0.0 and 1.0 indicating whether ZUPT should be made or not. 0.0
     * indicates that ZUPT should not be made, while 1.0 indicates that ZUPT should be made.
     */
    open fun process(syncedMeasurement: T): Double {
        val accelerometerMeasurement = syncedMeasurement.accelerometerMeasurement ?: return 0.0

        processAccelerometer(accelerometerMeasurement)

        if (syncedMeasurement is WithGyroscopeSensorMeasurement) {
            val gyroscopeMeasurement = syncedMeasurement.gyroscopeMeasurement
            if (gyroscopeMeasurement != null) {
                processGyroscope(gyroscopeMeasurement)

                zuptScore = evaluateAccelerometerAndGyroscope()
                return zuptScore
            }
        }

        zuptScore = evaluateAccelerometer()
        return zuptScore
    }

    /**
     * Evaluates accelerometer average magnitude and variance to determine whether ZUPT (Zero
     * Velocity Update) conditions apply or not.
     *
     * @return 1.0 if ZUPT conditions apply, 0.0 otherwise.
     *
     */
    protected abstract fun evaluateAccelerometer(): Double

    /**
     * Evaluates accelerometer average magnitude and variance, and gyroscope variance to
     * determine whether ZUPT (Zero Velocity Update) conditions apply or not.
     *
     * @return 1.0 if ZUPT conditions apply, 0.0 otherwise.
     */
    protected abstract fun evaluateAccelerometerAndGyroscope(): Double

    /**
     * Processes synced mseasurement to determine whether ZUPT (Zero Volocity Update) should be
     * made or not in a non-soft fashion.
     *
     * @return true indicates that ZUPT should be made, false otherwise.
     */
    fun processNonSoft(syncedMeasurement: T): Boolean {
        return process(syncedMeasurement) > 0.5
    }

    /**
     * Processes accelerometer measurement.
     */
    protected fun processAccelerometer(measurement: AccelerometerSensorMeasurement) {
        accelerometerAverageAndVarianceProcessor.process(measurement)
        accelerationAverage = accelerometerAverageAndVarianceProcessor.average
        accelerationVariance = accelerometerAverageAndVarianceProcessor.variance
    }

    /**
     * Processes gyroscope measurement.
     */
    protected fun processGyroscope(measurement: GyroscopeSensorMeasurement) {
        gyroscopeAverageAndVarianceProcessor.process(measurement)
        gyroscopeAverage = gyroscopeAverageAndVarianceProcessor.average
        gyroscopeVariance = gyroscopeAverageAndVarianceProcessor.variance
    }

    companion object {
        /**
         * Default amount of time to take measurements into account to compute variance
         * of accelerometer and gyroscope measurements.
         * By default, this is 200 ms (0.2s), which is about 10 samples at a rate of 50Hz.
         */
        const val DEFAULT_WINDOW_NANOSECONDS = 200_000_000L

        /**
         * Constant defining average acceleration due to gravity at Earth's sea level.
         * This is roughly 9.81 m/s¨2.
         */
        const val GRAVITY_EARTH = SensorManager.GRAVITY_EARTH.toDouble()
    }
}