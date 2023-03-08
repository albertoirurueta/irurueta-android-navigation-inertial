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
package com.irurueta.android.navigation.inertial.collectors.interpolators

import com.irurueta.android.navigation.inertial.collectors.AttitudeSensorMeasurement
import com.irurueta.geometry.Quaternion
import kotlin.math.abs
import kotlin.math.ceil
import kotlin.math.max

/**
 * Attitude linear interpolator.
 *
 * @property copyIfNotInitialized true indicates that measurement is copied into result if
 * interpolator is not completely initialized.
 */
class AttitudeLinearSensorMeasurementInterpolator(copyIfNotInitialized: Boolean = true) :
    LinearSensorMeasurementInterpolator<AttitudeSensorMeasurement>(copyIfNotInitialized),
    AttitudeSensorMeasurementInterpolator {

    /**
     * Oldest pushed measurement.
     */
    override val measurement0 = AttitudeSensorMeasurement()

    /**
     * Newest pushed measurement.
     */
    override val measurement1 = AttitudeSensorMeasurement()

    /**
     * Inverse of attitude contained in [measurement0]. This is used to estimate [delta] of attitude
     * between [measurement0] and [measurement1].
     */
    private val inverseAttitude0 = Quaternion()

    /**
     * Variation of attitude between [measurement0] and [measurement1].
     */
    private val delta = Quaternion()

    /**
     * Delta attitude multiplied by a number of times to perform a slerp interpolation.
     */
    private val timesDelta = Quaternion()

    /**
     * Delta attitude multiplied by a number of times - 1 to perform a slerp interpolation.
     */
    private val timesMinusOneDelta = Quaternion()

    /**
     * Start attitude obtained by multiplying [measurement0] attitude by [timesMinusOneDelta]. A
     * slerp interpolation is computed from [start] and [end] attitudes.
     */
    private val start = Quaternion()

    /**
     * End attitude obtained by multiplying [measurement0] attitude by [timesDelta]. A slerp
     * interpolation is computed from [start] and [end] attitudes.
     */
    private val end = Quaternion()

    /**
     * Linearly interpolates [measurement0] and [measurement1] with provided factor and stores
     * the result of interpolation into provided result measurement.
     *
     * @param measurement0 oldest pushed measurement to linearly interpolate from.
     * @param measurement1 newest pushed measurement to linearly interpolate from.
     * @param factor factor to interpolate measurements with.
     * @param timestamp timestamp to be set into result instance.
     * @param result instance where result of interpolation is stored.
     */
    override fun interpolate(
        measurement0: AttitudeSensorMeasurement,
        measurement1: AttitudeSensorMeasurement,
        factor: Double,
        timestamp: Long,
        result: AttitudeSensorMeasurement
    ) {
        val attitude0 = measurement0.attitude
        val attitude1 = measurement1.attitude

        // q1 = delta * q0 --> delta =  q1 * q0^-1
        attitude0.normalize()
        attitude1.normalize()
        attitude0.inverse(inverseAttitude0)
        inverseAttitude0.normalize()

        Quaternion.product(attitude1, inverseAttitude0, delta)
        delta.normalize()

        if (factor < 0.0) {
            delta.inverse()
            delta.normalize()
        }

        val absFactor = abs(factor)
        val ceilFactor = ceil(absFactor)
        val times = max(1, ceilFactor.toInt())
        resetQuaternion(timesDelta)
        resetQuaternion(timesMinusOneDelta)
        for (i in 1  .. times) {
            Quaternion.product(timesDelta, delta, timesDelta)
            timesDelta.normalize()
            if (i < times) {
                Quaternion.product(timesMinusOneDelta, delta, timesMinusOneDelta)
                timesMinusOneDelta.normalize()
            }
        }

        // q2 = timesDelta * q0
        Quaternion.product(timesDelta, attitude0, end)
        end.normalize()

        Quaternion.product(timesMinusOneDelta, attitude0, start)
        start.normalize()

        // interpolate attitude
        val fraction = if (absFactor > 1.0) {
            ceilFactor - absFactor
        } else {
            absFactor
        }
        Quaternion.slerp(start, end, fraction, result.attitude)
        result.attitude.normalize()

        val headingAccuracy0 = measurement0.headingAccuracy
        val headingAccuracy1 = measurement1.headingAccuracy
        result.headingAccuracy = if (headingAccuracy0 != null && headingAccuracy1 != null) {
            interpolate(headingAccuracy0, headingAccuracy1, factor)
        } else {
            null
        }
    }

    private companion object {

        /**
         * Resets quaternion so it contains no rotation.
         *
         * @param quaternion quaternion to be reset
         */
        private fun resetQuaternion(quaternion: Quaternion) {
            quaternion.a = 1.0
            quaternion.b = 0.0
            quaternion.c = 0.0
            quaternion.d = 0.0
            quaternion.normalize()
        }
    }
}