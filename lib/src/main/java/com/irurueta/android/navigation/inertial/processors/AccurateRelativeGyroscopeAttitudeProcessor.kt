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
package com.irurueta.android.navigation.inertial.processors

import com.irurueta.android.navigation.inertial.collectors.GyroscopeSensorMeasurement
import com.irurueta.navigation.inertial.calibration.gyroscope.QuaternionStepIntegrator
import com.irurueta.navigation.inertial.calibration.gyroscope.QuaternionStepIntegratorType

/**
 * Estimates relative attitude of device respect to an arbitrary initial attitude using gyroscope
 * measurements only.
 * This processor uses Runge-Kutta integration to obtain greater accuracy than
 * [RelativeGyroscopeAttitudeProcessor] at the expense of additional cpu usage.
 *
 * @property processorListener listener to notify new relative attitudes.
 */
class AccurateRelativeGyroscopeAttitudeProcessor(processorListener: OnProcessedListener? = null) :
    BaseRelativeGyroscopeAttitudeProcessor(processorListener) {

    /**
     * Previous x-coordinate angular speed expressed in radians per second (rad/s).
     */
    private var previousWx = 0.0

    /**
     * Previous y-coordinate angular speed expressed in radians per second (rad/s).
     */
    private var previousWy = 0.0

    /**
     * Previous z-coordinate angular speed expressed in radians per second (rad/s).
     */
    private var previousWz = 0.0

    /**
     * Integrates new gyroscope measurements into existing attitude.
     */
    private val quaternionStepIntegrator = QuaternionStepIntegrator.create(
        QuaternionStepIntegratorType.RUNGE_KUTTA
    )

    /**
     * Processes a gyroscope sensor measurement to integrate angular speed values to obtain
     * an accumulated attitude from an arbitrary attitude of origin.
     *
     * @param measurement gyroscope measurement expressed in ENU android coordinates system to be
     * processed
     * @return true if a new relative attitude is estimated, false otherwise.
     */
    override fun process(measurement: GyroscopeSensorMeasurement): Boolean {
        val isFirst = updateTimeInterval(measurement)

        val result = if (!isFirst) {
            updateTriad(measurement)

            val dt = timeIntervalEstimator.averageTimeInterval

            quaternionStepIntegrator.integrate(
                internalAttitude,
                previousWx,
                previousWy,
                previousWz,
                triad.valueX,
                triad.valueY,
                triad.valueZ,
                dt,
                internalAttitude
            )

            internalAttitude.copyTo(attitude)

            processorListener?.onProcessed(this, attitude)
            true
        } else {
            false
        }

        previousWx = triad.valueX
        previousWy = triad.valueY
        previousWz = triad.valueZ

        return result
    }
}