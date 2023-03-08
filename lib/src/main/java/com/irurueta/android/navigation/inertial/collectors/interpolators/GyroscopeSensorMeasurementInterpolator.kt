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

import com.irurueta.android.navigation.inertial.collectors.GyroscopeSensorMeasurement

/**
 * Gyroscope interpolator.
 */
interface GyroscopeSensorMeasurementInterpolator {

    /**
     * Pushes previous measurement into collection of processed measurements.
     *
     * @param previousMeasurement previous measurement to be pushed.
     */
    fun push(previousMeasurement: GyroscopeSensorMeasurement)

    /**
     * Interpolates provided current measurement.
     *
     * @param currentMeasurement current measurement to be interpolated with previous ones.
     * @param result instance where result of interpolation will be stored.
     * @param timestamp timestamp to perform interpolation respect previous measurements.
     * @return true if interpolation has been computed and result instance contains expected value,
     * false if result of interpolation must be discarded.
     */
    fun interpolate(
        currentMeasurement: GyroscopeSensorMeasurement,
        timestamp: Long,
        result: GyroscopeSensorMeasurement
    ): Boolean

    /**
     * Resets this interpolator.
     */
    fun reset()
}