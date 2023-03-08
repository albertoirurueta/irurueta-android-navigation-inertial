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

import com.irurueta.android.navigation.inertial.collectors.SensorMeasurement

/**
 * Implementation of an interpolator that makes no action.
 *
 * @param <M> Type of sensor measurement.
 */
abstract class DirectSensorMeasurementInterpolator<M : SensorMeasurement<M>> :
    SensorMeasurementInterpolator<DirectSensorMeasurementInterpolator<M>, M> {

    /**
     * Pushes previous measurement into collection of processed measurements.
     * This implementation makes no action.
     *
     * @param previousMeasurement previous measurement to be pushed.
     */
    override fun push(previousMeasurement: M) {
        // makes no action
    }

    /**
     * Interpolates provided current measurement.
     * This implementation always returns true and returns as a result a copy of provide current
     * measurement.
     *
     * @param currentMeasurement current measurement to be interpolated with previous ones.
     * @param timestamp timestamp to perform interpolation respect previous measurements.
     * @param result instance where result of interpolation will be stored. If will contain a copy
     * of [currentMeasurement].
     * @return true if interpolation has been computed and result instance contains expected value,
     * false if result of interpolation must be discarded. This implementation always returns true.
     */
    override fun interpolate(currentMeasurement: M, timestamp: Long, result: M): Boolean {
        result.copyFrom(currentMeasurement)
        return true
    }

    /**
     * Resets this interpolator.
     */
    override fun reset() {
        // makes no action
    }
}