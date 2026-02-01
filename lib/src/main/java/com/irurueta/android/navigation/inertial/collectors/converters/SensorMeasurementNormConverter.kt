package com.irurueta.android.navigation.inertial.collectors.converters

import com.irurueta.android.navigation.inertial.collectors.measurements.SensorMeasurement

/**
 * Converts a [com.irurueta.android.navigation.inertial.collectors.measurements.SensorMeasurement] into its norm value.
 * Conversion takes into account any available sensor bias data.
 *
 * @param T a sensor measurement type.
 */
interface SensorMeasurementNormConverter<T: SensorMeasurement<T>> {

    /**
     * Converts provided sensor measurement into its norm value.
     *
     * @param input input measurement to be converted.
     * @return norm value.
     */
    fun convert(input: T): Double
}