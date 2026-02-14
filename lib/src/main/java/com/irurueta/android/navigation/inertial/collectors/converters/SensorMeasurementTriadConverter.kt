package com.irurueta.android.navigation.inertial.collectors.converters

import com.irurueta.android.navigation.inertial.collectors.measurements.SensorMeasurement
import com.irurueta.navigation.inertial.calibration.Triad

/**
 * Converts a [com.irurueta.android.navigation.inertial.collectors.measurements.SensorMeasurement] into a [com.irurueta.navigation.inertial.calibration.Triad] of measurements.
 * Conversion keeps into account any available sensor bias data, and preserves coordinates system.
 *
 * @param SM a sensor measurement type.
 * @param T a triad type.
 */
interface SensorMeasurementTriadConverter<SM : SensorMeasurement<SM>,
        T : Triad<*, *, T>> {

    /**
     * Converts provided sensor measurement into a triad and stores the result
     * in the provided output instance.
     *
     * @param input input measurement to be converted.
     * @param output instance where converted measurement will be stored.
     */
    fun convert(input: SM, output: T)

    /**
     * Converts provided sensor measurement into a triad and returns a new
     * instance containing the result.
     *
     * @param input input measurement to be converted.
     * @return a new instance containing converted measurement.
     */
    fun convert(input: SM): T
}