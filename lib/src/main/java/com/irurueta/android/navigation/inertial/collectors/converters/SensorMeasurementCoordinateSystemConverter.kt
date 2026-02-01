package com.irurueta.android.navigation.inertial.collectors.converters

import com.irurueta.android.navigation.inertial.collectors.measurements.SensorMeasurement

/**
 * Converts sensor measurements between NED (North-East-Down) and ENU (East-North-Up)
 * coordinates systems.
 *
 * @see com.irurueta.android.navigation.inertial.collectors.measurements.SensorMeasurement for more information about sensor measurements.
 */
interface SensorMeasurementCoordinateSystemConverter<T: SensorMeasurement<T>> {

    /**
     * Converts provided sensor measurement to NED coordinates system and stores the result
     * in the provided output instance.
     *
     * @param input input measurement to be converted.
     * @param output instance where converted measurement will be stored.
     * @throws IllegalArgumentException if input measurements is not expressed in ENU coordinates
     * system.
     */
    @Throws(IllegalArgumentException::class)
    fun toNedOrThrow(input: T, output: T)

    /**
     * Converts provided sensor measurement to NED coordinates system and returns a new
     * instance containing the result.
     *
     * @param input input measurement to be converted.
     * @return a new instance containing converted measurement.
     * @throws IllegalArgumentException if input measurements is not expressed in ENU coordinates
     * system.
     */
    @Throws(IllegalArgumentException::class)
    fun toNedOrThrow(input: T) : T

    /**
     * Converts provided sensor measurement to ENU coordinates system and stores the result
     * in the provided output instance.
     *
     * @param input input measurement to be converted.
     * @param output instance where converted measurement will be stored.
     * @throws IllegalArgumentException if input measurements is not expressed in NED coordinates
     * system.
     */
    @Throws(IllegalArgumentException::class)
    fun toEnuOrThrow(input: T, output: T)

    /**
     * Converts provided sensor measurement to ENU coordinates system and returns a new
     * instance containing the result.
     *
     * @param input input measurement to be converted.
     * @return a new instance containing converted measurement.
     * @throws IllegalArgumentException if input measurements is not expressed in NED coordinates
     * system.
     */
    @Throws(IllegalArgumentException::class)
    fun toEnuOrThrow(input: T): T

    /**
     * Converts provided sensor measurement to NED coordinates system and stores the result
     * in the provided output instance.
     * If the measurement is already expressed in NED coordinates system, input measurement
     * is copied into output instance.
     *
     * @param input input measurement to be converted.
     * @param output instance where converted measurement will be stored.
     */
    fun toNed(input: T, output: T)

    /**
     * Converts provided sensor measurement to NED coordinates system and returns a new
     * instance containing the result.
     * If the measurement is already expressed in NED coordinates system, a copy of input
     * measurement is returned.
     *
     * @param input input measurement to be converted.
     * @return a new instance containing converted measurement.
     */
    fun toNed(input: T): T

    /**
     * Converts provided sensor measurement to ENU coordinates system and stores the result
     * in the provided output instance.
     * If the measurement is already expressed in ENU coordinates system, input measurement
     * is copied into output instance.
     *
     * @param input input measurement to be converted.
     * @param output instance where converted measurement will be stored.
     */
    fun toEnu(input: T, output: T)

    /**
     * Converts provided sensor measurement to ENU coordinates system and returns a new
     * instance containing the result.
     * If the measurement is already expressed in ENU coordinates system, a copy of input
     * measurement is returned.
     *
     * @param input input measurement to be converted.
     * @return a new instance containing converted measurement.
     */
    fun toEnu(input: T): T
}