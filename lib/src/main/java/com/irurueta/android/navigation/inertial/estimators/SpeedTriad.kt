/*
 * Copyright (C) 2022 Alberto Irurueta Carro (alberto@irurueta.com)
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
package com.irurueta.android.navigation.inertial.estimators

import com.irurueta.navigation.inertial.calibration.Triad
import com.irurueta.units.Speed
import com.irurueta.units.SpeedConverter
import com.irurueta.units.SpeedUnit

/**
 * Contains a triad of speed measurements.
 *
 * @param unit speed unit for stored values.
 */
class SpeedTriad constructor(unit: SpeedUnit = DEFAULT_UNIT) : Triad<SpeedUnit, Speed>(unit),
    Cloneable {

    /**
     * Constructor.
     *
     * @param unit speed unit for stored values.
     * @param valueX x-coordinate of measurement value expressed in provided unit.
     * @param valueY y-coordinate of measurement value expressed in provided unit.
     * @param valueZ z-coordinate of measurement value expressed in provided unit.
     */
    constructor(
        unit: SpeedUnit,
        valueX: Double,
        valueY: Double,
        valueZ: Double
    ) : this(unit) {
        super.setValueCoordinates(valueX, valueY, valueZ)
    }

    /**
     * Constructor.
     *
     * @param valueX x-coordinate of measurement value expressed in provided unit.
     * @param valueY y-coordinate of measurement value expressed in provided unit.
     * @param valueZ z-coordinate of measurement value expressed in provided unit.
     */
    constructor(valueX: Double, valueY: Double, valueZ: Double) : this(
        DEFAULT_UNIT,
        valueX,
        valueY,
        valueZ
    )

    /**
     * Constructor.
     *
     * @param measurementX x-coordinate of measurement.
     * @param measurementY y-coordinate of measurement.
     * @param measurementZ z-coordinate of measurement.
     */
    constructor(
        measurementX: Speed,
        measurementY: Speed,
        measurementZ: Speed
    ) : this(DEFAULT_UNIT) {
        setMeasurementCoordinates(measurementX, measurementY, measurementZ)
    }

    /**
     * Copy constructor.
     *
     * @param other instance to copy from.
     */
    constructor(other: SpeedTriad) : this(other.unit) {
        this.setValueCoordinates(other.valueX, other.valueY, other.valueZ)
    }

    /**
     * Gets x coordinate of measurement value.
     *
     * @return x coordinate of measurement value.
     */
    override fun getMeasurementX(): Speed {
        return Speed(valueX, unit)
    }

    /**
     * Gets x coordinate of measurement value.
     *
     * @param result instance where x coordinate of measurement value will be stored.
     */
    override fun getMeasurementX(result: Speed) {
        result.value = valueX
        result.unit = unit
    }

    /**
     * Sets x coordinate of measurement value.
     *
     * @param measurementX x coordinate of measurement value.
     */
    override fun setMeasurementX(measurementX: Speed) {
        valueX = SpeedConverter.convert(measurementX.value, measurementX.unit, unit).toDouble()
    }

    /**
     * Gets y coordinate of measurement value.
     *
     * @return y coordinate of measurement value.
     */
    override fun getMeasurementY(): Speed {
        return Speed(valueY, unit)
    }

    /**
     * Gets y coordinate of measurement value.
     *
     * @param result instance where y coordinate of measurement value will be stored.
     */
    override fun getMeasurementY(result: Speed) {
        result.value = valueY
        result.unit = unit
    }

    /**
     * Sets y coordinate of measurement value.
     *
     * @param measurementY y coordinate of measurement value.
     */
    override fun setMeasurementY(measurementY: Speed) {
        valueY = SpeedConverter.convert(measurementY.value, measurementY.unit, unit).toDouble()
    }

    /**
     * Gets z coordinae of measurement value.
     *
     * @return z coordinate of measurement value.
     */
    override fun getMeasurementZ(): Speed {
        return Speed(valueZ, unit)
    }

    /**
     * Gets z coordinate of measurement value.
     *
     * @param result instance where z coordinate of measurement value will be stored.
     */
    override fun getMeasurementZ(result: Speed) {
        result.value = valueZ
        result.unit = unit
    }

    /**
     * Sets z coordinate of measurement value.
     *
     * @param measurementZ z coordinate of measurement value.
     */
    override fun setMeasurementZ(measurementZ: Speed) {
        valueZ = SpeedConverter.convert(measurementZ.value, measurementZ.unit, unit).toDouble()
    }

    /**
     * Sets measurement coordinates.
     *
     * @param measurementX x coordinate of measurement value.
     * @param measurementY y coordinate of measurement value.
     * @param measurementZ z coordinate of measurement value
     */
    override fun setMeasurementCoordinates(
        measurementX: Speed,
        measurementY: Speed,
        measurementZ: Speed
    ) {
        this.measurementX = measurementX
        this.measurementY = measurementY
        this.measurementZ = measurementZ
    }

    /**
     * Gets norm as speed.
     *
     * @return speed containing triad norm.
     */
    override fun getMeasurementNorm(): Speed {
        return Speed(norm, unit)
    }

    /**
     * Makes a copy of this instance.
     *
     * @return a copy of this instance.
     * @throws CloneNotSupportedException if clone fails for some reason.
     */
    @Throws(CloneNotSupportedException::class)
    public override fun clone(): Any {
        val result = super.clone() as SpeedTriad
        copyTo(result)
        return result
    }

    companion object {
        /**
         * Default speed unit.
         */
        val DEFAULT_UNIT = SpeedUnit.METERS_PER_SECOND
    }
}