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
package com.irurueta.android.navigation.inertial.estimators.pose

import com.irurueta.algebra.Matrix
import com.irurueta.statistics.UniformRandomizer
import com.irurueta.units.Speed
import com.irurueta.units.SpeedUnit
import org.junit.Assert.*
import org.junit.Test
import kotlin.math.sqrt

class SpeedTriadTest {

    @Test
    fun constructor_whenNoParameters_setsExpectedValues() {
        val triad = SpeedTriad()

        // check
        assertEquals(0.0, triad.valueX, 0.0)
        assertEquals(0.0, triad.valueY, 0.0)
        assertEquals(0.0, triad.valueZ, 0.0)
        assertEquals(SpeedUnit.METERS_PER_SECOND, triad.unit)
        assertArrayEquals(DoubleArray(3), triad.valuesAsArray, 0.0)
        val values = DoubleArray(3)
        triad.getValuesAsArray(values)
        assertArrayEquals(DoubleArray(3), values, 0.0)
        assertEquals(Matrix(3, 1), triad.valuesAsMatrix)
        val v = Matrix(3, 1)
        triad.getValuesAsMatrix(v)
        assertEquals(Matrix(3, 1), v)
        val vx1 = triad.measurementX
        assertEquals(0.0, vx1.value.toDouble(), 0.0)
        assertEquals(SpeedUnit.METERS_PER_SECOND, vx1.unit)
        val vx2 = Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR)
        triad.getMeasurementX(vx2)
        assertEquals(0.0, vx2.value.toDouble(), 0.0)
        assertEquals(SpeedUnit.METERS_PER_SECOND, vx2.unit)
        val vy1 = triad.measurementY
        assertEquals(0.0, vy1.value.toDouble(), 0.0)
        assertEquals(SpeedUnit.METERS_PER_SECOND, vy1.unit)
        val vy2 = Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR)
        triad.getMeasurementY(vy2)
        assertEquals(0.0, vy2.value.toDouble(), 0.0)
        assertEquals(SpeedUnit.METERS_PER_SECOND, vy2.unit)
        val vz1 = triad.measurementZ
        assertEquals(0.0, vz1.value.toDouble(), 0.0)
        assertEquals(SpeedUnit.METERS_PER_SECOND, vz1.unit)
        val vz2 = Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR)
        triad.getMeasurementZ(vz2)
        assertEquals(0.0, vz2.value.toDouble(), 0.0)
        assertEquals(SpeedUnit.METERS_PER_SECOND, vz2.unit)
    }

    @Test
    fun constructor_whenUnit_setsExpectedValues() {
        val triad = SpeedTriad(SpeedUnit.KILOMETERS_PER_HOUR)

        // check
        assertEquals(0.0, triad.valueX, 0.0)
        assertEquals(0.0, triad.valueY, 0.0)
        assertEquals(0.0, triad.valueZ, 0.0)
        assertEquals(SpeedUnit.KILOMETERS_PER_HOUR, triad.unit)
        assertArrayEquals(DoubleArray(3), triad.valuesAsArray, 0.0)
        val values = DoubleArray(3)
        triad.getValuesAsArray(values)
        assertArrayEquals(DoubleArray(3), values, 0.0)
        assertEquals(Matrix(3, 1), triad.valuesAsMatrix)
        val v = Matrix(3, 1)
        triad.getValuesAsMatrix(v)
        assertEquals(Matrix(3, 1), v)
        val vx1 = triad.measurementX
        assertEquals(0.0, vx1.value.toDouble(), 0.0)
        assertEquals(SpeedUnit.KILOMETERS_PER_HOUR, vx1.unit)
        val vx2 = Speed(1.0, SpeedUnit.METERS_PER_SECOND)
        triad.getMeasurementX(vx2)
        assertEquals(0.0, vx2.value.toDouble(), 0.0)
        assertEquals(SpeedUnit.KILOMETERS_PER_HOUR, vx2.unit)
        val vy1 = triad.measurementY
        assertEquals(0.0, vy1.value.toDouble(), 0.0)
        assertEquals(SpeedUnit.KILOMETERS_PER_HOUR, vy1.unit)
        val vy2 = Speed(1.0, SpeedUnit.METERS_PER_SECOND)
        triad.getMeasurementY(vy2)
        assertEquals(0.0, vy2.value.toDouble(), 0.0)
        assertEquals(SpeedUnit.KILOMETERS_PER_HOUR, vy2.unit)
        val vz1 = triad.measurementZ
        assertEquals(0.0, vz1.value.toDouble(), 0.0)
        assertEquals(SpeedUnit.KILOMETERS_PER_HOUR, vz1.unit)
        val vz2 = Speed(1.0, SpeedUnit.METERS_PER_SECOND)
        triad.getMeasurementZ(vz2)
        assertEquals(0.0, vz2.value.toDouble(), 0.0)
        assertEquals(SpeedUnit.KILOMETERS_PER_HOUR, vz2.unit)
    }

    @Test
    fun constructor_whenValuesAndDefaultUnit_setsExpectedValues() {
        val randomizer = UniformRandomizer()
        val valueX = randomizer.nextDouble()
        val valueY = randomizer.nextDouble()
        val valueZ = randomizer.nextDouble()

        val values1 = doubleArrayOf(valueX, valueY, valueZ)
        val v1 = Matrix.newFromArray(values1)

        val triad = SpeedTriad(valueX, valueY, valueZ)

        // check
        assertEquals(valueX, triad.valueX, 0.0)
        assertEquals(valueY, triad.valueY, 0.0)
        assertEquals(valueZ, triad.valueZ, 0.0)
        assertEquals(SpeedTriad.DEFAULT_UNIT, triad.unit)
        assertArrayEquals(doubleArrayOf(valueX, valueY, valueZ), triad.valuesAsArray, 0.0)
        val values2 = DoubleArray(3)
        triad.getValuesAsArray(values2)
        assertArrayEquals(values1, values2, 0.0)
        assertEquals(v1, triad.valuesAsMatrix)
        val v2 = Matrix(3, 1)
        triad.getValuesAsMatrix(v2)
        assertEquals(v1, v2)
        val vx1 = triad.measurementX
        assertEquals(valueX, vx1.value.toDouble(), 0.0)
        assertEquals(SpeedUnit.METERS_PER_SECOND, vx1.unit)
        val vx2 = Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR)
        triad.getMeasurementX(vx2)
        assertEquals(valueX, vx2.value.toDouble(), 0.0)
        assertEquals(SpeedUnit.METERS_PER_SECOND, vx2.unit)
        val vy1 = triad.measurementY
        assertEquals(valueY, vy1.value.toDouble(), 0.0)
        assertEquals(SpeedUnit.METERS_PER_SECOND, vy1.unit)
        val vy2 = Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR)
        triad.getMeasurementY(vy2)
        assertEquals(valueY, vy2.value.toDouble(), 0.0)
        assertEquals(SpeedUnit.METERS_PER_SECOND, vy2.unit)
        val vz1 = triad.measurementZ
        assertEquals(valueZ, vz1.value.toDouble(), 0.0)
        assertEquals(SpeedUnit.METERS_PER_SECOND, vz1.unit)
        val vz2 = Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR)
        triad.getMeasurementZ(vz2)
        assertEquals(valueZ, vz2.value.toDouble(), 0.0)
        assertEquals(SpeedUnit.METERS_PER_SECOND, vz2.unit)
    }

    @Test
    fun constructor_whenValuesAndUnit_setsExpectedValues() {
        val randomizer = UniformRandomizer()
        val valueX = randomizer.nextDouble()
        val valueY = randomizer.nextDouble()
        val valueZ = randomizer.nextDouble()

        val values1 = doubleArrayOf(valueX, valueY, valueZ)
        val v1 = Matrix.newFromArray(values1)

        val triad = SpeedTriad(SpeedUnit.KILOMETERS_PER_HOUR, valueX, valueY, valueZ)

        // check
        assertEquals(valueX, triad.valueX, 0.0)
        assertEquals(valueY, triad.valueY, 0.0)
        assertEquals(valueZ, triad.valueZ, 0.0)
        assertEquals(SpeedUnit.KILOMETERS_PER_HOUR, triad.unit)
        assertArrayEquals(doubleArrayOf(valueX, valueY, valueZ), triad.valuesAsArray, 0.0)
        val values2 = DoubleArray(3)
        triad.getValuesAsArray(values2)
        assertArrayEquals(values1, values2, 0.0)
        assertEquals(v1, triad.valuesAsMatrix)
        val v2 = Matrix(3, 1)
        triad.getValuesAsMatrix(v2)
        assertEquals(v1, v2)
        val vx1 = triad.measurementX
        assertEquals(valueX, vx1.value.toDouble(), 0.0)
        assertEquals(SpeedUnit.KILOMETERS_PER_HOUR, vx1.unit)
        val vx2 = Speed(1.0, SpeedUnit.METERS_PER_SECOND)
        triad.getMeasurementX(vx2)
        assertEquals(valueX, vx2.value.toDouble(), 0.0)
        assertEquals(SpeedUnit.KILOMETERS_PER_HOUR, vx2.unit)
        val vy1 = triad.measurementY
        assertEquals(valueY, vy1.value.toDouble(), 0.0)
        assertEquals(SpeedUnit.KILOMETERS_PER_HOUR, vy1.unit)
        val vy2 = Speed(1.0, SpeedUnit.METERS_PER_SECOND)
        triad.getMeasurementY(vy2)
        assertEquals(valueY, vy2.value.toDouble(), 0.0)
        assertEquals(SpeedUnit.KILOMETERS_PER_HOUR, vy2.unit)
        val vz1 = triad.measurementZ
        assertEquals(valueZ, vz1.value.toDouble(), 0.0)
        assertEquals(SpeedUnit.KILOMETERS_PER_HOUR, vz1.unit)
        val vz2 = Speed(1.0, SpeedUnit.METERS_PER_SECOND)
        triad.getMeasurementZ(vz2)
        assertEquals(valueZ, vz2.value.toDouble(), 0.0)
        assertEquals(SpeedUnit.KILOMETERS_PER_HOUR, vz2.unit)
    }

    @Test
    fun constructor_whenMeasurementsAndDefaultUnit_setsExpectedValues() {
        val randomizer = UniformRandomizer()
        val valueX = randomizer.nextDouble()
        val valueY = randomizer.nextDouble()
        val valueZ = randomizer.nextDouble()

        val mx = Speed(valueX, SpeedUnit.METERS_PER_SECOND)
        val my = Speed(valueY, SpeedUnit.METERS_PER_SECOND)
        val mz = Speed(valueZ, SpeedUnit.METERS_PER_SECOND)

        val values1 = doubleArrayOf(valueX, valueY, valueZ)
        val v1 = Matrix.newFromArray(values1)

        val triad = SpeedTriad(mx, my, mz)

        // check
        assertEquals(valueX, triad.valueX, 0.0)
        assertEquals(valueY, triad.valueY, 0.0)
        assertEquals(valueZ, triad.valueZ, 0.0)
        assertEquals(SpeedTriad.DEFAULT_UNIT, triad.unit)
        assertArrayEquals(doubleArrayOf(valueX, valueY, valueZ), triad.valuesAsArray, 0.0)
        val values2 = DoubleArray(3)
        triad.getValuesAsArray(values2)
        assertArrayEquals(values1, values2, 0.0)
        assertEquals(v1, triad.valuesAsMatrix)
        val v2 = Matrix(3, 1)
        triad.getValuesAsMatrix(v2)
        assertEquals(v1, v2)
        val vx1 = triad.measurementX
        assertEquals(valueX, vx1.value.toDouble(), 0.0)
        assertEquals(SpeedUnit.METERS_PER_SECOND, vx1.unit)
        val vx2 = Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR)
        triad.getMeasurementX(vx2)
        assertEquals(valueX, vx2.value.toDouble(), 0.0)
        assertEquals(SpeedUnit.METERS_PER_SECOND, vx2.unit)
        val vy1 = triad.measurementY
        assertEquals(valueY, vy1.value.toDouble(), 0.0)
        assertEquals(SpeedUnit.METERS_PER_SECOND, vy1.unit)
        val vy2 = Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR)
        triad.getMeasurementY(vy2)
        assertEquals(valueY, vy2.value.toDouble(), 0.0)
        assertEquals(SpeedUnit.METERS_PER_SECOND, vy2.unit)
        val vz1 = triad.measurementZ
        assertEquals(valueZ, vz1.value.toDouble(), 0.0)
        assertEquals(SpeedUnit.METERS_PER_SECOND, vz1.unit)
        val vz2 = Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR)
        triad.getMeasurementZ(vz2)
        assertEquals(valueZ, vz2.value.toDouble(), 0.0)
        assertEquals(SpeedUnit.METERS_PER_SECOND, vz2.unit)
    }

    @Test
    fun constructor_whenOtherTriad_setsExpectedValues() {
        val randomizer = UniformRandomizer()
        val valueX = randomizer.nextDouble()
        val valueY = randomizer.nextDouble()
        val valueZ = randomizer.nextDouble()

        val triad1 = SpeedTriad(valueX, valueY, valueZ)
        val triad2 = SpeedTriad(triad1)

        // check
        assertEquals(valueX, triad2.valueX, 0.0)
        assertEquals(valueY, triad2.valueY, 0.0)
        assertEquals(valueZ, triad2.valueZ, 0.0)
        assertEquals(triad1.unit, triad2.unit)
    }

    @Test
    fun valueX_getsAndSetsExpectedValue() {
        val triad = SpeedTriad()

        // check default value
        assertEquals(0.0, triad.valueX, 0.0)

        // set new value
        val randomizer = UniformRandomizer()
        val valueX = randomizer.nextDouble()

        triad.valueX = valueX

        // check
        assertEquals(valueX, triad.valueX, 0.0)
    }

    @Test
    fun valueY_getsAndSetsExpectedValue() {
        val triad = SpeedTriad()

        // check default value
        assertEquals(0.0, triad.valueY, 0.0)

        // set new value
        val randomizer = UniformRandomizer()
        val valueY = randomizer.nextDouble()

        triad.valueY = valueY

        // check
        assertEquals(valueY, triad.valueY, 0.0)
    }

    @Test
    fun valueZ_getsAndSetsExpectedValue() {
        val triad = SpeedTriad()

        // check default value
        assertEquals(0.0, triad.valueZ, 0.0)

        // set new value
        val randomizer = UniformRandomizer()
        val valueZ = randomizer.nextDouble()

        triad.valueZ = valueZ

        // check
        assertEquals(valueZ, triad.valueZ, 0.0)
    }

    @Test
    fun setValueCoordinates_setsExpectedValues() {
        val triad = SpeedTriad()

        // check default value
        assertEquals(0.0, triad.valueX, 0.0)
        assertEquals(0.0, triad.valueY, 0.0)
        assertEquals(0.0, triad.valueZ, 0.0)

        // set new value
        val randomizer = UniformRandomizer()
        val valueX = randomizer.nextDouble()
        val valueY = randomizer.nextDouble()
        val valueZ = randomizer.nextDouble()

        triad.setValueCoordinates(valueX, valueY, valueZ)

        // check
        assertEquals(valueX, triad.valueX, 0.0)
        assertEquals(valueY, triad.valueY, 0.0)
        assertEquals(valueZ, triad.valueZ, 0.0)
    }

    @Test
    fun unit_whenNotNull_getsAndSetsExpectedValue() {
        val triad = SpeedTriad()

        // check default value
        assertEquals(SpeedUnit.METERS_PER_SECOND, triad.unit)

        // set new value
        triad.unit = SpeedUnit.KILOMETERS_PER_HOUR

        // check
        assertEquals(SpeedUnit.KILOMETERS_PER_HOUR, triad.unit)
    }

    @Test(expected = IllegalArgumentException::class)
    fun unit_whenNull_throwsIllegalArgumentException() {
        val triad = SpeedTriad()

        triad.unit = null
    }

    @Test
    fun setValueCoordinatesAndUnit_whenNotNullUnit_setsExpectedValues() {
        val triad = SpeedTriad()

        // check default values
        assertEquals(0.0, triad.valueX, 0.0)
        assertEquals(0.0, triad.valueY, 0.0)
        assertEquals(0.0, triad.valueZ, 0.0)
        assertEquals(SpeedUnit.METERS_PER_SECOND, triad.unit)

        // set new value
        val randomizer = UniformRandomizer()
        val valueX = randomizer.nextDouble()
        val valueY = randomizer.nextDouble()
        val valueZ = randomizer.nextDouble()

        triad.setValueCoordinatesAndUnit(valueX, valueY, valueZ, SpeedUnit.KILOMETERS_PER_HOUR)

        // check
        assertEquals(valueX, triad.valueX, 0.0)
        assertEquals(valueY, triad.valueY, 0.0)
        assertEquals(valueZ, triad.valueZ, 0.0)
        assertEquals(SpeedUnit.KILOMETERS_PER_HOUR, triad.unit)
    }

    @Test(expected = IllegalArgumentException::class)
    fun setValueCoordinatesAndUnit_whenNullUnit_throwsIllegalArgumentException() {
        val triad = SpeedTriad()

        val randomizer = UniformRandomizer()
        val valueX = randomizer.nextDouble()
        val valueY = randomizer.nextDouble()
        val valueZ = randomizer.nextDouble()

        triad.setValueCoordinatesAndUnit(valueX, valueY, valueZ, null)
    }

    @Test
    fun getSetValuesAsArray_whenValid_getsExpectedValues() {
        val triad = SpeedTriad()

        // check default value
        assertArrayEquals(DoubleArray(3), triad.valuesAsArray, 0.0)

        // set new value
        val randomizer = UniformRandomizer()
        val values1 = DoubleArray(3)
        randomizer.fill(values1)

        triad.setValueCoordinates(values1)

        // check
        val values2 = triad.valuesAsArray
        val values3 = DoubleArray(3)
        triad.getValuesAsArray(values3)

        assertArrayEquals(values1, values2, 0.0)
        assertArrayEquals(values1, values3, 0.0)
    }

    @Test
    fun getSetValuesAsArray_whenInvalidLength_throwsIllegalArgumentException() {
        val triad = SpeedTriad()

        assertThrows(IllegalArgumentException::class.java) { triad.getValuesAsArray(DoubleArray(1)) }
        assertThrows(IllegalArgumentException::class.java) { triad.setValueCoordinates(DoubleArray(2)) }
    }

    @Test
    fun getSetValuesAsMatrix_whenValidSize_getsAndSetsExpectedValues() {
        val triad = SpeedTriad()

        // check default value
        assertEquals(Matrix(3, 1), triad.valuesAsMatrix)

        // set new value
        val values1 = Matrix.createWithUniformRandomValues(3, 1, -1.0, 1.0)

        triad.setValueCoordinates(values1)

        // check
        val values2 = triad.valuesAsMatrix
        val values3 = Matrix(3, 1)
        triad.getValuesAsMatrix(values3)

        assertEquals(values1, values2)
        assertEquals(values1, values3)
    }

    @Test
    fun getSetValuesAsMatrix_whenInvalidSize_throwsIllegalArgumentException() {
        val triad = SpeedTriad()

        assertThrows(IllegalArgumentException::class.java) { triad.getValuesAsMatrix(Matrix(1, 1)) }
        assertThrows(IllegalArgumentException::class.java) { triad.getValuesAsMatrix(Matrix(3, 3)) }
        assertThrows(IllegalArgumentException::class.java) {
            triad.setValueCoordinates(
                Matrix(
                    1,
                    1
                )
            )
        }
        assertThrows(IllegalArgumentException::class.java) {
            triad.setValueCoordinates(
                Matrix(
                    3,
                    3
                )
            )
        }
    }

    @Test
    fun getSetMeasurementX_getsExpectedValues() {
        val triad = SpeedTriad()

        // check default value
        val mx1 = triad.measurementX
        assertEquals(0.0, mx1.value.toDouble(), 0.0)
        assertEquals(SpeedUnit.METERS_PER_SECOND, mx1.unit)

        // set new value
        val randomizer = UniformRandomizer()
        val valueX = randomizer.nextDouble()
        val mx2 = Speed(valueX, SpeedUnit.METERS_PER_SECOND)

        triad.measurementX = mx2

        // check
        val mx3 = triad.measurementX
        val mx4 = Speed(0.0, SpeedUnit.METERS_PER_SECOND)
        triad.getMeasurementX(mx4)

        assertEquals(mx2, mx3)
        assertEquals(mx2, mx4)
    }

    @Test
    fun getSetMeasurementY_getsExpectedValues() {
        val triad = SpeedTriad()

        // check default value
        val my1 = triad.measurementY
        assertEquals(0.0, my1.value.toDouble(), 0.0)
        assertEquals(SpeedUnit.METERS_PER_SECOND, my1.unit)

        // set new value
        val randomizer = UniformRandomizer()
        val valueY = randomizer.nextDouble()
        val my2 = Speed(valueY, SpeedUnit.METERS_PER_SECOND)

        triad.measurementY = my2

        // check
        val my3 = triad.measurementY
        val my4 = Speed(0.0, SpeedUnit.METERS_PER_SECOND)
        triad.getMeasurementY(my4)

        assertEquals(my2, my3)
        assertEquals(my2, my4)
    }

    @Test
    fun getSetMeasurementZ_getsExpectedValues() {
        val triad = SpeedTriad()

        // check default value
        val mz1 = triad.measurementZ
        assertEquals(0.0, mz1.value.toDouble(), 0.0)
        assertEquals(SpeedUnit.METERS_PER_SECOND, mz1.unit)

        // set new value
        val randomizer = UniformRandomizer()
        val valueZ = randomizer.nextDouble()
        val mz2 = Speed(valueZ, SpeedUnit.METERS_PER_SECOND)

        triad.measurementZ = mz2

        // check
        val mz3 = triad.measurementZ
        val mz4 = Speed(0.0, SpeedUnit.METERS_PER_SECOND)
        triad.getMeasurementZ(mz4)

        assertEquals(mz2, mz3)
        assertEquals(mz2, mz4)
    }

    @Test
    fun setMeasurementCoordinates_setsExpectedValues() {
        val triad = SpeedTriad()

        // check default values
        val mx1 = triad.measurementX
        val my1 = triad.measurementY
        val mz1 = triad.measurementZ

        assertEquals(0.0, mx1.value.toDouble(), 0.0)
        assertEquals(SpeedUnit.METERS_PER_SECOND, mx1.unit)
        assertEquals(0.0, my1.value.toDouble(), 0.0)
        assertEquals(SpeedUnit.METERS_PER_SECOND, my1.unit)
        assertEquals(0.0, mz1.value.toDouble(), 0.0)
        assertEquals(SpeedUnit.METERS_PER_SECOND, mz1.unit)

        // set new values
        val randomizer = UniformRandomizer()
        val valueX = randomizer.nextDouble()
        val valueY = randomizer.nextDouble()
        val valueZ = randomizer.nextDouble()
        val mx2 = Speed(valueX, SpeedUnit.METERS_PER_SECOND)
        val my2 = Speed(valueY, SpeedUnit.METERS_PER_SECOND)
        val mz2 = Speed(valueZ, SpeedUnit.METERS_PER_SECOND)

        triad.setMeasurementCoordinates(mx2, my2, mz2)

        // check
        val mx3 = triad.measurementX
        val my3 = triad.measurementY
        val mz3 = triad.measurementZ

        assertEquals(mx2, mx3)
        assertEquals(my2, my3)
        assertEquals(mz2, mz3)
    }

    @Test
    fun copyTo_makesExpectedCopy() {
        val randomizer = UniformRandomizer()
        val valueX = randomizer.nextDouble()
        val valueY = randomizer.nextDouble()
        val valueZ = randomizer.nextDouble()

        val triad1 = SpeedTriad(valueX, valueY, valueZ)
        val triad2 = SpeedTriad(SpeedUnit.KILOMETERS_PER_HOUR)

        triad1.copyTo(triad2)

        // check
        assertEquals(valueX, triad2.valueX, 0.0)
        assertEquals(valueY, triad2.valueY, 0.0)
        assertEquals(valueZ, triad2.valueZ, 0.0)
        assertEquals(SpeedUnit.METERS_PER_SECOND, triad2.unit)
    }

    @Test
    fun copyFrom_makesExpectedCopy() {
        val randomizer = UniformRandomizer()
        val valueX = randomizer.nextDouble()
        val valueY = randomizer.nextDouble()
        val valueZ = randomizer.nextDouble()

        val triad1 = SpeedTriad(valueX, valueY, valueZ)
        val triad2 = SpeedTriad(SpeedUnit.KILOMETERS_PER_HOUR)

        triad2.copyFrom(triad1)

        // check
        assertEquals(valueX, triad2.valueX, 0.0)
        assertEquals(valueY, triad2.valueY, 0.0)
        assertEquals(valueZ, triad2.valueZ, 0.0)
        assertEquals(SpeedUnit.METERS_PER_SECOND, triad2.unit)
    }

    @Test
    fun hashCode_computesExpectedHash() {
        val randomizer = UniformRandomizer()
        val valueX = randomizer.nextDouble()
        val valueY = randomizer.nextDouble()
        val valueZ = randomizer.nextDouble()

        val triad1 = SpeedTriad(valueX, valueY, valueZ)
        val triad2 = SpeedTriad(triad1)
        val triad3 = SpeedTriad()

        assertEquals(triad1.hashCode(), triad2.hashCode())
        assertNotEquals(triad1.hashCode(), triad3.hashCode())
    }

    @Test
    fun equals_whenNoThreshold_checksEquality() {
        val randomizer = UniformRandomizer()
        val valueX = randomizer.nextDouble()
        val valueY = randomizer.nextDouble()
        val valueZ = randomizer.nextDouble()

        val triad1 = SpeedTriad(valueX, valueY, valueZ)
        val triad2 = SpeedTriad(triad1)
        val triad3 = SpeedTriad()

        assertTrue(triad1.equals(triad2))
        assertTrue(triad2.equals(triad1))
        assertTrue(triad1.equals(triad1))
        assertTrue(triad2.equals(triad2))
        assertFalse(triad1.equals(triad3))
        assertFalse(triad2.equals(triad3))
        assertFalse(triad1.equals(null))
    }

    @Test
    fun equals_whenThreshold_checksEquality() {
        val randomizer = UniformRandomizer()
        val valueX = randomizer.nextDouble()
        val valueY = randomizer.nextDouble()
        val valueZ = randomizer.nextDouble()

        val triad1 = SpeedTriad(valueX, valueY, valueZ)
        val triad2 = SpeedTriad(triad1)
        val triad3 = SpeedTriad()

        assertTrue(triad1.equals(triad2, ABSOLUTE_ERROR))
        assertTrue(triad2.equals(triad1, ABSOLUTE_ERROR))
        assertTrue(triad1.equals(triad1, ABSOLUTE_ERROR))
        assertTrue(triad2.equals(triad2, ABSOLUTE_ERROR))
        assertFalse(triad1.equals(triad3, ABSOLUTE_ERROR))
        assertFalse(triad2.equals(triad3, ABSOLUTE_ERROR))
        assertFalse(triad1.equals(null, ABSOLUTE_ERROR))
    }

    @Test
    fun equals_whenObjectOfAnotherType_checksEquality() {
        val randomizer = UniformRandomizer()
        val valueX = randomizer.nextDouble()
        val valueY = randomizer.nextDouble()
        val valueZ = randomizer.nextDouble()

        val triad1 = SpeedTriad(valueX, valueY, valueZ)
        val triad2 = SpeedTriad(triad1)
        val triad3 = SpeedTriad()
        val obj = Object()

        assertEquals(triad1, triad2)
        assertNotEquals(triad1, triad3)
        assertNotEquals(triad1, obj)
        assertNotEquals(triad1, null)
    }

    @Test
    fun clone_makesExpectedCopy() {
        val randomizer = UniformRandomizer()
        val valueX = randomizer.nextDouble()
        val valueY = randomizer.nextDouble()
        val valueZ = randomizer.nextDouble()

        val triad1 = SpeedTriad(valueX, valueY, valueZ)
        val triad2 = triad1.clone()

        assertEquals(triad1, triad2)
    }

    @Test
    fun norm_returnsExpectedValue() {
        val randomizer = UniformRandomizer()
        val valueX = randomizer.nextDouble()
        val valueY = randomizer.nextDouble()
        val valueZ = randomizer.nextDouble()

        val triad = SpeedTriad(valueX, valueY, valueZ)

        val sqrNorm = valueX * valueX + valueY * valueY + valueZ * valueZ
        val norm = sqrt(sqrNorm)

        assertEquals(sqrNorm, triad.sqrNorm, 0.0)
        assertEquals(norm, triad.norm, 0.0)

        val s1 = triad.measurementNorm
        val s2 = Speed(0.0, SpeedUnit.KILOMETERS_PER_HOUR)
        triad.getMeasurementNorm(s2)

        assertEquals(norm, s1.value.toDouble(), 0.0)
        assertEquals(SpeedUnit.METERS_PER_SECOND, s1.unit)
        assertEquals(s1, s2)
    }

    companion object {
        const val ABSOLUTE_ERROR = 1e-12
    }
}