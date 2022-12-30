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
package com.irurueta.android.navigation.inertial.collectors

import android.os.Build
import org.junit.Assert
import org.junit.Test
import org.junit.runner.RunWith
import org.robolectric.RobolectricTestRunner
import org.robolectric.annotation.Config

@RunWith(RobolectricTestRunner::class)
class SensorMeasurementSyncerTest {

    @Config(sdk = [Build.VERSION_CODES.O])
    @Test
    fun sensorType_fromIntWhenSdkO_returnsExpectedValues() {
        Assert.assertEquals(7, SensorMeasurementSyncer.SensorType.values().size)
        Assert.assertEquals(
            SensorMeasurementSyncer.SensorType.ACCELEROMETER,
            SensorMeasurementSyncer.SensorType.from(SensorMeasurementSyncer.SensorType.ACCELEROMETER.value)
        )
        Assert.assertEquals(
            SensorMeasurementSyncer.SensorType.ACCELEROMETER_UNCALIBRATED,
            SensorMeasurementSyncer.SensorType.from(SensorMeasurementSyncer.SensorType.ACCELEROMETER_UNCALIBRATED.value)
        )
        Assert.assertEquals(
            SensorMeasurementSyncer.SensorType.GYROSCOPE,
            SensorMeasurementSyncer.SensorType.from(SensorMeasurementSyncer.SensorType.GYROSCOPE.value)
        )
        Assert.assertEquals(
            SensorMeasurementSyncer.SensorType.GYROSCOPE_UNCALIBRATED,
            SensorMeasurementSyncer.SensorType.from(SensorMeasurementSyncer.SensorType.GYROSCOPE_UNCALIBRATED.value)
        )
        Assert.assertEquals(
            SensorMeasurementSyncer.SensorType.MAGNETOMETER,
            SensorMeasurementSyncer.SensorType.from(SensorMeasurementSyncer.SensorType.MAGNETOMETER.value)
        )
        Assert.assertEquals(
            SensorMeasurementSyncer.SensorType.MAGNETOMETER_UNCALIBRATED,
            SensorMeasurementSyncer.SensorType.from(SensorMeasurementSyncer.SensorType.MAGNETOMETER_UNCALIBRATED.value)
        )
        Assert.assertEquals(
            SensorMeasurementSyncer.SensorType.GRAVITY,
            SensorMeasurementSyncer.SensorType.from(SensorMeasurementSyncer.SensorType.GRAVITY.value)
        )
    }

    @Config(sdk = [Build.VERSION_CODES.N])
    @Test
    fun sensorType_fromIntWhenSdkN_returnsExpectedValues() {
        Assert.assertEquals(7, SensorMeasurementSyncer.SensorType.values().size)
        Assert.assertEquals(
            SensorMeasurementSyncer.SensorType.ACCELEROMETER,
            SensorMeasurementSyncer.SensorType.from(SensorMeasurementSyncer.SensorType.ACCELEROMETER.value)
        )
        Assert.assertNull(SensorMeasurementSyncer.SensorType.from(SensorMeasurementSyncer.SensorType.ACCELEROMETER_UNCALIBRATED.value))
        Assert.assertEquals(
            SensorMeasurementSyncer.SensorType.GYROSCOPE,
            SensorMeasurementSyncer.SensorType.from(SensorMeasurementSyncer.SensorType.GYROSCOPE.value)
        )
        Assert.assertEquals(
            SensorMeasurementSyncer.SensorType.GYROSCOPE_UNCALIBRATED,
            SensorMeasurementSyncer.SensorType.from(SensorMeasurementSyncer.SensorType.GYROSCOPE_UNCALIBRATED.value)
        )
        Assert.assertEquals(
            SensorMeasurementSyncer.SensorType.MAGNETOMETER,
            SensorMeasurementSyncer.SensorType.from(SensorMeasurementSyncer.SensorType.MAGNETOMETER.value)
        )
        Assert.assertEquals(
            SensorMeasurementSyncer.SensorType.MAGNETOMETER_UNCALIBRATED,
            SensorMeasurementSyncer.SensorType.from(SensorMeasurementSyncer.SensorType.MAGNETOMETER_UNCALIBRATED.value)
        )
        Assert.assertEquals(
            SensorMeasurementSyncer.SensorType.GRAVITY,
            SensorMeasurementSyncer.SensorType.from(SensorMeasurementSyncer.SensorType.GRAVITY.value)
        )
    }

    @Test
    fun sensorType_fromAccelerometerSensorType_returnsExpectedValue() {
        Assert.assertEquals(
            SensorMeasurementSyncer.SensorType.ACCELEROMETER,
            SensorMeasurementSyncer.SensorType.from(AccelerometerSensorType.ACCELEROMETER)
        )
        Assert.assertEquals(
            SensorMeasurementSyncer.SensorType.ACCELEROMETER_UNCALIBRATED,
            SensorMeasurementSyncer.SensorType.from(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED)
        )
    }

    @Test
    fun sensorType_fromGyroscopeSensorType_returnsExpectedValue() {
        Assert.assertEquals(
            SensorMeasurementSyncer.SensorType.GYROSCOPE,
            SensorMeasurementSyncer.SensorType.from(GyroscopeSensorType.GYROSCOPE)
        )
        Assert.assertEquals(
            SensorMeasurementSyncer.SensorType.GYROSCOPE_UNCALIBRATED,
            SensorMeasurementSyncer.SensorType.from(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED)
        )
    }

    @Test
    fun sensorType_fromMagnetometerSensorType_returnsExpectedValue() {
        Assert.assertEquals(
            SensorMeasurementSyncer.SensorType.MAGNETOMETER,
            SensorMeasurementSyncer.SensorType.from(MagnetometerSensorType.MAGNETOMETER)
        )
        Assert.assertEquals(
            SensorMeasurementSyncer.SensorType.MAGNETOMETER_UNCALIBRATED,
            SensorMeasurementSyncer.SensorType.from(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED)
        )
    }
}