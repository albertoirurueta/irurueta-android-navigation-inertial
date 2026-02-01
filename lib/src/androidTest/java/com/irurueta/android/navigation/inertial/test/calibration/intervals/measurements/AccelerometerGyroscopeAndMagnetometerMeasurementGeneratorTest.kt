/*
 * Copyright (C) 2025 Alberto Irurueta Carro (alberto@irurueta.com)
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
package com.irurueta.android.navigation.inertial.test.calibration.intervals.measurements

import android.util.Log
import androidx.test.platform.app.InstrumentationRegistry
import com.irurueta.android.navigation.inertial.ThreadSyncHelper
import com.irurueta.android.navigation.inertial.calibration.intervals.measurements.AccelerometerGyroscopeAndMagnetometerMeasurementGenerator
import com.irurueta.android.testutils.RequiresRealDevice
import org.junit.Before
import org.junit.Test

class AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorTest {

    private val syncHelper = ThreadSyncHelper()

    @Volatile
    private var completed = 0

    @Before
    fun setUp() {
        completed = 0
    }

    @RequiresRealDevice
    @Test
    fun startAndStop_generatesMeasurements() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(
            context,
            initializationStartedListener = {
                Log.i(
                    "AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorTest",
                    "Initialization started"
                )
            },
            initializationCompletedListener = { _, baseNoiseLevel ->
                Log.i(
                    "AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorTest",
                    "Initialization completed. Base noise level: $baseNoiseLevel m/s^2"
                )
            },
            errorListener = { _, reason ->
                Log.i(
                    "AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorTest",
                    "Error: $reason"
                )
            },
            staticIntervalDetectedListener = {
                Log.i(
                    "AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorTest",
                    "Static interval detected"
                )
            },
            dynamicIntervalDetectedListener = {
                Log.i(
                    "AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorTest",
                    "Dynamic interval detected"
                )
            },
            staticIntervalSkippedListener = {
                Log.i(
                    "AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorTest",
                    "Static interval skipped"
                )
            },
            dynamicIntervalSkippedListener = {
                Log.i(
                    "AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorTest",
                    "Dynamic interval skipped"
                )
            },
            generatedAccelerometerMeasurementListener = { _, _ ->
                Log.i(
                    "AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorTest",
                    "Accelerometer measurement generated"
                )
            },
            generatedGyroscopeMeasurementListener = { _, _ ->
                Log.i(
                    "AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorTest",
                    "Gyroscope measurement generated"
                )
            },
            generatedMagnetometerMeasurementListener = { _, _ ->
                Log.i(
                    "AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorTest",
                    "Magnetometer measurement generated"
                )
            },
            resetListener = {
                Log.i("AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorTest", "Reset")
            }
        )

        generator.instantaneousNoiseLevelFactor = INSTANTANEOUS_NOISE_LEVEL_FACTOR
        generator.thresholdFactor = THRESHOLD_FACTOR

        generator.start()

        syncHelper.waitOnCondition({ completed < 1 }, timeout = TIMEOUT)

        generator.stop()
    }

    private companion object {
        const val TIMEOUT = 1000L

        const val INSTANTANEOUS_NOISE_LEVEL_FACTOR = 3.0

        const val THRESHOLD_FACTOR = 3.0
    }
}