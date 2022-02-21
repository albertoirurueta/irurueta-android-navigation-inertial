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
package com.irurueta.android.navigation.inertial.test.calibration.intervals.measurements

import android.util.Log
import androidx.test.platform.app.InstrumentationRegistry
import com.irurueta.android.navigation.inertial.ThreadSyncHelper
import com.irurueta.android.navigation.inertial.calibration.intervals.measurements.MagnetometerMeasurementGenerator
import org.junit.Before
import org.junit.Test

class MagnetometerMeasurementGeneratorTest {

    private val syncHelper = ThreadSyncHelper()

    @Volatile
    private var completed = 0

    @Before
    fun setUp() {
        completed = 0
    }

    @Test
    fun startAndStop_generatesMeasurements() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val generator = MagnetometerMeasurementGenerator(context,
            initializationStartedListener = {
                Log.i("MagnetometerMeasurementGeneratorTest", "Initialization started")
            },
            initializationCompletedListener = { _, baseNoiseLevel ->
                Log.i(
                    "MagnetometerMeasurementGeneratorTest",
                    "Initialization completed. Base noise level: $baseNoiseLevel m/s^2"
                )
            },
            errorListener = { _, reason ->
                Log.i("MagnetometerMeasurementGeneratorTest", "Error: $reason")
            },
            staticIntervalDetectedListener = {
                Log.i("MagnetometerMeasurementGeneratorTest", "Static interval detected")
            },
            dynamicIntervalDetectedListener = {
                Log.i("MagnetometerMeasurementGeneratorTest", "Dynamic interval detected")
            },
            staticIntervalSkippedListener = {
                Log.i("MagnetometerMeasurementGeneratorTest", "Static interval skipped")
            },
            dynamicIntervalSkippedListener = {
                Log.i("MagnetometerMeasurementGeneratorTest", "Dynamic interval skipped")
            },
            generatedMeasurementListener = { _, _ ->
                Log.i("MagnetometerMeasurementGeneratorTest", "Measurement generated")
            },
            resetListener = {
                Log.i("MagnetometerMeasurementGeneratorTest", "Reset")
            },
            accuracyChangedListener = {
                Log.i("MagnetometerMeasurementGeneratorTest", "Accuracy changed")
            }
        )

        generator.start()

        syncHelper.waitOnCondition({ completed < 1 }, timeout = TIMEOUT)

        generator.stop()
    }

    private companion object {
        const val TIMEOUT = 100000L
    }
}