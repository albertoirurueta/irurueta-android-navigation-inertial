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
package com.irurueta.android.navigation.inertial.test.old.calibration.intervals.measurements

import android.util.Log
import androidx.test.filters.RequiresDevice
import androidx.test.platform.app.InstrumentationRegistry
import com.irurueta.android.navigation.inertial.ThreadSyncHelper
import com.irurueta.android.navigation.inertial.old.calibration.intervals.measurements.AccelerometerAndGyroscopeMeasurementGenerator
import org.junit.Before
import org.junit.Test

@RequiresDevice
class AccelerometerAndGyroscopeMeasurementGeneratorTest {

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
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(context,
            initializationStartedListener = {
                Log.i("AccelerometerAndGyroscopeMeasurementGeneratorTest", "Initialization started")
            },
            initializationCompletedListener = { _, baseNoiseLevel ->
                Log.i(
                    "AccelerometerAndGyroscopeMeasurementGeneratorTest",
                    "Initialization completed. Base noise level: $baseNoiseLevel m/s^2"
                )
            },
            errorListener = { _, reason ->
                Log.i("AccelerometerAndGyroscopeMeasurementGeneratorTest", "Error: $reason")
            },
            staticIntervalDetectedListener = {
                Log.i(
                    "AccelerometerAndGyroscopeMeasurementGeneratorTest",
                    "Static interval detected"
                )
            },
            dynamicIntervalDetectedListener = {
                Log.i(
                    "AccelerometerAndGyroscopeMeasurementGeneratorTest",
                    "Dynamic interval detected"
                )
            },
            staticIntervalSkippedListener = {
                Log.i(
                    "AccelerometerAndGyroscopeMeasurementGeneratorTest",
                    "Static interval skipped"
                )
            },
            dynamicIntervalSkippedListener = {
                Log.i(
                    "AccelerometerAndGyroscopeMeasurementGeneratorTest",
                    "Dynamic interval skipped"
                )
            },
            generatedAccelerometerMeasurementListener = { _, _ ->
                Log.i(
                    "AccelerometerAndGyroscopeMeasurementGeneratorTest",
                    "Accelerometer measurement generated"
                )
            },
            generatedGyroscopeMeasurementListener = { _, _ ->
                Log.i(
                    "AccelerometerAndGyroscopeMeasurementGeneratorTest",
                    "Gyroscope measurement generated"
                )
            },
            resetListener = {
                Log.i("AccelerometerAndGyroscopeMeasurementGeneratorTest", "Reset")
            }
        )

        generator.start()

        syncHelper.waitOnCondition({ completed < 1 }, timeout = TIMEOUT)

        generator.stop()
    }

    private companion object {
        const val TIMEOUT = 1000L
    }
}