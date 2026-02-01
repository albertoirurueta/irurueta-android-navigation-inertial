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
package com.irurueta.android.navigation.inertial.test.old.calibration.intervals

import android.util.Log
import androidx.test.filters.RequiresDevice
import androidx.test.platform.app.InstrumentationRegistry
import com.irurueta.android.navigation.inertial.ThreadSyncHelper
import com.irurueta.android.navigation.inertial.old.calibration.intervals.AccelerometerIntervalDetector
import org.junit.Before
import org.junit.Test

class AccelerometerIntervalDetectorTest {

    private val syncHelper = ThreadSyncHelper()

    @Volatile
    private var completed = 0

    @Before
    fun setUp() {
        completed = 0
    }

    @RequiresDevice
    @Test
    fun startAndStop_detectsIntervals() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val detector = AccelerometerIntervalDetector(context,
            initializationStartedListener = {
                Log.i(
                    "AccelerometerIntervalDetectorTest",
                    "Initialization started"
                )
            },
            initializationCompletedListener = { _, baseNoiseLevel ->
                Log.i(
                    "AccelerometerIntervalDetectorTest",
                    "Initialization completed. Base noise level: $baseNoiseLevel m/s^2"
                )
            },
            errorListener = { _, reason ->
                Log.i(
                    "AccelerometerIntervalDetectorTest",
                    "Error: $reason"
                )
            },
            staticIntervalDetectedListener = { _, _, _, _, _, _, _ ->
                Log.i(
                    "AccelerometerIntervalDetectorTest",
                    "Static interval detected"
                )
            },
            dynamicIntervalDetectedListener = { _, _, _, _, _, _, _, _, _, _, _, _, _ ->
                Log.i(
                    "AccelerometerIntervalDetectorTest",
                    "Dynamic interval detected"
                )
            },
            resetListener = { Log.i("AccelerometerIntervalDetectorTest", "Reset") }
        )

        detector.start()

        syncHelper.waitOnCondition({ completed < 1 }, timeout = TIMEOUT)

        detector.stop()
    }

    private companion object {
        const val TIMEOUT = 1000L
    }
}