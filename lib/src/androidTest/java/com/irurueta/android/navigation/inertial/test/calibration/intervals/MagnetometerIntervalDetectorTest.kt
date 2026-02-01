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

package com.irurueta.android.navigation.inertial.test.calibration.intervals

import android.util.Log
import androidx.test.platform.app.InstrumentationRegistry
import com.irurueta.android.navigation.inertial.ThreadSyncHelper
import com.irurueta.android.navigation.inertial.calibration.intervals.MagnetometerIntervalDetector
import com.irurueta.android.testutils.RequiresRealDevice
import org.junit.Before
import org.junit.Test

class MagnetometerIntervalDetectorTest {

    private val syncHelper = ThreadSyncHelper()

    @Volatile
    private var completed = 0

    @Before
    fun setUp() {
        completed = 0
    }

    @RequiresRealDevice
    @Test
    fun startAndStop_detectsIntervals() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val detector = MagnetometerIntervalDetector(
            context,
            initializationStartedListener = {
                Log.i("MagnetometerIntervalDetectorTest", "Initialization started")
            },
            initializationCompletedListener = { _, baseNoiseLevel ->
                Log.i(
                    "MagnetometerIntervalDetectorTest",
                    "Initialization completed. Base noise level: $baseNoiseLevel T"
                )
            },
            errorListener = { _, reason ->
                Log.i("MagnetometerIntervalDetectorTest", "Error: $reason")
            },
            staticIntervalDetectedListener = { _, _, _, _, _, _, _ ->
                Log.i("MagnetometerIntervalDetectorTest", "Static interval detected")
            },
            dynamicIntervalDetectedListener = { _, _, _, _, _, _, _, _, _, _, _, _, _ ->
                Log.i("MagnetometerIntervalDetectorTest", "Dynamic interval detected")
            },
            resetListener = {
                Log.i("MagnetometerIntervalDetectorTest", "Reset")
            }
        )

        detector.instantaneousNoiseLevelFactor = INSTANTANEOUS_NOISE_LEVEL_FACTOR
        detector.thresholdFactor = THRESHOLD_FACTOR

        detector.start()

        syncHelper.waitOnCondition({ completed < 1 }, timeout = TIMEOUT)

        detector.stop()
    }

    private companion object {
        const val TIMEOUT = 1000L

        const val INSTANTANEOUS_NOISE_LEVEL_FACTOR = 3.0

        const val THRESHOLD_FACTOR = 3.0
    }
}