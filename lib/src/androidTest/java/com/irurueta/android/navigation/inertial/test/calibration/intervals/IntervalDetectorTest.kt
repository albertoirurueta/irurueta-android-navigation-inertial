package com.irurueta.android.navigation.inertial.test.calibration.intervals

import android.util.Log
import androidx.test.filters.RequiresDevice
import androidx.test.platform.app.InstrumentationRegistry
import com.irurueta.android.navigation.inertial.ThreadSyncHelper
import com.irurueta.android.navigation.inertial.calibration.intervals.IntervalDetector
import org.junit.Before
import org.junit.Test

class IntervalDetectorTest {

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
        val detector = IntervalDetector(context,
            initializationStartedListener = {
                Log.i(
                    "IntervalDetectorTest",
                    "Initialization started"
                )
            },
            initializationCompletedListener = { _, baseNoiseLevel ->
                Log.i(
                    "IntervalDetectorTest",
                    "Initialization completed. Base noise level: $baseNoiseLevel m/s^2"
                )
            },
            errorListener = { _, reason -> Log.i("IntervalDetectorTest", "Error: $reason") },
            staticIntervalDetectedListener = { _, _, _, _, _, _, _ ->
                Log.i(
                    "IntervalDetectorTest",
                    "Static interval detected"
                )
            },
            dynamicIntervalDetectedListener = { _, _, _, _, _, _, _, _, _, _, _, _, _ ->
                Log.i(
                    "IntervalDetectorTest",
                    "Dynamic interval detected"
                )
            }, resetListener = { Log.i("IntervalDetectorTest", "Reset") })

        detector.start()

        syncHelper.waitOnCondition({ completed < 1 }, timeout = TIMEOUT)

        detector.stop()
    }

    private companion object {
        const val TIMEOUT = 100000L
    }
}