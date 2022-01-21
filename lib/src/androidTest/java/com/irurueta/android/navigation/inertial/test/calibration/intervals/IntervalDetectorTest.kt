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
            initializationStartedListener = object :
                IntervalDetector.OnInitializationStartedListener {
                override fun onInitializationStarted(detector: IntervalDetector) {
                    Log.i("IntervalDetectorTest", "Initialization started")
                }
            },
            initializationCompletedListener = object :
                IntervalDetector.OnInitializationCompletedListener {
                override fun onInitializationCompleted(
                    detector: IntervalDetector,
                    baseNoiseLevel: Double
                ) {
                    Log.i("IntervalDetectorTest",
                        "Initialization completed. Base noise level: $baseNoiseLevel m/s^2"
                    )
                }
            },
            errorListener = object : IntervalDetector.OnErrorListener {
                override fun onError(
                    detector: IntervalDetector,
                    reason: IntervalDetector.ErrorReason
                ) {
                    Log.i("IntervalDetectorTest", "Error: $reason")
                }
            },
            staticIntervalDetectedListener = object :
                IntervalDetector.OnStaticIntervalDetectedListener {
                override fun onStaticIntervalDetected(
                    detector: IntervalDetector,
                    instantaneousAvgX: Double,
                    instantaneousAvgY: Double,
                    instantaneousAvgZ: Double,
                    instantaneousStdX: Double,
                    instantaneousStdY: Double,
                    instantaneousStdZ: Double
                ) {
                    Log.i("IntervalDetectorTest", "Static interval detected")
                }
            },
            dynamicIntervalDetectedListener = object :
                IntervalDetector.OnDynamicIntervalDetectedListener {
                override fun onDynamicIntervalDetected(
                    detector: IntervalDetector,
                    instantaneousAvgX: Double,
                    instantaneousAvgY: Double,
                    instantaneousAvgZ: Double,
                    instantaneousStdX: Double,
                    instantaneousStdY: Double,
                    instantaneousStdZ: Double,
                    accumulatedAvgX: Double,
                    accumulatedAvgY: Double,
                    accumulatedAvgZ: Double,
                    accumulatedStdX: Double,
                    accumulatedStdY: Double,
                    accumulatedStdZ: Double
                ) {
                    Log.i("IntervalDetectorTest", "Dynamic interval detected")
                }
            }, resetListener = object : IntervalDetector.OnResetListener {
                override fun onReset(detector: IntervalDetector) {
                    Log.i("IntervalDetectorTest", "Reset")
                }
            })

        detector.start()

        syncHelper.waitOnCondition({ completed < 1 }, timeout = TIMEOUT)

        detector.stop()
    }

    private companion object {
        const val TIMEOUT = 100000L
    }
}