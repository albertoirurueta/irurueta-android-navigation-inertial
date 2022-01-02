package com.irurueta.android.navigation.inertial.test.estimators

import android.util.Log
import androidx.test.platform.app.InstrumentationRegistry
import com.irurueta.android.navigation.inertial.ThreadSyncHelper
import com.irurueta.android.navigation.inertial.estimators.GravityNormEstimator
import com.irurueta.units.Acceleration
import com.irurueta.units.AccelerationUnit
import com.irurueta.units.Time
import com.irurueta.units.TimeUnit
import org.junit.Assert.*
import org.junit.Before
import org.junit.Test

class GravityNormEstimatorTest {

    private val syncHelper = ThreadSyncHelper()

    @Volatile
    private var completed = 0

    @Before
    fun setUp() {
        completed = 0
    }

    @Test
    fun startAndStop_estimatesGravity() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val estimator = GravityNormEstimator(context,
            completedListener = object : GravityNormEstimator.OnEstimationCompletedListener {
                override fun onEstimatedCompleted(estimator: GravityNormEstimator) {
                    assertFalse(estimator.running)

                    syncHelper.notifyAll { completed++ }
                }
            },
            unreliableListener = object : GravityNormEstimator.OnUnreliableListener {
                override fun onUnreliable(estimator: GravityNormEstimator) {
                    Log.d("GravityNormEstimatorTest", "Sensor is unreliable")
                    assertFalse(estimator.running)
                }
            }
        )

        estimator.start()

        syncHelper.waitOnCondition({ completed < 1 }, timeout = 2L * estimator.maxDurationMillis)

        // obtain results
        assertFalse(estimator.running)
        assertTrue(estimator.numberOfProcessedMeasurements > 0)
        assertTrue(estimator.resultAvailable)

        val averageGravityNorm = estimator.averageGravityNorm
        requireNotNull(averageGravityNorm)
        val averageGravityNorm1 = estimator.averageGravityNormAsAcceleration
        requireNotNull(averageGravityNorm1)
        val averageGravityNorm2 =
            Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        estimator.getAverageGravityNormAsAcceleration(averageGravityNorm2)
        assertEquals(averageGravityNorm1, averageGravityNorm2)
        assertEquals(averageGravityNorm, averageGravityNorm1.value.toDouble(), 0.0)

        val gravityNormVariance = estimator.gravityNormVariance
        requireNotNull(gravityNormVariance)
        assertTrue(gravityNormVariance > 0.0)

        val gravityNormStandardDeviation = estimator.gravityNormStandardDeviation
        requireNotNull(gravityNormStandardDeviation)
        assertTrue(gravityNormStandardDeviation > 0.0)
        val gravityNormStandardDeviation1 = estimator.gravityNormStandardDeviationAsAcceleration
        requireNotNull(gravityNormStandardDeviation1)
        val gravityNormStandardDeviation2 =
            Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        estimator.getGravityNormStandardDeviationAsAcceleration(gravityNormStandardDeviation2)
        assertEquals(gravityNormStandardDeviation1, gravityNormStandardDeviation2)
        assertEquals(
            gravityNormStandardDeviation,
            gravityNormStandardDeviation1.value.toDouble(),
            0.0
        )

        val gravityPsd = estimator.gravityPsd
        requireNotNull(gravityPsd)
        assertTrue(gravityPsd > 0.0)

        val gravityRootPsd = estimator.gravityRootPsd
        requireNotNull(gravityRootPsd)
        assertTrue(gravityRootPsd > 0.0)

        val averageTimeInterval = estimator.averageTimeInterval
        requireNotNull(averageTimeInterval)
        assertTrue(averageTimeInterval > 0.0)
        val averageTimeInterval1 = estimator.averageTimeIntervalAsTime
        requireNotNull(averageTimeInterval1)
        val averageTimeInterval2 = Time(0.0, TimeUnit.SECOND)
        estimator.getAverageTimeIntervalAsTime(averageTimeInterval2)
        assertEquals(averageTimeInterval1, averageTimeInterval2)
        assertEquals(averageTimeInterval, averageTimeInterval1.value.toDouble(), 0.0)

        val timeIntervalVariance = estimator.timeIntervalVariance
        requireNotNull(timeIntervalVariance)
        assertTrue(timeIntervalVariance > 0.0)

        val timeIntervalStandardDeviation = estimator.timeIntervalStandardDeviation
        requireNotNull(timeIntervalStandardDeviation)
        assertTrue(timeIntervalStandardDeviation > 0.0)
        val timeIntervalStandardDeviation1 = estimator.timeIntervalStandardDeviationAsTime
        requireNotNull(timeIntervalStandardDeviation1)
        val timeIntervalStandardDeviation2 = Time(0.0, TimeUnit.SECOND)
        estimator.getTimeIntervalStandardDeviationAsTime(timeIntervalStandardDeviation2)
        assertEquals(timeIntervalStandardDeviation1, timeIntervalStandardDeviation2)
        assertEquals(
            timeIntervalStandardDeviation,
            timeIntervalStandardDeviation1.value.toDouble(),
            0.0
        )

        val elapsedTime = estimator.elapsedTimeNanos
        assertTrue(elapsedTime > 0L)
        val elapsedTime1 = estimator.elapsedTime
        val elapsedTime2 = Time(0.0, TimeUnit.NANOSECOND)
        estimator.getElapsedTime(elapsedTime2)
        assertEquals(elapsedTime1, elapsedTime2)
        assertEquals(elapsedTime.toDouble(), elapsedTime1.value.toDouble(), 0.0)
    }
}