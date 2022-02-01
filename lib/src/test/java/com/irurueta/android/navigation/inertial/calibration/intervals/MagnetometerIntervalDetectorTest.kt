package com.irurueta.android.navigation.inertial.calibration.intervals

import android.content.Context
import android.hardware.Sensor
import android.os.SystemClock
import androidx.test.core.app.ApplicationProvider
import com.irurueta.android.navigation.inertial.callPrivateFuncWithResult
import com.irurueta.android.navigation.inertial.collectors.MagnetometerSensorCollector
import com.irurueta.android.navigation.inertial.collectors.SensorAccuracy
import com.irurueta.android.navigation.inertial.collectors.SensorCollector
import com.irurueta.android.navigation.inertial.collectors.SensorDelay
import com.irurueta.android.navigation.inertial.getPrivateProperty
import com.irurueta.android.navigation.inertial.setPrivateProperty
import com.irurueta.navigation.frames.CoordinateTransformation
import com.irurueta.navigation.frames.FrameType
import com.irurueta.navigation.frames.NEDPosition
import com.irurueta.navigation.inertial.BodyMagneticFluxDensity
import com.irurueta.navigation.inertial.calibration.MagneticFluxDensityTriad
import com.irurueta.navigation.inertial.calibration.TimeIntervalEstimator
import com.irurueta.navigation.inertial.calibration.intervals.MagneticFluxDensityTriadStaticIntervalDetector
import com.irurueta.navigation.inertial.calibration.intervals.MagneticFluxDensityTriadStaticIntervalDetectorListener
import com.irurueta.navigation.inertial.calibration.intervals.TriadStaticIntervalDetector
import com.irurueta.navigation.inertial.calibration.noise.WindowedTriadNoiseEstimator
import com.irurueta.navigation.inertial.estimators.BodyMagneticFluxDensityEstimator
import com.irurueta.navigation.inertial.wmm.WMMEarthMagneticFluxDensityEstimator
import com.irurueta.statistics.UniformRandomizer
import com.irurueta.units.*
import io.mockk.every
import io.mockk.mockk
import io.mockk.spyk
import io.mockk.verify
import org.junit.Assert.*
import org.junit.Test
import org.junit.runner.RunWith
import org.robolectric.RobolectricTestRunner
import java.util.*


@RunWith(RobolectricTestRunner::class)
class MagnetometerIntervalDetectorTest {

    @Test
    fun constructor_whenContext_setsDefaultValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        // check default values
        assertSame(context, detector.context)
        assertEquals(MagnetometerSensorCollector.SensorType.MAGNETOMETER, detector.sensorType)
        assertEquals(SensorDelay.FASTEST, detector.sensorDelay)
        assertNull(detector.initializationStartedListener)
        assertNull(detector.initializationCompletedListener)
        assertNull(detector.errorListener)
        assertNull(detector.staticIntervalDetectedListener)
        assertNull(detector.dynamicIntervalDetectedListener)
        assertNull(detector.resetListener)
        assertNull(detector.measurementListener)
        assertNull(detector.accuracyChangedListener)
        assertNull(detector.sensor)
        assertEquals(TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE, detector.windowSize)
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES,
            detector.initialStaticSamples
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_THRESHOLD_FACTOR,
            detector.thresholdFactor,
            0.0
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR,
            detector.instantaneousNoiseLevelFactor,
            0.0
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
            detector.baseNoiseLevelAbsoluteThreshold,
            0.0
        )
        val baseNoiseLevelAbsoluteThreshold1 =
            detector.baseNoiseLevelAbsoluteThresholdAsMagneticFluxDensity
        assertEquals(
            detector.baseNoiseLevelAbsoluteThreshold,
            baseNoiseLevelAbsoluteThreshold1.value.toDouble(),
            0.0
        )
        assertEquals(MagneticFluxDensityUnit.TESLA, baseNoiseLevelAbsoluteThreshold1.unit)
        val baseNoiseLevelAbsoluteThreshold2 =
            MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getBaseNoiseLevelAbsoluteThresholdAsMagneticFluxDensity(
            baseNoiseLevelAbsoluteThreshold2
        )
        assertEquals(baseNoiseLevelAbsoluteThreshold1, baseNoiseLevelAbsoluteThreshold2)
        assertNull(detector.baseNoiseLevel)
        assertNull(detector.baseNoiseLevelAsMagneticFluxDensity)
        val baseNoiseLevel = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(detector.getBaseNoiseLevelAsMagneticFluxDensity(baseNoiseLevel))
        assertNull(detector.baseNoiseLevelPsd)
        assertNull(detector.baseNoiseLevelRootPsd)
        assertNull(detector.threshold)
        assertNull(detector.thresholdAsMagneticFluxDensity)
        val threshold = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(detector.getThresholdAsMeasurement(threshold))
        assertEquals(0.0, detector.accumulatedAvgX, 0.0)
        val accumulatedAvgX1 = detector.accumulatedAvgXAsMagneticFluxDensity
        assertEquals(0.0, accumulatedAvgX1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, accumulatedAvgX1.unit)
        val accumulatedAvgX2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getAccumulatedAvgXAsMagneticFluxDensity(accumulatedAvgX2)
        assertEquals(accumulatedAvgX1, accumulatedAvgX2)
        assertEquals(accumulatedAvgX1, accumulatedAvgX2)
        assertEquals(0.0, detector.accumulatedAvgY, 0.0)
        val accumulatedAvgY1 = detector.accumulatedAvgYAsMagneticFluxDensity
        assertEquals(0.0, accumulatedAvgY1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, accumulatedAvgY1.unit)
        val accumulatedAvgY2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getAccumulatedAvgYAsMagneticFluxDensity(accumulatedAvgY2)
        assertEquals(accumulatedAvgY1, accumulatedAvgY2)
        assertEquals(0.0, detector.accumulatedAvgZ, 0.0)
        val accumulatedAvgZ1 = detector.accumulatedAvgZAsMagneticFluxDensity
        assertEquals(0.0, accumulatedAvgZ1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, accumulatedAvgZ1.unit)
        val accumulatedAvgZ2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getAccumulatedAvgZAsMagneticFluxDensity(accumulatedAvgZ2)
        assertEquals(accumulatedAvgZ1, accumulatedAvgZ2)
        val accumulatedAvgTriad1 = detector.accumulatedAvgTriad
        assertEquals(0.0, accumulatedAvgTriad1.valueX, 0.0)
        assertEquals(0.0, accumulatedAvgTriad1.valueY, 0.0)
        assertEquals(0.0, accumulatedAvgTriad1.valueZ, 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, accumulatedAvgTriad1.unit)
        val accumulatedAvgTriad2 = MagneticFluxDensityTriad()
        detector.getAccumulatedAvgTriad(accumulatedAvgTriad2)
        assertEquals(accumulatedAvgTriad1, accumulatedAvgTriad2)
        assertEquals(0.0, detector.accumulatedStdX, 0.0)
        val accumulatedStdX1 = detector.accumulatedStdXAsMagneticFluxDensity
        assertEquals(0.0, accumulatedStdX1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, accumulatedStdX1.unit)
        val accumulatedStdX2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getAccumulatedStdXAsMagneticFluxDensity(accumulatedStdX2)
        assertEquals(accumulatedStdX1, accumulatedStdX2)
        assertEquals(0.0, detector.accumulatedStdY, 0.0)
        val accumulatedStdY1 = detector.accumulatedStdYAsMagneticFluxDensity
        assertEquals(0.0, accumulatedStdY1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, accumulatedStdY1.unit)
        val accumulatedStdY2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getAccumulatedStdYAsMagneticFluxDensity(accumulatedStdY2)
        assertEquals(accumulatedStdY1, accumulatedStdY2)
        assertEquals(0.0, detector.accumulatedStdZ, 0.0)
        val accumulatedStdZ1 = detector.accumulatedStdZAsMagneticFluxDensity
        assertEquals(0.0, accumulatedStdZ1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, accumulatedStdZ1.unit)
        val accumulatedStdZ2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getAccumulatedStdZAsMagneticFluxDensity(accumulatedStdZ2)
        assertEquals(accumulatedStdZ1, accumulatedStdZ2)
        val accumulatedStdTriad1 = detector.accumulatedStdTriad
        assertEquals(0.0, accumulatedStdTriad1.valueX, 0.0)
        assertEquals(0.0, accumulatedStdTriad1.valueY, 0.0)
        assertEquals(0.0, accumulatedStdTriad1.valueZ, 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, accumulatedStdTriad1.unit)
        val accumulatedStdTriad2 = MagneticFluxDensityTriad()
        detector.getAccumulatedStdTriad(accumulatedStdTriad2)
        assertEquals(accumulatedStdTriad1, accumulatedAvgTriad2)
        assertEquals(0.0, detector.instantaneousAvgX, 0.0)
        val instantaneousAvgX1 = detector.instantaneousAvgXAsMagneticFluxDensity
        assertEquals(0.0, instantaneousAvgX1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, instantaneousAvgX1.unit)
        val instantaneousAvgX2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getInstantaneousAvgXAsMagneticFluxDensity(instantaneousAvgX2)
        assertEquals(instantaneousAvgX1, instantaneousAvgX2)
        assertEquals(0.0, detector.instantaneousAvgY, 0.0)
        val instantaneousAvgY1 = detector.instantaneousAvgYAsMagneticFluxDensity
        assertEquals(0.0, instantaneousAvgY1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, instantaneousAvgY1.unit)
        val instantaneousAvgY2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getInstantaneousAvgYAsMagneticFluxDensity(instantaneousAvgY2)
        assertEquals(instantaneousAvgY1, instantaneousAvgY2)
        assertEquals(0.0, detector.instantaneousAvgZ, 0.0)
        val instantaneousAvgZ1 = detector.instantaneousAvgZAsMagneticFluxDensity
        assertEquals(0.0, instantaneousAvgZ1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, instantaneousAvgZ1.unit)
        val instantaneousAvgZ2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getInstantaneousAvgZAsMagneticFluxDensity(instantaneousAvgZ2)
        assertEquals(instantaneousAvgZ1, instantaneousAvgZ2)
        val instantaneousAvgTriad1 = detector.instantaneousAvgTriad
        assertEquals(0.0, instantaneousAvgTriad1.valueX, 0.0)
        assertEquals(0.0, instantaneousAvgTriad1.valueY, 0.0)
        assertEquals(0.0, instantaneousAvgTriad1.valueZ, 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, instantaneousAvgTriad1.unit)
        val instantaneousAvgTriad2 = MagneticFluxDensityTriad()
        detector.getInstantaneousAvgTriad(instantaneousAvgTriad2)
        assertEquals(instantaneousAvgTriad1, instantaneousAvgTriad2)
        assertEquals(0.0, detector.instantaneousStdX, 0.0)
        val instantaneousStdX1 = detector.instantaneousStdXAsMagneticFluxDensity
        assertEquals(0.0, instantaneousStdX1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, instantaneousStdX1.unit)
        val instantaneousStdX2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getInstantaneousStdXAsMagneticFluxDensity(instantaneousStdX2)
        assertEquals(instantaneousStdX1, instantaneousStdX2)
        assertEquals(0.0, detector.instantaneousStdY, 0.0)
        val instantaneousStdY1 = detector.instantaneousStdYAsMagneticFluxDensity
        assertEquals(0.0, instantaneousStdY1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, instantaneousStdY1.unit)
        val instantaneousStdY2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getInstantaneousStdYAsMagneticFluxDensity(instantaneousStdY2)
        assertEquals(instantaneousStdY1, instantaneousStdY2)
        assertEquals(0.0, detector.instantaneousStdZ, 0.0)
        val instantaneousStdZ1 = detector.instantaneousStdZAsMagneticFluxDensity
        assertEquals(0.0, instantaneousStdZ1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, instantaneousStdZ1.unit)
        val instantaneousStdZ2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getInstantaneousStdZAsMagneticFluxDensity(instantaneousStdZ2)
        assertEquals(instantaneousStdZ1, instantaneousStdZ2)
        val instantaneousStdTriad1 = detector.instantaneousStdTriad
        assertEquals(0.0, instantaneousAvgTriad1.valueX, 0.0)
        assertEquals(0.0, instantaneousStdTriad1.valueY, 0.0)
        assertEquals(0.0, instantaneousStdTriad1.valueZ, 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, instantaneousStdTriad1.unit)
        val instantaneousStdTriad2 = MagneticFluxDensityTriad()
        detector.getInstantaneousStdTriad(instantaneousStdTriad2)
        assertEquals(instantaneousStdTriad1, instantaneousStdTriad2)
        assertNull(detector.averageTimeInterval)
        assertNull(detector.averageTimeIntervalAsTime)
        val time = Time(0.0, TimeUnit.SECOND)
        assertFalse(detector.getAverageTimeIntervalAsTime(time))
        assertNull(detector.timeIntervalVariance)
        assertNull(detector.timeIntervalStandardDeviation)
        assertNull(detector.timeIntervalStandardDeviationAsTime)
        assertFalse(detector.getTimeIntervalStandardDeviationAsTime(time))
        assertEquals(0, detector.numberOfProcessedMeasurements)
        assertFalse(detector.running)
        assertEquals(MagnetometerIntervalDetector.Status.IDLE, detector.status)
    }

    @Test
    fun constructor_whenSensorType_setsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(
            context,
            MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED
        )

        // check default values
        assertSame(context, detector.context)
        assertEquals(
            MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED,
            detector.sensorType
        )
        assertEquals(SensorDelay.FASTEST, detector.sensorDelay)
        assertNull(detector.initializationStartedListener)
        assertNull(detector.initializationCompletedListener)
        assertNull(detector.errorListener)
        assertNull(detector.staticIntervalDetectedListener)
        assertNull(detector.dynamicIntervalDetectedListener)
        assertNull(detector.resetListener)
        assertNull(detector.measurementListener)
        assertNull(detector.accuracyChangedListener)
        assertNull(detector.sensor)
        assertEquals(TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE, detector.windowSize)
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES,
            detector.initialStaticSamples
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_THRESHOLD_FACTOR,
            detector.thresholdFactor,
            0.0
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR,
            detector.instantaneousNoiseLevelFactor,
            0.0
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
            detector.baseNoiseLevelAbsoluteThreshold,
            0.0
        )
        val baseNoiseLevelAbsoluteThreshold1 =
            detector.baseNoiseLevelAbsoluteThresholdAsMagneticFluxDensity
        assertEquals(
            detector.baseNoiseLevelAbsoluteThreshold,
            baseNoiseLevelAbsoluteThreshold1.value.toDouble(),
            0.0
        )
        assertEquals(MagneticFluxDensityUnit.TESLA, baseNoiseLevelAbsoluteThreshold1.unit)
        val baseNoiseLevelAbsoluteThreshold2 =
            MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getBaseNoiseLevelAbsoluteThresholdAsMagneticFluxDensity(
            baseNoiseLevelAbsoluteThreshold2
        )
        assertEquals(baseNoiseLevelAbsoluteThreshold1, baseNoiseLevelAbsoluteThreshold2)
        assertNull(detector.baseNoiseLevel)
        assertNull(detector.baseNoiseLevelAsMagneticFluxDensity)
        val baseNoiseLevel = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(detector.getBaseNoiseLevelAsMagneticFluxDensity(baseNoiseLevel))
        assertNull(detector.baseNoiseLevelPsd)
        assertNull(detector.baseNoiseLevelRootPsd)
        assertNull(detector.threshold)
        assertNull(detector.thresholdAsMagneticFluxDensity)
        val threshold = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(detector.getThresholdAsMeasurement(threshold))
        assertEquals(0.0, detector.accumulatedAvgX, 0.0)
        val accumulatedAvgX1 = detector.accumulatedAvgXAsMagneticFluxDensity
        assertEquals(0.0, accumulatedAvgX1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, accumulatedAvgX1.unit)
        val accumulatedAvgX2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getAccumulatedAvgXAsMagneticFluxDensity(accumulatedAvgX2)
        assertEquals(accumulatedAvgX1, accumulatedAvgX2)
        assertEquals(accumulatedAvgX1, accumulatedAvgX2)
        assertEquals(0.0, detector.accumulatedAvgY, 0.0)
        val accumulatedAvgY1 = detector.accumulatedAvgYAsMagneticFluxDensity
        assertEquals(0.0, accumulatedAvgY1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, accumulatedAvgY1.unit)
        val accumulatedAvgY2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getAccumulatedAvgYAsMagneticFluxDensity(accumulatedAvgY2)
        assertEquals(accumulatedAvgY1, accumulatedAvgY2)
        assertEquals(0.0, detector.accumulatedAvgZ, 0.0)
        val accumulatedAvgZ1 = detector.accumulatedAvgZAsMagneticFluxDensity
        assertEquals(0.0, accumulatedAvgZ1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, accumulatedAvgZ1.unit)
        val accumulatedAvgZ2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getAccumulatedAvgZAsMagneticFluxDensity(accumulatedAvgZ2)
        assertEquals(accumulatedAvgZ1, accumulatedAvgZ2)
        val accumulatedAvgTriad1 = detector.accumulatedAvgTriad
        assertEquals(0.0, accumulatedAvgTriad1.valueX, 0.0)
        assertEquals(0.0, accumulatedAvgTriad1.valueY, 0.0)
        assertEquals(0.0, accumulatedAvgTriad1.valueZ, 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, accumulatedAvgTriad1.unit)
        val accumulatedAvgTriad2 = MagneticFluxDensityTriad()
        detector.getAccumulatedAvgTriad(accumulatedAvgTriad2)
        assertEquals(accumulatedAvgTriad1, accumulatedAvgTriad2)
        assertEquals(0.0, detector.accumulatedStdX, 0.0)
        val accumulatedStdX1 = detector.accumulatedStdXAsMagneticFluxDensity
        assertEquals(0.0, accumulatedStdX1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, accumulatedStdX1.unit)
        val accumulatedStdX2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getAccumulatedStdXAsMagneticFluxDensity(accumulatedStdX2)
        assertEquals(accumulatedStdX1, accumulatedStdX2)
        assertEquals(0.0, detector.accumulatedStdY, 0.0)
        val accumulatedStdY1 = detector.accumulatedStdYAsMagneticFluxDensity
        assertEquals(0.0, accumulatedStdY1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, accumulatedStdY1.unit)
        val accumulatedStdY2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getAccumulatedStdYAsMagneticFluxDensity(accumulatedStdY2)
        assertEquals(accumulatedStdY1, accumulatedStdY2)
        assertEquals(0.0, detector.accumulatedStdZ, 0.0)
        val accumulatedStdZ1 = detector.accumulatedStdZAsMagneticFluxDensity
        assertEquals(0.0, accumulatedStdZ1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, accumulatedStdZ1.unit)
        val accumulatedStdZ2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getAccumulatedStdZAsMagneticFluxDensity(accumulatedStdZ2)
        assertEquals(accumulatedStdZ1, accumulatedStdZ2)
        val accumulatedStdTriad1 = detector.accumulatedStdTriad
        assertEquals(0.0, accumulatedStdTriad1.valueX, 0.0)
        assertEquals(0.0, accumulatedStdTriad1.valueY, 0.0)
        assertEquals(0.0, accumulatedStdTriad1.valueZ, 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, accumulatedStdTriad1.unit)
        val accumulatedStdTriad2 = MagneticFluxDensityTriad()
        detector.getAccumulatedStdTriad(accumulatedStdTriad2)
        assertEquals(accumulatedStdTriad1, accumulatedAvgTriad2)
        assertEquals(0.0, detector.instantaneousAvgX, 0.0)
        val instantaneousAvgX1 = detector.instantaneousAvgXAsMagneticFluxDensity
        assertEquals(0.0, instantaneousAvgX1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, instantaneousAvgX1.unit)
        val instantaneousAvgX2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getInstantaneousAvgXAsMagneticFluxDensity(instantaneousAvgX2)
        assertEquals(instantaneousAvgX1, instantaneousAvgX2)
        assertEquals(0.0, detector.instantaneousAvgY, 0.0)
        val instantaneousAvgY1 = detector.instantaneousAvgYAsMagneticFluxDensity
        assertEquals(0.0, instantaneousAvgY1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, instantaneousAvgY1.unit)
        val instantaneousAvgY2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getInstantaneousAvgYAsMagneticFluxDensity(instantaneousAvgY2)
        assertEquals(instantaneousAvgY1, instantaneousAvgY2)
        assertEquals(0.0, detector.instantaneousAvgZ, 0.0)
        val instantaneousAvgZ1 = detector.instantaneousAvgZAsMagneticFluxDensity
        assertEquals(0.0, instantaneousAvgZ1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, instantaneousAvgZ1.unit)
        val instantaneousAvgZ2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getInstantaneousAvgZAsMagneticFluxDensity(instantaneousAvgZ2)
        assertEquals(instantaneousAvgZ1, instantaneousAvgZ2)
        val instantaneousAvgTriad1 = detector.instantaneousAvgTriad
        assertEquals(0.0, instantaneousAvgTriad1.valueX, 0.0)
        assertEquals(0.0, instantaneousAvgTriad1.valueY, 0.0)
        assertEquals(0.0, instantaneousAvgTriad1.valueZ, 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, instantaneousAvgTriad1.unit)
        val instantaneousAvgTriad2 = MagneticFluxDensityTriad()
        detector.getInstantaneousAvgTriad(instantaneousAvgTriad2)
        assertEquals(instantaneousAvgTriad1, instantaneousAvgTriad2)
        assertEquals(0.0, detector.instantaneousStdX, 0.0)
        val instantaneousStdX1 = detector.instantaneousStdXAsMagneticFluxDensity
        assertEquals(0.0, instantaneousStdX1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, instantaneousStdX1.unit)
        val instantaneousStdX2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getInstantaneousStdXAsMagneticFluxDensity(instantaneousStdX2)
        assertEquals(instantaneousStdX1, instantaneousStdX2)
        assertEquals(0.0, detector.instantaneousStdY, 0.0)
        val instantaneousStdY1 = detector.instantaneousStdYAsMagneticFluxDensity
        assertEquals(0.0, instantaneousStdY1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, instantaneousStdY1.unit)
        val instantaneousStdY2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getInstantaneousStdYAsMagneticFluxDensity(instantaneousStdY2)
        assertEquals(instantaneousStdY1, instantaneousStdY2)
        assertEquals(0.0, detector.instantaneousStdZ, 0.0)
        val instantaneousStdZ1 = detector.instantaneousStdZAsMagneticFluxDensity
        assertEquals(0.0, instantaneousStdZ1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, instantaneousStdZ1.unit)
        val instantaneousStdZ2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getInstantaneousStdZAsMagneticFluxDensity(instantaneousStdZ2)
        assertEquals(instantaneousStdZ1, instantaneousStdZ2)
        val instantaneousStdTriad1 = detector.instantaneousStdTriad
        assertEquals(0.0, instantaneousAvgTriad1.valueX, 0.0)
        assertEquals(0.0, instantaneousStdTriad1.valueY, 0.0)
        assertEquals(0.0, instantaneousStdTriad1.valueZ, 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, instantaneousStdTriad1.unit)
        val instantaneousStdTriad2 = MagneticFluxDensityTriad()
        detector.getInstantaneousStdTriad(instantaneousStdTriad2)
        assertEquals(instantaneousStdTriad1, instantaneousStdTriad2)
        assertNull(detector.averageTimeInterval)
        assertNull(detector.averageTimeIntervalAsTime)
        val time = Time(0.0, TimeUnit.SECOND)
        assertFalse(detector.getAverageTimeIntervalAsTime(time))
        assertNull(detector.timeIntervalVariance)
        assertNull(detector.timeIntervalStandardDeviation)
        assertNull(detector.timeIntervalStandardDeviationAsTime)
        assertFalse(detector.getTimeIntervalStandardDeviationAsTime(time))
        assertEquals(0, detector.numberOfProcessedMeasurements)
        assertFalse(detector.running)
        assertEquals(MagnetometerIntervalDetector.Status.IDLE, detector.status)
    }

    @Test
    fun constructor_whenSensorDelay_setsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(
            context,
            MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED,
            SensorDelay.NORMAL
        )

        // check default values
        assertSame(context, detector.context)
        assertEquals(
            MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED,
            detector.sensorType
        )
        assertEquals(SensorDelay.NORMAL, detector.sensorDelay)
        assertNull(detector.initializationStartedListener)
        assertNull(detector.initializationCompletedListener)
        assertNull(detector.errorListener)
        assertNull(detector.staticIntervalDetectedListener)
        assertNull(detector.dynamicIntervalDetectedListener)
        assertNull(detector.resetListener)
        assertNull(detector.measurementListener)
        assertNull(detector.accuracyChangedListener)
        assertNull(detector.sensor)
        assertEquals(TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE, detector.windowSize)
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES,
            detector.initialStaticSamples
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_THRESHOLD_FACTOR,
            detector.thresholdFactor,
            0.0
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR,
            detector.instantaneousNoiseLevelFactor,
            0.0
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
            detector.baseNoiseLevelAbsoluteThreshold,
            0.0
        )
        val baseNoiseLevelAbsoluteThreshold1 =
            detector.baseNoiseLevelAbsoluteThresholdAsMagneticFluxDensity
        assertEquals(
            detector.baseNoiseLevelAbsoluteThreshold,
            baseNoiseLevelAbsoluteThreshold1.value.toDouble(),
            0.0
        )
        assertEquals(MagneticFluxDensityUnit.TESLA, baseNoiseLevelAbsoluteThreshold1.unit)
        val baseNoiseLevelAbsoluteThreshold2 =
            MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getBaseNoiseLevelAbsoluteThresholdAsMagneticFluxDensity(
            baseNoiseLevelAbsoluteThreshold2
        )
        assertEquals(baseNoiseLevelAbsoluteThreshold1, baseNoiseLevelAbsoluteThreshold2)
        assertNull(detector.baseNoiseLevel)
        assertNull(detector.baseNoiseLevelAsMagneticFluxDensity)
        val baseNoiseLevel = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(detector.getBaseNoiseLevelAsMagneticFluxDensity(baseNoiseLevel))
        assertNull(detector.baseNoiseLevelPsd)
        assertNull(detector.baseNoiseLevelRootPsd)
        assertNull(detector.threshold)
        assertNull(detector.thresholdAsMagneticFluxDensity)
        val threshold = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(detector.getThresholdAsMeasurement(threshold))
        assertEquals(0.0, detector.accumulatedAvgX, 0.0)
        val accumulatedAvgX1 = detector.accumulatedAvgXAsMagneticFluxDensity
        assertEquals(0.0, accumulatedAvgX1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, accumulatedAvgX1.unit)
        val accumulatedAvgX2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getAccumulatedAvgXAsMagneticFluxDensity(accumulatedAvgX2)
        assertEquals(accumulatedAvgX1, accumulatedAvgX2)
        assertEquals(accumulatedAvgX1, accumulatedAvgX2)
        assertEquals(0.0, detector.accumulatedAvgY, 0.0)
        val accumulatedAvgY1 = detector.accumulatedAvgYAsMagneticFluxDensity
        assertEquals(0.0, accumulatedAvgY1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, accumulatedAvgY1.unit)
        val accumulatedAvgY2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getAccumulatedAvgYAsMagneticFluxDensity(accumulatedAvgY2)
        assertEquals(accumulatedAvgY1, accumulatedAvgY2)
        assertEquals(0.0, detector.accumulatedAvgZ, 0.0)
        val accumulatedAvgZ1 = detector.accumulatedAvgZAsMagneticFluxDensity
        assertEquals(0.0, accumulatedAvgZ1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, accumulatedAvgZ1.unit)
        val accumulatedAvgZ2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getAccumulatedAvgZAsMagneticFluxDensity(accumulatedAvgZ2)
        assertEquals(accumulatedAvgZ1, accumulatedAvgZ2)
        val accumulatedAvgTriad1 = detector.accumulatedAvgTriad
        assertEquals(0.0, accumulatedAvgTriad1.valueX, 0.0)
        assertEquals(0.0, accumulatedAvgTriad1.valueY, 0.0)
        assertEquals(0.0, accumulatedAvgTriad1.valueZ, 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, accumulatedAvgTriad1.unit)
        val accumulatedAvgTriad2 = MagneticFluxDensityTriad()
        detector.getAccumulatedAvgTriad(accumulatedAvgTriad2)
        assertEquals(accumulatedAvgTriad1, accumulatedAvgTriad2)
        assertEquals(0.0, detector.accumulatedStdX, 0.0)
        val accumulatedStdX1 = detector.accumulatedStdXAsMagneticFluxDensity
        assertEquals(0.0, accumulatedStdX1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, accumulatedStdX1.unit)
        val accumulatedStdX2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getAccumulatedStdXAsMagneticFluxDensity(accumulatedStdX2)
        assertEquals(accumulatedStdX1, accumulatedStdX2)
        assertEquals(0.0, detector.accumulatedStdY, 0.0)
        val accumulatedStdY1 = detector.accumulatedStdYAsMagneticFluxDensity
        assertEquals(0.0, accumulatedStdY1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, accumulatedStdY1.unit)
        val accumulatedStdY2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getAccumulatedStdYAsMagneticFluxDensity(accumulatedStdY2)
        assertEquals(accumulatedStdY1, accumulatedStdY2)
        assertEquals(0.0, detector.accumulatedStdZ, 0.0)
        val accumulatedStdZ1 = detector.accumulatedStdZAsMagneticFluxDensity
        assertEquals(0.0, accumulatedStdZ1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, accumulatedStdZ1.unit)
        val accumulatedStdZ2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getAccumulatedStdZAsMagneticFluxDensity(accumulatedStdZ2)
        assertEquals(accumulatedStdZ1, accumulatedStdZ2)
        val accumulatedStdTriad1 = detector.accumulatedStdTriad
        assertEquals(0.0, accumulatedStdTriad1.valueX, 0.0)
        assertEquals(0.0, accumulatedStdTriad1.valueY, 0.0)
        assertEquals(0.0, accumulatedStdTriad1.valueZ, 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, accumulatedStdTriad1.unit)
        val accumulatedStdTriad2 = MagneticFluxDensityTriad()
        detector.getAccumulatedStdTriad(accumulatedStdTriad2)
        assertEquals(accumulatedStdTriad1, accumulatedAvgTriad2)
        assertEquals(0.0, detector.instantaneousAvgX, 0.0)
        val instantaneousAvgX1 = detector.instantaneousAvgXAsMagneticFluxDensity
        assertEquals(0.0, instantaneousAvgX1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, instantaneousAvgX1.unit)
        val instantaneousAvgX2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getInstantaneousAvgXAsMagneticFluxDensity(instantaneousAvgX2)
        assertEquals(instantaneousAvgX1, instantaneousAvgX2)
        assertEquals(0.0, detector.instantaneousAvgY, 0.0)
        val instantaneousAvgY1 = detector.instantaneousAvgYAsMagneticFluxDensity
        assertEquals(0.0, instantaneousAvgY1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, instantaneousAvgY1.unit)
        val instantaneousAvgY2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getInstantaneousAvgYAsMagneticFluxDensity(instantaneousAvgY2)
        assertEquals(instantaneousAvgY1, instantaneousAvgY2)
        assertEquals(0.0, detector.instantaneousAvgZ, 0.0)
        val instantaneousAvgZ1 = detector.instantaneousAvgZAsMagneticFluxDensity
        assertEquals(0.0, instantaneousAvgZ1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, instantaneousAvgZ1.unit)
        val instantaneousAvgZ2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getInstantaneousAvgZAsMagneticFluxDensity(instantaneousAvgZ2)
        assertEquals(instantaneousAvgZ1, instantaneousAvgZ2)
        val instantaneousAvgTriad1 = detector.instantaneousAvgTriad
        assertEquals(0.0, instantaneousAvgTriad1.valueX, 0.0)
        assertEquals(0.0, instantaneousAvgTriad1.valueY, 0.0)
        assertEquals(0.0, instantaneousAvgTriad1.valueZ, 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, instantaneousAvgTriad1.unit)
        val instantaneousAvgTriad2 = MagneticFluxDensityTriad()
        detector.getInstantaneousAvgTriad(instantaneousAvgTriad2)
        assertEquals(instantaneousAvgTriad1, instantaneousAvgTriad2)
        assertEquals(0.0, detector.instantaneousStdX, 0.0)
        val instantaneousStdX1 = detector.instantaneousStdXAsMagneticFluxDensity
        assertEquals(0.0, instantaneousStdX1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, instantaneousStdX1.unit)
        val instantaneousStdX2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getInstantaneousStdXAsMagneticFluxDensity(instantaneousStdX2)
        assertEquals(instantaneousStdX1, instantaneousStdX2)
        assertEquals(0.0, detector.instantaneousStdY, 0.0)
        val instantaneousStdY1 = detector.instantaneousStdYAsMagneticFluxDensity
        assertEquals(0.0, instantaneousStdY1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, instantaneousStdY1.unit)
        val instantaneousStdY2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getInstantaneousStdYAsMagneticFluxDensity(instantaneousStdY2)
        assertEquals(instantaneousStdY1, instantaneousStdY2)
        assertEquals(0.0, detector.instantaneousStdZ, 0.0)
        val instantaneousStdZ1 = detector.instantaneousStdZAsMagneticFluxDensity
        assertEquals(0.0, instantaneousStdZ1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, instantaneousStdZ1.unit)
        val instantaneousStdZ2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getInstantaneousStdZAsMagneticFluxDensity(instantaneousStdZ2)
        assertEquals(instantaneousStdZ1, instantaneousStdZ2)
        val instantaneousStdTriad1 = detector.instantaneousStdTriad
        assertEquals(0.0, instantaneousAvgTriad1.valueX, 0.0)
        assertEquals(0.0, instantaneousStdTriad1.valueY, 0.0)
        assertEquals(0.0, instantaneousStdTriad1.valueZ, 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, instantaneousStdTriad1.unit)
        val instantaneousStdTriad2 = MagneticFluxDensityTriad()
        detector.getInstantaneousStdTriad(instantaneousStdTriad2)
        assertEquals(instantaneousStdTriad1, instantaneousStdTriad2)
        assertNull(detector.averageTimeInterval)
        assertNull(detector.averageTimeIntervalAsTime)
        val time = Time(0.0, TimeUnit.SECOND)
        assertFalse(detector.getAverageTimeIntervalAsTime(time))
        assertNull(detector.timeIntervalVariance)
        assertNull(detector.timeIntervalStandardDeviation)
        assertNull(detector.timeIntervalStandardDeviationAsTime)
        assertFalse(detector.getTimeIntervalStandardDeviationAsTime(time))
        assertEquals(0, detector.numberOfProcessedMeasurements)
        assertFalse(detector.running)
        assertEquals(MagnetometerIntervalDetector.Status.IDLE, detector.status)
    }

    @Test
    fun constructor_whenInitializationStartedListener_setsExpectedValues() {
        val initializationStartedListener =
            mockk<MagnetometerIntervalDetector.OnInitializationStartedListener>()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(
            context,
            MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED,
            SensorDelay.NORMAL,
            initializationStartedListener
        )

        // check default values
        assertSame(context, detector.context)
        assertEquals(
            MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED,
            detector.sensorType
        )
        assertEquals(SensorDelay.NORMAL, detector.sensorDelay)
        assertSame(initializationStartedListener, detector.initializationStartedListener)
        assertNull(detector.initializationCompletedListener)
        assertNull(detector.errorListener)
        assertNull(detector.staticIntervalDetectedListener)
        assertNull(detector.dynamicIntervalDetectedListener)
        assertNull(detector.resetListener)
        assertNull(detector.measurementListener)
        assertNull(detector.accuracyChangedListener)
        assertNull(detector.sensor)
        assertEquals(TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE, detector.windowSize)
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES,
            detector.initialStaticSamples
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_THRESHOLD_FACTOR,
            detector.thresholdFactor,
            0.0
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR,
            detector.instantaneousNoiseLevelFactor,
            0.0
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
            detector.baseNoiseLevelAbsoluteThreshold,
            0.0
        )
        val baseNoiseLevelAbsoluteThreshold1 =
            detector.baseNoiseLevelAbsoluteThresholdAsMagneticFluxDensity
        assertEquals(
            detector.baseNoiseLevelAbsoluteThreshold,
            baseNoiseLevelAbsoluteThreshold1.value.toDouble(),
            0.0
        )
        assertEquals(MagneticFluxDensityUnit.TESLA, baseNoiseLevelAbsoluteThreshold1.unit)
        val baseNoiseLevelAbsoluteThreshold2 =
            MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getBaseNoiseLevelAbsoluteThresholdAsMagneticFluxDensity(
            baseNoiseLevelAbsoluteThreshold2
        )
        assertEquals(baseNoiseLevelAbsoluteThreshold1, baseNoiseLevelAbsoluteThreshold2)
        assertNull(detector.baseNoiseLevel)
        assertNull(detector.baseNoiseLevelAsMagneticFluxDensity)
        val baseNoiseLevel = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(detector.getBaseNoiseLevelAsMagneticFluxDensity(baseNoiseLevel))
        assertNull(detector.baseNoiseLevelPsd)
        assertNull(detector.baseNoiseLevelRootPsd)
        assertNull(detector.threshold)
        assertNull(detector.thresholdAsMagneticFluxDensity)
        val threshold = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(detector.getThresholdAsMeasurement(threshold))
        assertEquals(0.0, detector.accumulatedAvgX, 0.0)
        val accumulatedAvgX1 = detector.accumulatedAvgXAsMagneticFluxDensity
        assertEquals(0.0, accumulatedAvgX1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, accumulatedAvgX1.unit)
        val accumulatedAvgX2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getAccumulatedAvgXAsMagneticFluxDensity(accumulatedAvgX2)
        assertEquals(accumulatedAvgX1, accumulatedAvgX2)
        assertEquals(accumulatedAvgX1, accumulatedAvgX2)
        assertEquals(0.0, detector.accumulatedAvgY, 0.0)
        val accumulatedAvgY1 = detector.accumulatedAvgYAsMagneticFluxDensity
        assertEquals(0.0, accumulatedAvgY1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, accumulatedAvgY1.unit)
        val accumulatedAvgY2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getAccumulatedAvgYAsMagneticFluxDensity(accumulatedAvgY2)
        assertEquals(accumulatedAvgY1, accumulatedAvgY2)
        assertEquals(0.0, detector.accumulatedAvgZ, 0.0)
        val accumulatedAvgZ1 = detector.accumulatedAvgZAsMagneticFluxDensity
        assertEquals(0.0, accumulatedAvgZ1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, accumulatedAvgZ1.unit)
        val accumulatedAvgZ2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getAccumulatedAvgZAsMagneticFluxDensity(accumulatedAvgZ2)
        assertEquals(accumulatedAvgZ1, accumulatedAvgZ2)
        val accumulatedAvgTriad1 = detector.accumulatedAvgTriad
        assertEquals(0.0, accumulatedAvgTriad1.valueX, 0.0)
        assertEquals(0.0, accumulatedAvgTriad1.valueY, 0.0)
        assertEquals(0.0, accumulatedAvgTriad1.valueZ, 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, accumulatedAvgTriad1.unit)
        val accumulatedAvgTriad2 = MagneticFluxDensityTriad()
        detector.getAccumulatedAvgTriad(accumulatedAvgTriad2)
        assertEquals(accumulatedAvgTriad1, accumulatedAvgTriad2)
        assertEquals(0.0, detector.accumulatedStdX, 0.0)
        val accumulatedStdX1 = detector.accumulatedStdXAsMagneticFluxDensity
        assertEquals(0.0, accumulatedStdX1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, accumulatedStdX1.unit)
        val accumulatedStdX2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getAccumulatedStdXAsMagneticFluxDensity(accumulatedStdX2)
        assertEquals(accumulatedStdX1, accumulatedStdX2)
        assertEquals(0.0, detector.accumulatedStdY, 0.0)
        val accumulatedStdY1 = detector.accumulatedStdYAsMagneticFluxDensity
        assertEquals(0.0, accumulatedStdY1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, accumulatedStdY1.unit)
        val accumulatedStdY2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getAccumulatedStdYAsMagneticFluxDensity(accumulatedStdY2)
        assertEquals(accumulatedStdY1, accumulatedStdY2)
        assertEquals(0.0, detector.accumulatedStdZ, 0.0)
        val accumulatedStdZ1 = detector.accumulatedStdZAsMagneticFluxDensity
        assertEquals(0.0, accumulatedStdZ1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, accumulatedStdZ1.unit)
        val accumulatedStdZ2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getAccumulatedStdZAsMagneticFluxDensity(accumulatedStdZ2)
        assertEquals(accumulatedStdZ1, accumulatedStdZ2)
        val accumulatedStdTriad1 = detector.accumulatedStdTriad
        assertEquals(0.0, accumulatedStdTriad1.valueX, 0.0)
        assertEquals(0.0, accumulatedStdTriad1.valueY, 0.0)
        assertEquals(0.0, accumulatedStdTriad1.valueZ, 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, accumulatedStdTriad1.unit)
        val accumulatedStdTriad2 = MagneticFluxDensityTriad()
        detector.getAccumulatedStdTriad(accumulatedStdTriad2)
        assertEquals(accumulatedStdTriad1, accumulatedAvgTriad2)
        assertEquals(0.0, detector.instantaneousAvgX, 0.0)
        val instantaneousAvgX1 = detector.instantaneousAvgXAsMagneticFluxDensity
        assertEquals(0.0, instantaneousAvgX1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, instantaneousAvgX1.unit)
        val instantaneousAvgX2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getInstantaneousAvgXAsMagneticFluxDensity(instantaneousAvgX2)
        assertEquals(instantaneousAvgX1, instantaneousAvgX2)
        assertEquals(0.0, detector.instantaneousAvgY, 0.0)
        val instantaneousAvgY1 = detector.instantaneousAvgYAsMagneticFluxDensity
        assertEquals(0.0, instantaneousAvgY1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, instantaneousAvgY1.unit)
        val instantaneousAvgY2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getInstantaneousAvgYAsMagneticFluxDensity(instantaneousAvgY2)
        assertEquals(instantaneousAvgY1, instantaneousAvgY2)
        assertEquals(0.0, detector.instantaneousAvgZ, 0.0)
        val instantaneousAvgZ1 = detector.instantaneousAvgZAsMagneticFluxDensity
        assertEquals(0.0, instantaneousAvgZ1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, instantaneousAvgZ1.unit)
        val instantaneousAvgZ2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getInstantaneousAvgZAsMagneticFluxDensity(instantaneousAvgZ2)
        assertEquals(instantaneousAvgZ1, instantaneousAvgZ2)
        val instantaneousAvgTriad1 = detector.instantaneousAvgTriad
        assertEquals(0.0, instantaneousAvgTriad1.valueX, 0.0)
        assertEquals(0.0, instantaneousAvgTriad1.valueY, 0.0)
        assertEquals(0.0, instantaneousAvgTriad1.valueZ, 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, instantaneousAvgTriad1.unit)
        val instantaneousAvgTriad2 = MagneticFluxDensityTriad()
        detector.getInstantaneousAvgTriad(instantaneousAvgTriad2)
        assertEquals(instantaneousAvgTriad1, instantaneousAvgTriad2)
        assertEquals(0.0, detector.instantaneousStdX, 0.0)
        val instantaneousStdX1 = detector.instantaneousStdXAsMagneticFluxDensity
        assertEquals(0.0, instantaneousStdX1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, instantaneousStdX1.unit)
        val instantaneousStdX2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getInstantaneousStdXAsMagneticFluxDensity(instantaneousStdX2)
        assertEquals(instantaneousStdX1, instantaneousStdX2)
        assertEquals(0.0, detector.instantaneousStdY, 0.0)
        val instantaneousStdY1 = detector.instantaneousStdYAsMagneticFluxDensity
        assertEquals(0.0, instantaneousStdY1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, instantaneousStdY1.unit)
        val instantaneousStdY2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getInstantaneousStdYAsMagneticFluxDensity(instantaneousStdY2)
        assertEquals(instantaneousStdY1, instantaneousStdY2)
        assertEquals(0.0, detector.instantaneousStdZ, 0.0)
        val instantaneousStdZ1 = detector.instantaneousStdZAsMagneticFluxDensity
        assertEquals(0.0, instantaneousStdZ1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, instantaneousStdZ1.unit)
        val instantaneousStdZ2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getInstantaneousStdZAsMagneticFluxDensity(instantaneousStdZ2)
        assertEquals(instantaneousStdZ1, instantaneousStdZ2)
        val instantaneousStdTriad1 = detector.instantaneousStdTriad
        assertEquals(0.0, instantaneousAvgTriad1.valueX, 0.0)
        assertEquals(0.0, instantaneousStdTriad1.valueY, 0.0)
        assertEquals(0.0, instantaneousStdTriad1.valueZ, 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, instantaneousStdTriad1.unit)
        val instantaneousStdTriad2 = MagneticFluxDensityTriad()
        detector.getInstantaneousStdTriad(instantaneousStdTriad2)
        assertEquals(instantaneousStdTriad1, instantaneousStdTriad2)
        assertNull(detector.averageTimeInterval)
        assertNull(detector.averageTimeIntervalAsTime)
        val time = Time(0.0, TimeUnit.SECOND)
        assertFalse(detector.getAverageTimeIntervalAsTime(time))
        assertNull(detector.timeIntervalVariance)
        assertNull(detector.timeIntervalStandardDeviation)
        assertNull(detector.timeIntervalStandardDeviationAsTime)
        assertFalse(detector.getTimeIntervalStandardDeviationAsTime(time))
        assertEquals(0, detector.numberOfProcessedMeasurements)
        assertFalse(detector.running)
        assertEquals(MagnetometerIntervalDetector.Status.IDLE, detector.status)
    }

    @Test
    fun constructor_whenInitializationCompletedListener_setsExpectedValues() {
        val initializationStartedListener =
            mockk<MagnetometerIntervalDetector.OnInitializationStartedListener>()
        val initializationCompletedListener =
            mockk<MagnetometerIntervalDetector.OnInitializationCompletedListener>()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(
            context,
            MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED,
            SensorDelay.NORMAL,
            initializationStartedListener,
            initializationCompletedListener
        )

        // check default values
        assertSame(context, detector.context)
        assertEquals(
            MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED,
            detector.sensorType
        )
        assertEquals(SensorDelay.NORMAL, detector.sensorDelay)
        assertSame(initializationStartedListener, detector.initializationStartedListener)
        assertSame(initializationCompletedListener, detector.initializationCompletedListener)
        assertNull(detector.errorListener)
        assertNull(detector.staticIntervalDetectedListener)
        assertNull(detector.dynamicIntervalDetectedListener)
        assertNull(detector.resetListener)
        assertNull(detector.measurementListener)
        assertNull(detector.accuracyChangedListener)
        assertNull(detector.sensor)
        assertEquals(TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE, detector.windowSize)
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES,
            detector.initialStaticSamples
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_THRESHOLD_FACTOR,
            detector.thresholdFactor,
            0.0
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR,
            detector.instantaneousNoiseLevelFactor,
            0.0
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
            detector.baseNoiseLevelAbsoluteThreshold,
            0.0
        )
        val baseNoiseLevelAbsoluteThreshold1 =
            detector.baseNoiseLevelAbsoluteThresholdAsMagneticFluxDensity
        assertEquals(
            detector.baseNoiseLevelAbsoluteThreshold,
            baseNoiseLevelAbsoluteThreshold1.value.toDouble(),
            0.0
        )
        assertEquals(MagneticFluxDensityUnit.TESLA, baseNoiseLevelAbsoluteThreshold1.unit)
        val baseNoiseLevelAbsoluteThreshold2 =
            MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getBaseNoiseLevelAbsoluteThresholdAsMagneticFluxDensity(
            baseNoiseLevelAbsoluteThreshold2
        )
        assertEquals(baseNoiseLevelAbsoluteThreshold1, baseNoiseLevelAbsoluteThreshold2)
        assertNull(detector.baseNoiseLevel)
        assertNull(detector.baseNoiseLevelAsMagneticFluxDensity)
        val baseNoiseLevel = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(detector.getBaseNoiseLevelAsMagneticFluxDensity(baseNoiseLevel))
        assertNull(detector.baseNoiseLevelPsd)
        assertNull(detector.baseNoiseLevelRootPsd)
        assertNull(detector.threshold)
        assertNull(detector.thresholdAsMagneticFluxDensity)
        val threshold = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(detector.getThresholdAsMeasurement(threshold))
        assertEquals(0.0, detector.accumulatedAvgX, 0.0)
        val accumulatedAvgX1 = detector.accumulatedAvgXAsMagneticFluxDensity
        assertEquals(0.0, accumulatedAvgX1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, accumulatedAvgX1.unit)
        val accumulatedAvgX2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getAccumulatedAvgXAsMagneticFluxDensity(accumulatedAvgX2)
        assertEquals(accumulatedAvgX1, accumulatedAvgX2)
        assertEquals(accumulatedAvgX1, accumulatedAvgX2)
        assertEquals(0.0, detector.accumulatedAvgY, 0.0)
        val accumulatedAvgY1 = detector.accumulatedAvgYAsMagneticFluxDensity
        assertEquals(0.0, accumulatedAvgY1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, accumulatedAvgY1.unit)
        val accumulatedAvgY2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getAccumulatedAvgYAsMagneticFluxDensity(accumulatedAvgY2)
        assertEquals(accumulatedAvgY1, accumulatedAvgY2)
        assertEquals(0.0, detector.accumulatedAvgZ, 0.0)
        val accumulatedAvgZ1 = detector.accumulatedAvgZAsMagneticFluxDensity
        assertEquals(0.0, accumulatedAvgZ1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, accumulatedAvgZ1.unit)
        val accumulatedAvgZ2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getAccumulatedAvgZAsMagneticFluxDensity(accumulatedAvgZ2)
        assertEquals(accumulatedAvgZ1, accumulatedAvgZ2)
        val accumulatedAvgTriad1 = detector.accumulatedAvgTriad
        assertEquals(0.0, accumulatedAvgTriad1.valueX, 0.0)
        assertEquals(0.0, accumulatedAvgTriad1.valueY, 0.0)
        assertEquals(0.0, accumulatedAvgTriad1.valueZ, 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, accumulatedAvgTriad1.unit)
        val accumulatedAvgTriad2 = MagneticFluxDensityTriad()
        detector.getAccumulatedAvgTriad(accumulatedAvgTriad2)
        assertEquals(accumulatedAvgTriad1, accumulatedAvgTriad2)
        assertEquals(0.0, detector.accumulatedStdX, 0.0)
        val accumulatedStdX1 = detector.accumulatedStdXAsMagneticFluxDensity
        assertEquals(0.0, accumulatedStdX1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, accumulatedStdX1.unit)
        val accumulatedStdX2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getAccumulatedStdXAsMagneticFluxDensity(accumulatedStdX2)
        assertEquals(accumulatedStdX1, accumulatedStdX2)
        assertEquals(0.0, detector.accumulatedStdY, 0.0)
        val accumulatedStdY1 = detector.accumulatedStdYAsMagneticFluxDensity
        assertEquals(0.0, accumulatedStdY1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, accumulatedStdY1.unit)
        val accumulatedStdY2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getAccumulatedStdYAsMagneticFluxDensity(accumulatedStdY2)
        assertEquals(accumulatedStdY1, accumulatedStdY2)
        assertEquals(0.0, detector.accumulatedStdZ, 0.0)
        val accumulatedStdZ1 = detector.accumulatedStdZAsMagneticFluxDensity
        assertEquals(0.0, accumulatedStdZ1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, accumulatedStdZ1.unit)
        val accumulatedStdZ2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getAccumulatedStdZAsMagneticFluxDensity(accumulatedStdZ2)
        assertEquals(accumulatedStdZ1, accumulatedStdZ2)
        val accumulatedStdTriad1 = detector.accumulatedStdTriad
        assertEquals(0.0, accumulatedStdTriad1.valueX, 0.0)
        assertEquals(0.0, accumulatedStdTriad1.valueY, 0.0)
        assertEquals(0.0, accumulatedStdTriad1.valueZ, 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, accumulatedStdTriad1.unit)
        val accumulatedStdTriad2 = MagneticFluxDensityTriad()
        detector.getAccumulatedStdTriad(accumulatedStdTriad2)
        assertEquals(accumulatedStdTriad1, accumulatedAvgTriad2)
        assertEquals(0.0, detector.instantaneousAvgX, 0.0)
        val instantaneousAvgX1 = detector.instantaneousAvgXAsMagneticFluxDensity
        assertEquals(0.0, instantaneousAvgX1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, instantaneousAvgX1.unit)
        val instantaneousAvgX2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getInstantaneousAvgXAsMagneticFluxDensity(instantaneousAvgX2)
        assertEquals(instantaneousAvgX1, instantaneousAvgX2)
        assertEquals(0.0, detector.instantaneousAvgY, 0.0)
        val instantaneousAvgY1 = detector.instantaneousAvgYAsMagneticFluxDensity
        assertEquals(0.0, instantaneousAvgY1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, instantaneousAvgY1.unit)
        val instantaneousAvgY2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getInstantaneousAvgYAsMagneticFluxDensity(instantaneousAvgY2)
        assertEquals(instantaneousAvgY1, instantaneousAvgY2)
        assertEquals(0.0, detector.instantaneousAvgZ, 0.0)
        val instantaneousAvgZ1 = detector.instantaneousAvgZAsMagneticFluxDensity
        assertEquals(0.0, instantaneousAvgZ1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, instantaneousAvgZ1.unit)
        val instantaneousAvgZ2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getInstantaneousAvgZAsMagneticFluxDensity(instantaneousAvgZ2)
        assertEquals(instantaneousAvgZ1, instantaneousAvgZ2)
        val instantaneousAvgTriad1 = detector.instantaneousAvgTriad
        assertEquals(0.0, instantaneousAvgTriad1.valueX, 0.0)
        assertEquals(0.0, instantaneousAvgTriad1.valueY, 0.0)
        assertEquals(0.0, instantaneousAvgTriad1.valueZ, 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, instantaneousAvgTriad1.unit)
        val instantaneousAvgTriad2 = MagneticFluxDensityTriad()
        detector.getInstantaneousAvgTriad(instantaneousAvgTriad2)
        assertEquals(instantaneousAvgTriad1, instantaneousAvgTriad2)
        assertEquals(0.0, detector.instantaneousStdX, 0.0)
        val instantaneousStdX1 = detector.instantaneousStdXAsMagneticFluxDensity
        assertEquals(0.0, instantaneousStdX1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, instantaneousStdX1.unit)
        val instantaneousStdX2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getInstantaneousStdXAsMagneticFluxDensity(instantaneousStdX2)
        assertEquals(instantaneousStdX1, instantaneousStdX2)
        assertEquals(0.0, detector.instantaneousStdY, 0.0)
        val instantaneousStdY1 = detector.instantaneousStdYAsMagneticFluxDensity
        assertEquals(0.0, instantaneousStdY1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, instantaneousStdY1.unit)
        val instantaneousStdY2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getInstantaneousStdYAsMagneticFluxDensity(instantaneousStdY2)
        assertEquals(instantaneousStdY1, instantaneousStdY2)
        assertEquals(0.0, detector.instantaneousStdZ, 0.0)
        val instantaneousStdZ1 = detector.instantaneousStdZAsMagneticFluxDensity
        assertEquals(0.0, instantaneousStdZ1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, instantaneousStdZ1.unit)
        val instantaneousStdZ2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getInstantaneousStdZAsMagneticFluxDensity(instantaneousStdZ2)
        assertEquals(instantaneousStdZ1, instantaneousStdZ2)
        val instantaneousStdTriad1 = detector.instantaneousStdTriad
        assertEquals(0.0, instantaneousAvgTriad1.valueX, 0.0)
        assertEquals(0.0, instantaneousStdTriad1.valueY, 0.0)
        assertEquals(0.0, instantaneousStdTriad1.valueZ, 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, instantaneousStdTriad1.unit)
        val instantaneousStdTriad2 = MagneticFluxDensityTriad()
        detector.getInstantaneousStdTriad(instantaneousStdTriad2)
        assertEquals(instantaneousStdTriad1, instantaneousStdTriad2)
        assertNull(detector.averageTimeInterval)
        assertNull(detector.averageTimeIntervalAsTime)
        val time = Time(0.0, TimeUnit.SECOND)
        assertFalse(detector.getAverageTimeIntervalAsTime(time))
        assertNull(detector.timeIntervalVariance)
        assertNull(detector.timeIntervalStandardDeviation)
        assertNull(detector.timeIntervalStandardDeviationAsTime)
        assertFalse(detector.getTimeIntervalStandardDeviationAsTime(time))
        assertEquals(0, detector.numberOfProcessedMeasurements)
        assertFalse(detector.running)
        assertEquals(MagnetometerIntervalDetector.Status.IDLE, detector.status)
    }

    @Test
    fun constructor_whenErrorListener_setsExpectedValues() {
        val initializationStartedListener =
            mockk<MagnetometerIntervalDetector.OnInitializationStartedListener>()
        val initializationCompletedListener =
            mockk<MagnetometerIntervalDetector.OnInitializationCompletedListener>()
        val errorListener = mockk<MagnetometerIntervalDetector.OnErrorListener>()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(
            context,
            MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED,
            SensorDelay.NORMAL,
            initializationStartedListener,
            initializationCompletedListener,
            errorListener
        )

        // check default values
        assertSame(context, detector.context)
        assertEquals(
            MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED,
            detector.sensorType
        )
        assertEquals(SensorDelay.NORMAL, detector.sensorDelay)
        assertSame(initializationStartedListener, detector.initializationStartedListener)
        assertSame(initializationCompletedListener, detector.initializationCompletedListener)
        assertSame(errorListener, detector.errorListener)
        assertNull(detector.staticIntervalDetectedListener)
        assertNull(detector.dynamicIntervalDetectedListener)
        assertNull(detector.resetListener)
        assertNull(detector.measurementListener)
        assertNull(detector.accuracyChangedListener)
        assertNull(detector.sensor)
        assertEquals(TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE, detector.windowSize)
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES,
            detector.initialStaticSamples
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_THRESHOLD_FACTOR,
            detector.thresholdFactor,
            0.0
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR,
            detector.instantaneousNoiseLevelFactor,
            0.0
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
            detector.baseNoiseLevelAbsoluteThreshold,
            0.0
        )
        val baseNoiseLevelAbsoluteThreshold1 =
            detector.baseNoiseLevelAbsoluteThresholdAsMagneticFluxDensity
        assertEquals(
            detector.baseNoiseLevelAbsoluteThreshold,
            baseNoiseLevelAbsoluteThreshold1.value.toDouble(),
            0.0
        )
        assertEquals(MagneticFluxDensityUnit.TESLA, baseNoiseLevelAbsoluteThreshold1.unit)
        val baseNoiseLevelAbsoluteThreshold2 =
            MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getBaseNoiseLevelAbsoluteThresholdAsMagneticFluxDensity(
            baseNoiseLevelAbsoluteThreshold2
        )
        assertEquals(baseNoiseLevelAbsoluteThreshold1, baseNoiseLevelAbsoluteThreshold2)
        assertNull(detector.baseNoiseLevel)
        assertNull(detector.baseNoiseLevelAsMagneticFluxDensity)
        val baseNoiseLevel = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(detector.getBaseNoiseLevelAsMagneticFluxDensity(baseNoiseLevel))
        assertNull(detector.baseNoiseLevelPsd)
        assertNull(detector.baseNoiseLevelRootPsd)
        assertNull(detector.threshold)
        assertNull(detector.thresholdAsMagneticFluxDensity)
        val threshold = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(detector.getThresholdAsMeasurement(threshold))
        assertEquals(0.0, detector.accumulatedAvgX, 0.0)
        val accumulatedAvgX1 = detector.accumulatedAvgXAsMagneticFluxDensity
        assertEquals(0.0, accumulatedAvgX1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, accumulatedAvgX1.unit)
        val accumulatedAvgX2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getAccumulatedAvgXAsMagneticFluxDensity(accumulatedAvgX2)
        assertEquals(accumulatedAvgX1, accumulatedAvgX2)
        assertEquals(accumulatedAvgX1, accumulatedAvgX2)
        assertEquals(0.0, detector.accumulatedAvgY, 0.0)
        val accumulatedAvgY1 = detector.accumulatedAvgYAsMagneticFluxDensity
        assertEquals(0.0, accumulatedAvgY1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, accumulatedAvgY1.unit)
        val accumulatedAvgY2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getAccumulatedAvgYAsMagneticFluxDensity(accumulatedAvgY2)
        assertEquals(accumulatedAvgY1, accumulatedAvgY2)
        assertEquals(0.0, detector.accumulatedAvgZ, 0.0)
        val accumulatedAvgZ1 = detector.accumulatedAvgZAsMagneticFluxDensity
        assertEquals(0.0, accumulatedAvgZ1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, accumulatedAvgZ1.unit)
        val accumulatedAvgZ2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getAccumulatedAvgZAsMagneticFluxDensity(accumulatedAvgZ2)
        assertEquals(accumulatedAvgZ1, accumulatedAvgZ2)
        val accumulatedAvgTriad1 = detector.accumulatedAvgTriad
        assertEquals(0.0, accumulatedAvgTriad1.valueX, 0.0)
        assertEquals(0.0, accumulatedAvgTriad1.valueY, 0.0)
        assertEquals(0.0, accumulatedAvgTriad1.valueZ, 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, accumulatedAvgTriad1.unit)
        val accumulatedAvgTriad2 = MagneticFluxDensityTriad()
        detector.getAccumulatedAvgTriad(accumulatedAvgTriad2)
        assertEquals(accumulatedAvgTriad1, accumulatedAvgTriad2)
        assertEquals(0.0, detector.accumulatedStdX, 0.0)
        val accumulatedStdX1 = detector.accumulatedStdXAsMagneticFluxDensity
        assertEquals(0.0, accumulatedStdX1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, accumulatedStdX1.unit)
        val accumulatedStdX2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getAccumulatedStdXAsMagneticFluxDensity(accumulatedStdX2)
        assertEquals(accumulatedStdX1, accumulatedStdX2)
        assertEquals(0.0, detector.accumulatedStdY, 0.0)
        val accumulatedStdY1 = detector.accumulatedStdYAsMagneticFluxDensity
        assertEquals(0.0, accumulatedStdY1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, accumulatedStdY1.unit)
        val accumulatedStdY2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getAccumulatedStdYAsMagneticFluxDensity(accumulatedStdY2)
        assertEquals(accumulatedStdY1, accumulatedStdY2)
        assertEquals(0.0, detector.accumulatedStdZ, 0.0)
        val accumulatedStdZ1 = detector.accumulatedStdZAsMagneticFluxDensity
        assertEquals(0.0, accumulatedStdZ1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, accumulatedStdZ1.unit)
        val accumulatedStdZ2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getAccumulatedStdZAsMagneticFluxDensity(accumulatedStdZ2)
        assertEquals(accumulatedStdZ1, accumulatedStdZ2)
        val accumulatedStdTriad1 = detector.accumulatedStdTriad
        assertEquals(0.0, accumulatedStdTriad1.valueX, 0.0)
        assertEquals(0.0, accumulatedStdTriad1.valueY, 0.0)
        assertEquals(0.0, accumulatedStdTriad1.valueZ, 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, accumulatedStdTriad1.unit)
        val accumulatedStdTriad2 = MagneticFluxDensityTriad()
        detector.getAccumulatedStdTriad(accumulatedStdTriad2)
        assertEquals(accumulatedStdTriad1, accumulatedAvgTriad2)
        assertEquals(0.0, detector.instantaneousAvgX, 0.0)
        val instantaneousAvgX1 = detector.instantaneousAvgXAsMagneticFluxDensity
        assertEquals(0.0, instantaneousAvgX1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, instantaneousAvgX1.unit)
        val instantaneousAvgX2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getInstantaneousAvgXAsMagneticFluxDensity(instantaneousAvgX2)
        assertEquals(instantaneousAvgX1, instantaneousAvgX2)
        assertEquals(0.0, detector.instantaneousAvgY, 0.0)
        val instantaneousAvgY1 = detector.instantaneousAvgYAsMagneticFluxDensity
        assertEquals(0.0, instantaneousAvgY1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, instantaneousAvgY1.unit)
        val instantaneousAvgY2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getInstantaneousAvgYAsMagneticFluxDensity(instantaneousAvgY2)
        assertEquals(instantaneousAvgY1, instantaneousAvgY2)
        assertEquals(0.0, detector.instantaneousAvgZ, 0.0)
        val instantaneousAvgZ1 = detector.instantaneousAvgZAsMagneticFluxDensity
        assertEquals(0.0, instantaneousAvgZ1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, instantaneousAvgZ1.unit)
        val instantaneousAvgZ2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getInstantaneousAvgZAsMagneticFluxDensity(instantaneousAvgZ2)
        assertEquals(instantaneousAvgZ1, instantaneousAvgZ2)
        val instantaneousAvgTriad1 = detector.instantaneousAvgTriad
        assertEquals(0.0, instantaneousAvgTriad1.valueX, 0.0)
        assertEquals(0.0, instantaneousAvgTriad1.valueY, 0.0)
        assertEquals(0.0, instantaneousAvgTriad1.valueZ, 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, instantaneousAvgTriad1.unit)
        val instantaneousAvgTriad2 = MagneticFluxDensityTriad()
        detector.getInstantaneousAvgTriad(instantaneousAvgTriad2)
        assertEquals(instantaneousAvgTriad1, instantaneousAvgTriad2)
        assertEquals(0.0, detector.instantaneousStdX, 0.0)
        val instantaneousStdX1 = detector.instantaneousStdXAsMagneticFluxDensity
        assertEquals(0.0, instantaneousStdX1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, instantaneousStdX1.unit)
        val instantaneousStdX2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getInstantaneousStdXAsMagneticFluxDensity(instantaneousStdX2)
        assertEquals(instantaneousStdX1, instantaneousStdX2)
        assertEquals(0.0, detector.instantaneousStdY, 0.0)
        val instantaneousStdY1 = detector.instantaneousStdYAsMagneticFluxDensity
        assertEquals(0.0, instantaneousStdY1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, instantaneousStdY1.unit)
        val instantaneousStdY2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getInstantaneousStdYAsMagneticFluxDensity(instantaneousStdY2)
        assertEquals(instantaneousStdY1, instantaneousStdY2)
        assertEquals(0.0, detector.instantaneousStdZ, 0.0)
        val instantaneousStdZ1 = detector.instantaneousStdZAsMagneticFluxDensity
        assertEquals(0.0, instantaneousStdZ1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, instantaneousStdZ1.unit)
        val instantaneousStdZ2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getInstantaneousStdZAsMagneticFluxDensity(instantaneousStdZ2)
        assertEquals(instantaneousStdZ1, instantaneousStdZ2)
        val instantaneousStdTriad1 = detector.instantaneousStdTriad
        assertEquals(0.0, instantaneousAvgTriad1.valueX, 0.0)
        assertEquals(0.0, instantaneousStdTriad1.valueY, 0.0)
        assertEquals(0.0, instantaneousStdTriad1.valueZ, 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, instantaneousStdTriad1.unit)
        val instantaneousStdTriad2 = MagneticFluxDensityTriad()
        detector.getInstantaneousStdTriad(instantaneousStdTriad2)
        assertEquals(instantaneousStdTriad1, instantaneousStdTriad2)
        assertNull(detector.averageTimeInterval)
        assertNull(detector.averageTimeIntervalAsTime)
        val time = Time(0.0, TimeUnit.SECOND)
        assertFalse(detector.getAverageTimeIntervalAsTime(time))
        assertNull(detector.timeIntervalVariance)
        assertNull(detector.timeIntervalStandardDeviation)
        assertNull(detector.timeIntervalStandardDeviationAsTime)
        assertFalse(detector.getTimeIntervalStandardDeviationAsTime(time))
        assertEquals(0, detector.numberOfProcessedMeasurements)
        assertFalse(detector.running)
        assertEquals(MagnetometerIntervalDetector.Status.IDLE, detector.status)
    }

    @Test
    fun constructor_whenStaticIntervalDetectedListener_setsExpectedValues() {
        val initializationStartedListener =
            mockk<MagnetometerIntervalDetector.OnInitializationStartedListener>()
        val initializationCompletedListener =
            mockk<MagnetometerIntervalDetector.OnInitializationCompletedListener>()
        val errorListener = mockk<MagnetometerIntervalDetector.OnErrorListener>()
        val staticIntervalDetectedListener =
            mockk<MagnetometerIntervalDetector.OnStaticIntervalDetectedListener>()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(
            context,
            MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED,
            SensorDelay.NORMAL,
            initializationStartedListener,
            initializationCompletedListener,
            errorListener,
            staticIntervalDetectedListener
        )

        // check default values
        assertSame(context, detector.context)
        assertEquals(
            MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED,
            detector.sensorType
        )
        assertEquals(SensorDelay.NORMAL, detector.sensorDelay)
        assertSame(initializationStartedListener, detector.initializationStartedListener)
        assertSame(initializationCompletedListener, detector.initializationCompletedListener)
        assertSame(errorListener, detector.errorListener)
        assertSame(staticIntervalDetectedListener, detector.staticIntervalDetectedListener)
        assertNull(detector.dynamicIntervalDetectedListener)
        assertNull(detector.resetListener)
        assertNull(detector.measurementListener)
        assertNull(detector.accuracyChangedListener)
        assertNull(detector.sensor)
        assertEquals(TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE, detector.windowSize)
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES,
            detector.initialStaticSamples
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_THRESHOLD_FACTOR,
            detector.thresholdFactor,
            0.0
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR,
            detector.instantaneousNoiseLevelFactor,
            0.0
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
            detector.baseNoiseLevelAbsoluteThreshold,
            0.0
        )
        val baseNoiseLevelAbsoluteThreshold1 =
            detector.baseNoiseLevelAbsoluteThresholdAsMagneticFluxDensity
        assertEquals(
            detector.baseNoiseLevelAbsoluteThreshold,
            baseNoiseLevelAbsoluteThreshold1.value.toDouble(),
            0.0
        )
        assertEquals(MagneticFluxDensityUnit.TESLA, baseNoiseLevelAbsoluteThreshold1.unit)
        val baseNoiseLevelAbsoluteThreshold2 =
            MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getBaseNoiseLevelAbsoluteThresholdAsMagneticFluxDensity(
            baseNoiseLevelAbsoluteThreshold2
        )
        assertEquals(baseNoiseLevelAbsoluteThreshold1, baseNoiseLevelAbsoluteThreshold2)
        assertNull(detector.baseNoiseLevel)
        assertNull(detector.baseNoiseLevelAsMagneticFluxDensity)
        val baseNoiseLevel = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(detector.getBaseNoiseLevelAsMagneticFluxDensity(baseNoiseLevel))
        assertNull(detector.baseNoiseLevelPsd)
        assertNull(detector.baseNoiseLevelRootPsd)
        assertNull(detector.threshold)
        assertNull(detector.thresholdAsMagneticFluxDensity)
        val threshold = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(detector.getThresholdAsMeasurement(threshold))
        assertEquals(0.0, detector.accumulatedAvgX, 0.0)
        val accumulatedAvgX1 = detector.accumulatedAvgXAsMagneticFluxDensity
        assertEquals(0.0, accumulatedAvgX1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, accumulatedAvgX1.unit)
        val accumulatedAvgX2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getAccumulatedAvgXAsMagneticFluxDensity(accumulatedAvgX2)
        assertEquals(accumulatedAvgX1, accumulatedAvgX2)
        assertEquals(accumulatedAvgX1, accumulatedAvgX2)
        assertEquals(0.0, detector.accumulatedAvgY, 0.0)
        val accumulatedAvgY1 = detector.accumulatedAvgYAsMagneticFluxDensity
        assertEquals(0.0, accumulatedAvgY1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, accumulatedAvgY1.unit)
        val accumulatedAvgY2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getAccumulatedAvgYAsMagneticFluxDensity(accumulatedAvgY2)
        assertEquals(accumulatedAvgY1, accumulatedAvgY2)
        assertEquals(0.0, detector.accumulatedAvgZ, 0.0)
        val accumulatedAvgZ1 = detector.accumulatedAvgZAsMagneticFluxDensity
        assertEquals(0.0, accumulatedAvgZ1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, accumulatedAvgZ1.unit)
        val accumulatedAvgZ2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getAccumulatedAvgZAsMagneticFluxDensity(accumulatedAvgZ2)
        assertEquals(accumulatedAvgZ1, accumulatedAvgZ2)
        val accumulatedAvgTriad1 = detector.accumulatedAvgTriad
        assertEquals(0.0, accumulatedAvgTriad1.valueX, 0.0)
        assertEquals(0.0, accumulatedAvgTriad1.valueY, 0.0)
        assertEquals(0.0, accumulatedAvgTriad1.valueZ, 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, accumulatedAvgTriad1.unit)
        val accumulatedAvgTriad2 = MagneticFluxDensityTriad()
        detector.getAccumulatedAvgTriad(accumulatedAvgTriad2)
        assertEquals(accumulatedAvgTriad1, accumulatedAvgTriad2)
        assertEquals(0.0, detector.accumulatedStdX, 0.0)
        val accumulatedStdX1 = detector.accumulatedStdXAsMagneticFluxDensity
        assertEquals(0.0, accumulatedStdX1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, accumulatedStdX1.unit)
        val accumulatedStdX2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getAccumulatedStdXAsMagneticFluxDensity(accumulatedStdX2)
        assertEquals(accumulatedStdX1, accumulatedStdX2)
        assertEquals(0.0, detector.accumulatedStdY, 0.0)
        val accumulatedStdY1 = detector.accumulatedStdYAsMagneticFluxDensity
        assertEquals(0.0, accumulatedStdY1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, accumulatedStdY1.unit)
        val accumulatedStdY2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getAccumulatedStdYAsMagneticFluxDensity(accumulatedStdY2)
        assertEquals(accumulatedStdY1, accumulatedStdY2)
        assertEquals(0.0, detector.accumulatedStdZ, 0.0)
        val accumulatedStdZ1 = detector.accumulatedStdZAsMagneticFluxDensity
        assertEquals(0.0, accumulatedStdZ1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, accumulatedStdZ1.unit)
        val accumulatedStdZ2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getAccumulatedStdZAsMagneticFluxDensity(accumulatedStdZ2)
        assertEquals(accumulatedStdZ1, accumulatedStdZ2)
        val accumulatedStdTriad1 = detector.accumulatedStdTriad
        assertEquals(0.0, accumulatedStdTriad1.valueX, 0.0)
        assertEquals(0.0, accumulatedStdTriad1.valueY, 0.0)
        assertEquals(0.0, accumulatedStdTriad1.valueZ, 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, accumulatedStdTriad1.unit)
        val accumulatedStdTriad2 = MagneticFluxDensityTriad()
        detector.getAccumulatedStdTriad(accumulatedStdTriad2)
        assertEquals(accumulatedStdTriad1, accumulatedAvgTriad2)
        assertEquals(0.0, detector.instantaneousAvgX, 0.0)
        val instantaneousAvgX1 = detector.instantaneousAvgXAsMagneticFluxDensity
        assertEquals(0.0, instantaneousAvgX1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, instantaneousAvgX1.unit)
        val instantaneousAvgX2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getInstantaneousAvgXAsMagneticFluxDensity(instantaneousAvgX2)
        assertEquals(instantaneousAvgX1, instantaneousAvgX2)
        assertEquals(0.0, detector.instantaneousAvgY, 0.0)
        val instantaneousAvgY1 = detector.instantaneousAvgYAsMagneticFluxDensity
        assertEquals(0.0, instantaneousAvgY1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, instantaneousAvgY1.unit)
        val instantaneousAvgY2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getInstantaneousAvgYAsMagneticFluxDensity(instantaneousAvgY2)
        assertEquals(instantaneousAvgY1, instantaneousAvgY2)
        assertEquals(0.0, detector.instantaneousAvgZ, 0.0)
        val instantaneousAvgZ1 = detector.instantaneousAvgZAsMagneticFluxDensity
        assertEquals(0.0, instantaneousAvgZ1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, instantaneousAvgZ1.unit)
        val instantaneousAvgZ2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getInstantaneousAvgZAsMagneticFluxDensity(instantaneousAvgZ2)
        assertEquals(instantaneousAvgZ1, instantaneousAvgZ2)
        val instantaneousAvgTriad1 = detector.instantaneousAvgTriad
        assertEquals(0.0, instantaneousAvgTriad1.valueX, 0.0)
        assertEquals(0.0, instantaneousAvgTriad1.valueY, 0.0)
        assertEquals(0.0, instantaneousAvgTriad1.valueZ, 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, instantaneousAvgTriad1.unit)
        val instantaneousAvgTriad2 = MagneticFluxDensityTriad()
        detector.getInstantaneousAvgTriad(instantaneousAvgTriad2)
        assertEquals(instantaneousAvgTriad1, instantaneousAvgTriad2)
        assertEquals(0.0, detector.instantaneousStdX, 0.0)
        val instantaneousStdX1 = detector.instantaneousStdXAsMagneticFluxDensity
        assertEquals(0.0, instantaneousStdX1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, instantaneousStdX1.unit)
        val instantaneousStdX2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getInstantaneousStdXAsMagneticFluxDensity(instantaneousStdX2)
        assertEquals(instantaneousStdX1, instantaneousStdX2)
        assertEquals(0.0, detector.instantaneousStdY, 0.0)
        val instantaneousStdY1 = detector.instantaneousStdYAsMagneticFluxDensity
        assertEquals(0.0, instantaneousStdY1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, instantaneousStdY1.unit)
        val instantaneousStdY2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getInstantaneousStdYAsMagneticFluxDensity(instantaneousStdY2)
        assertEquals(instantaneousStdY1, instantaneousStdY2)
        assertEquals(0.0, detector.instantaneousStdZ, 0.0)
        val instantaneousStdZ1 = detector.instantaneousStdZAsMagneticFluxDensity
        assertEquals(0.0, instantaneousStdZ1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, instantaneousStdZ1.unit)
        val instantaneousStdZ2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getInstantaneousStdZAsMagneticFluxDensity(instantaneousStdZ2)
        assertEquals(instantaneousStdZ1, instantaneousStdZ2)
        val instantaneousStdTriad1 = detector.instantaneousStdTriad
        assertEquals(0.0, instantaneousAvgTriad1.valueX, 0.0)
        assertEquals(0.0, instantaneousStdTriad1.valueY, 0.0)
        assertEquals(0.0, instantaneousStdTriad1.valueZ, 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, instantaneousStdTriad1.unit)
        val instantaneousStdTriad2 = MagneticFluxDensityTriad()
        detector.getInstantaneousStdTriad(instantaneousStdTriad2)
        assertEquals(instantaneousStdTriad1, instantaneousStdTriad2)
        assertNull(detector.averageTimeInterval)
        assertNull(detector.averageTimeIntervalAsTime)
        val time = Time(0.0, TimeUnit.SECOND)
        assertFalse(detector.getAverageTimeIntervalAsTime(time))
        assertNull(detector.timeIntervalVariance)
        assertNull(detector.timeIntervalStandardDeviation)
        assertNull(detector.timeIntervalStandardDeviationAsTime)
        assertFalse(detector.getTimeIntervalStandardDeviationAsTime(time))
        assertEquals(0, detector.numberOfProcessedMeasurements)
        assertFalse(detector.running)
        assertEquals(MagnetometerIntervalDetector.Status.IDLE, detector.status)
    }

    @Test
    fun constructor_whenDynamicIntervalDetectedListener_setsExpectedValues() {
        val initializationStartedListener =
            mockk<MagnetometerIntervalDetector.OnInitializationStartedListener>()
        val initializationCompletedListener =
            mockk<MagnetometerIntervalDetector.OnInitializationCompletedListener>()
        val errorListener = mockk<MagnetometerIntervalDetector.OnErrorListener>()
        val staticIntervalDetectedListener =
            mockk<MagnetometerIntervalDetector.OnStaticIntervalDetectedListener>()
        val dynamicIntervalDetectedListener =
            mockk<MagnetometerIntervalDetector.OnDynamicIntervalDetectedListener>()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(
            context,
            MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED,
            SensorDelay.NORMAL,
            initializationStartedListener,
            initializationCompletedListener,
            errorListener,
            staticIntervalDetectedListener,
            dynamicIntervalDetectedListener
        )

        // check default values
        assertSame(context, detector.context)
        assertEquals(
            MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED,
            detector.sensorType
        )
        assertEquals(SensorDelay.NORMAL, detector.sensorDelay)
        assertSame(initializationStartedListener, detector.initializationStartedListener)
        assertSame(initializationCompletedListener, detector.initializationCompletedListener)
        assertSame(errorListener, detector.errorListener)
        assertSame(staticIntervalDetectedListener, detector.staticIntervalDetectedListener)
        assertSame(dynamicIntervalDetectedListener, detector.dynamicIntervalDetectedListener)
        assertNull(detector.resetListener)
        assertNull(detector.measurementListener)
        assertNull(detector.accuracyChangedListener)
        assertNull(detector.sensor)
        assertEquals(TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE, detector.windowSize)
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES,
            detector.initialStaticSamples
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_THRESHOLD_FACTOR,
            detector.thresholdFactor,
            0.0
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR,
            detector.instantaneousNoiseLevelFactor,
            0.0
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
            detector.baseNoiseLevelAbsoluteThreshold,
            0.0
        )
        val baseNoiseLevelAbsoluteThreshold1 =
            detector.baseNoiseLevelAbsoluteThresholdAsMagneticFluxDensity
        assertEquals(
            detector.baseNoiseLevelAbsoluteThreshold,
            baseNoiseLevelAbsoluteThreshold1.value.toDouble(),
            0.0
        )
        assertEquals(MagneticFluxDensityUnit.TESLA, baseNoiseLevelAbsoluteThreshold1.unit)
        val baseNoiseLevelAbsoluteThreshold2 =
            MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getBaseNoiseLevelAbsoluteThresholdAsMagneticFluxDensity(
            baseNoiseLevelAbsoluteThreshold2
        )
        assertEquals(baseNoiseLevelAbsoluteThreshold1, baseNoiseLevelAbsoluteThreshold2)
        assertNull(detector.baseNoiseLevel)
        assertNull(detector.baseNoiseLevelAsMagneticFluxDensity)
        val baseNoiseLevel = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(detector.getBaseNoiseLevelAsMagneticFluxDensity(baseNoiseLevel))
        assertNull(detector.baseNoiseLevelPsd)
        assertNull(detector.baseNoiseLevelRootPsd)
        assertNull(detector.threshold)
        assertNull(detector.thresholdAsMagneticFluxDensity)
        val threshold = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(detector.getThresholdAsMeasurement(threshold))
        assertEquals(0.0, detector.accumulatedAvgX, 0.0)
        val accumulatedAvgX1 = detector.accumulatedAvgXAsMagneticFluxDensity
        assertEquals(0.0, accumulatedAvgX1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, accumulatedAvgX1.unit)
        val accumulatedAvgX2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getAccumulatedAvgXAsMagneticFluxDensity(accumulatedAvgX2)
        assertEquals(accumulatedAvgX1, accumulatedAvgX2)
        assertEquals(accumulatedAvgX1, accumulatedAvgX2)
        assertEquals(0.0, detector.accumulatedAvgY, 0.0)
        val accumulatedAvgY1 = detector.accumulatedAvgYAsMagneticFluxDensity
        assertEquals(0.0, accumulatedAvgY1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, accumulatedAvgY1.unit)
        val accumulatedAvgY2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getAccumulatedAvgYAsMagneticFluxDensity(accumulatedAvgY2)
        assertEquals(accumulatedAvgY1, accumulatedAvgY2)
        assertEquals(0.0, detector.accumulatedAvgZ, 0.0)
        val accumulatedAvgZ1 = detector.accumulatedAvgZAsMagneticFluxDensity
        assertEquals(0.0, accumulatedAvgZ1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, accumulatedAvgZ1.unit)
        val accumulatedAvgZ2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getAccumulatedAvgZAsMagneticFluxDensity(accumulatedAvgZ2)
        assertEquals(accumulatedAvgZ1, accumulatedAvgZ2)
        val accumulatedAvgTriad1 = detector.accumulatedAvgTriad
        assertEquals(0.0, accumulatedAvgTriad1.valueX, 0.0)
        assertEquals(0.0, accumulatedAvgTriad1.valueY, 0.0)
        assertEquals(0.0, accumulatedAvgTriad1.valueZ, 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, accumulatedAvgTriad1.unit)
        val accumulatedAvgTriad2 = MagneticFluxDensityTriad()
        detector.getAccumulatedAvgTriad(accumulatedAvgTriad2)
        assertEquals(accumulatedAvgTriad1, accumulatedAvgTriad2)
        assertEquals(0.0, detector.accumulatedStdX, 0.0)
        val accumulatedStdX1 = detector.accumulatedStdXAsMagneticFluxDensity
        assertEquals(0.0, accumulatedStdX1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, accumulatedStdX1.unit)
        val accumulatedStdX2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getAccumulatedStdXAsMagneticFluxDensity(accumulatedStdX2)
        assertEquals(accumulatedStdX1, accumulatedStdX2)
        assertEquals(0.0, detector.accumulatedStdY, 0.0)
        val accumulatedStdY1 = detector.accumulatedStdYAsMagneticFluxDensity
        assertEquals(0.0, accumulatedStdY1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, accumulatedStdY1.unit)
        val accumulatedStdY2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getAccumulatedStdYAsMagneticFluxDensity(accumulatedStdY2)
        assertEquals(accumulatedStdY1, accumulatedStdY2)
        assertEquals(0.0, detector.accumulatedStdZ, 0.0)
        val accumulatedStdZ1 = detector.accumulatedStdZAsMagneticFluxDensity
        assertEquals(0.0, accumulatedStdZ1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, accumulatedStdZ1.unit)
        val accumulatedStdZ2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getAccumulatedStdZAsMagneticFluxDensity(accumulatedStdZ2)
        assertEquals(accumulatedStdZ1, accumulatedStdZ2)
        val accumulatedStdTriad1 = detector.accumulatedStdTriad
        assertEquals(0.0, accumulatedStdTriad1.valueX, 0.0)
        assertEquals(0.0, accumulatedStdTriad1.valueY, 0.0)
        assertEquals(0.0, accumulatedStdTriad1.valueZ, 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, accumulatedStdTriad1.unit)
        val accumulatedStdTriad2 = MagneticFluxDensityTriad()
        detector.getAccumulatedStdTriad(accumulatedStdTriad2)
        assertEquals(accumulatedStdTriad1, accumulatedAvgTriad2)
        assertEquals(0.0, detector.instantaneousAvgX, 0.0)
        val instantaneousAvgX1 = detector.instantaneousAvgXAsMagneticFluxDensity
        assertEquals(0.0, instantaneousAvgX1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, instantaneousAvgX1.unit)
        val instantaneousAvgX2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getInstantaneousAvgXAsMagneticFluxDensity(instantaneousAvgX2)
        assertEquals(instantaneousAvgX1, instantaneousAvgX2)
        assertEquals(0.0, detector.instantaneousAvgY, 0.0)
        val instantaneousAvgY1 = detector.instantaneousAvgYAsMagneticFluxDensity
        assertEquals(0.0, instantaneousAvgY1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, instantaneousAvgY1.unit)
        val instantaneousAvgY2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getInstantaneousAvgYAsMagneticFluxDensity(instantaneousAvgY2)
        assertEquals(instantaneousAvgY1, instantaneousAvgY2)
        assertEquals(0.0, detector.instantaneousAvgZ, 0.0)
        val instantaneousAvgZ1 = detector.instantaneousAvgZAsMagneticFluxDensity
        assertEquals(0.0, instantaneousAvgZ1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, instantaneousAvgZ1.unit)
        val instantaneousAvgZ2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getInstantaneousAvgZAsMagneticFluxDensity(instantaneousAvgZ2)
        assertEquals(instantaneousAvgZ1, instantaneousAvgZ2)
        val instantaneousAvgTriad1 = detector.instantaneousAvgTriad
        assertEquals(0.0, instantaneousAvgTriad1.valueX, 0.0)
        assertEquals(0.0, instantaneousAvgTriad1.valueY, 0.0)
        assertEquals(0.0, instantaneousAvgTriad1.valueZ, 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, instantaneousAvgTriad1.unit)
        val instantaneousAvgTriad2 = MagneticFluxDensityTriad()
        detector.getInstantaneousAvgTriad(instantaneousAvgTriad2)
        assertEquals(instantaneousAvgTriad1, instantaneousAvgTriad2)
        assertEquals(0.0, detector.instantaneousStdX, 0.0)
        val instantaneousStdX1 = detector.instantaneousStdXAsMagneticFluxDensity
        assertEquals(0.0, instantaneousStdX1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, instantaneousStdX1.unit)
        val instantaneousStdX2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getInstantaneousStdXAsMagneticFluxDensity(instantaneousStdX2)
        assertEquals(instantaneousStdX1, instantaneousStdX2)
        assertEquals(0.0, detector.instantaneousStdY, 0.0)
        val instantaneousStdY1 = detector.instantaneousStdYAsMagneticFluxDensity
        assertEquals(0.0, instantaneousStdY1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, instantaneousStdY1.unit)
        val instantaneousStdY2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getInstantaneousStdYAsMagneticFluxDensity(instantaneousStdY2)
        assertEquals(instantaneousStdY1, instantaneousStdY2)
        assertEquals(0.0, detector.instantaneousStdZ, 0.0)
        val instantaneousStdZ1 = detector.instantaneousStdZAsMagneticFluxDensity
        assertEquals(0.0, instantaneousStdZ1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, instantaneousStdZ1.unit)
        val instantaneousStdZ2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getInstantaneousStdZAsMagneticFluxDensity(instantaneousStdZ2)
        assertEquals(instantaneousStdZ1, instantaneousStdZ2)
        val instantaneousStdTriad1 = detector.instantaneousStdTriad
        assertEquals(0.0, instantaneousAvgTriad1.valueX, 0.0)
        assertEquals(0.0, instantaneousStdTriad1.valueY, 0.0)
        assertEquals(0.0, instantaneousStdTriad1.valueZ, 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, instantaneousStdTriad1.unit)
        val instantaneousStdTriad2 = MagneticFluxDensityTriad()
        detector.getInstantaneousStdTriad(instantaneousStdTriad2)
        assertEquals(instantaneousStdTriad1, instantaneousStdTriad2)
        assertNull(detector.averageTimeInterval)
        assertNull(detector.averageTimeIntervalAsTime)
        val time = Time(0.0, TimeUnit.SECOND)
        assertFalse(detector.getAverageTimeIntervalAsTime(time))
        assertNull(detector.timeIntervalVariance)
        assertNull(detector.timeIntervalStandardDeviation)
        assertNull(detector.timeIntervalStandardDeviationAsTime)
        assertFalse(detector.getTimeIntervalStandardDeviationAsTime(time))
        assertEquals(0, detector.numberOfProcessedMeasurements)
        assertFalse(detector.running)
        assertEquals(MagnetometerIntervalDetector.Status.IDLE, detector.status)
    }

    @Test
    fun constructor_whenResetListener_setsExpectedValues() {
        val initializationStartedListener =
            mockk<MagnetometerIntervalDetector.OnInitializationStartedListener>()
        val initializationCompletedListener =
            mockk<MagnetometerIntervalDetector.OnInitializationCompletedListener>()
        val errorListener = mockk<MagnetometerIntervalDetector.OnErrorListener>()
        val staticIntervalDetectedListener =
            mockk<MagnetometerIntervalDetector.OnStaticIntervalDetectedListener>()
        val dynamicIntervalDetectedListener =
            mockk<MagnetometerIntervalDetector.OnDynamicIntervalDetectedListener>()
        val resetListener = mockk<MagnetometerIntervalDetector.OnResetListener>()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(
            context,
            MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED,
            SensorDelay.NORMAL,
            initializationStartedListener,
            initializationCompletedListener,
            errorListener,
            staticIntervalDetectedListener,
            dynamicIntervalDetectedListener,
            resetListener
        )

        // check default values
        assertSame(context, detector.context)
        assertEquals(
            MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED,
            detector.sensorType
        )
        assertEquals(SensorDelay.NORMAL, detector.sensorDelay)
        assertSame(initializationStartedListener, detector.initializationStartedListener)
        assertSame(initializationCompletedListener, detector.initializationCompletedListener)
        assertSame(errorListener, detector.errorListener)
        assertSame(staticIntervalDetectedListener, detector.staticIntervalDetectedListener)
        assertSame(dynamicIntervalDetectedListener, detector.dynamicIntervalDetectedListener)
        assertSame(resetListener, detector.resetListener)
        assertNull(detector.measurementListener)
        assertNull(detector.accuracyChangedListener)
        assertNull(detector.sensor)
        assertEquals(TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE, detector.windowSize)
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES,
            detector.initialStaticSamples
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_THRESHOLD_FACTOR,
            detector.thresholdFactor,
            0.0
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR,
            detector.instantaneousNoiseLevelFactor,
            0.0
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
            detector.baseNoiseLevelAbsoluteThreshold,
            0.0
        )
        val baseNoiseLevelAbsoluteThreshold1 =
            detector.baseNoiseLevelAbsoluteThresholdAsMagneticFluxDensity
        assertEquals(
            detector.baseNoiseLevelAbsoluteThreshold,
            baseNoiseLevelAbsoluteThreshold1.value.toDouble(),
            0.0
        )
        assertEquals(MagneticFluxDensityUnit.TESLA, baseNoiseLevelAbsoluteThreshold1.unit)
        val baseNoiseLevelAbsoluteThreshold2 =
            MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getBaseNoiseLevelAbsoluteThresholdAsMagneticFluxDensity(
            baseNoiseLevelAbsoluteThreshold2
        )
        assertEquals(baseNoiseLevelAbsoluteThreshold1, baseNoiseLevelAbsoluteThreshold2)
        assertNull(detector.baseNoiseLevel)
        assertNull(detector.baseNoiseLevelAsMagneticFluxDensity)
        val baseNoiseLevel = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(detector.getBaseNoiseLevelAsMagneticFluxDensity(baseNoiseLevel))
        assertNull(detector.baseNoiseLevelPsd)
        assertNull(detector.baseNoiseLevelRootPsd)
        assertNull(detector.threshold)
        assertNull(detector.thresholdAsMagneticFluxDensity)
        val threshold = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(detector.getThresholdAsMeasurement(threshold))
        assertEquals(0.0, detector.accumulatedAvgX, 0.0)
        val accumulatedAvgX1 = detector.accumulatedAvgXAsMagneticFluxDensity
        assertEquals(0.0, accumulatedAvgX1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, accumulatedAvgX1.unit)
        val accumulatedAvgX2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getAccumulatedAvgXAsMagneticFluxDensity(accumulatedAvgX2)
        assertEquals(accumulatedAvgX1, accumulatedAvgX2)
        assertEquals(accumulatedAvgX1, accumulatedAvgX2)
        assertEquals(0.0, detector.accumulatedAvgY, 0.0)
        val accumulatedAvgY1 = detector.accumulatedAvgYAsMagneticFluxDensity
        assertEquals(0.0, accumulatedAvgY1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, accumulatedAvgY1.unit)
        val accumulatedAvgY2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getAccumulatedAvgYAsMagneticFluxDensity(accumulatedAvgY2)
        assertEquals(accumulatedAvgY1, accumulatedAvgY2)
        assertEquals(0.0, detector.accumulatedAvgZ, 0.0)
        val accumulatedAvgZ1 = detector.accumulatedAvgZAsMagneticFluxDensity
        assertEquals(0.0, accumulatedAvgZ1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, accumulatedAvgZ1.unit)
        val accumulatedAvgZ2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getAccumulatedAvgZAsMagneticFluxDensity(accumulatedAvgZ2)
        assertEquals(accumulatedAvgZ1, accumulatedAvgZ2)
        val accumulatedAvgTriad1 = detector.accumulatedAvgTriad
        assertEquals(0.0, accumulatedAvgTriad1.valueX, 0.0)
        assertEquals(0.0, accumulatedAvgTriad1.valueY, 0.0)
        assertEquals(0.0, accumulatedAvgTriad1.valueZ, 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, accumulatedAvgTriad1.unit)
        val accumulatedAvgTriad2 = MagneticFluxDensityTriad()
        detector.getAccumulatedAvgTriad(accumulatedAvgTriad2)
        assertEquals(accumulatedAvgTriad1, accumulatedAvgTriad2)
        assertEquals(0.0, detector.accumulatedStdX, 0.0)
        val accumulatedStdX1 = detector.accumulatedStdXAsMagneticFluxDensity
        assertEquals(0.0, accumulatedStdX1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, accumulatedStdX1.unit)
        val accumulatedStdX2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getAccumulatedStdXAsMagneticFluxDensity(accumulatedStdX2)
        assertEquals(accumulatedStdX1, accumulatedStdX2)
        assertEquals(0.0, detector.accumulatedStdY, 0.0)
        val accumulatedStdY1 = detector.accumulatedStdYAsMagneticFluxDensity
        assertEquals(0.0, accumulatedStdY1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, accumulatedStdY1.unit)
        val accumulatedStdY2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getAccumulatedStdYAsMagneticFluxDensity(accumulatedStdY2)
        assertEquals(accumulatedStdY1, accumulatedStdY2)
        assertEquals(0.0, detector.accumulatedStdZ, 0.0)
        val accumulatedStdZ1 = detector.accumulatedStdZAsMagneticFluxDensity
        assertEquals(0.0, accumulatedStdZ1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, accumulatedStdZ1.unit)
        val accumulatedStdZ2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getAccumulatedStdZAsMagneticFluxDensity(accumulatedStdZ2)
        assertEquals(accumulatedStdZ1, accumulatedStdZ2)
        val accumulatedStdTriad1 = detector.accumulatedStdTriad
        assertEquals(0.0, accumulatedStdTriad1.valueX, 0.0)
        assertEquals(0.0, accumulatedStdTriad1.valueY, 0.0)
        assertEquals(0.0, accumulatedStdTriad1.valueZ, 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, accumulatedStdTriad1.unit)
        val accumulatedStdTriad2 = MagneticFluxDensityTriad()
        detector.getAccumulatedStdTriad(accumulatedStdTriad2)
        assertEquals(accumulatedStdTriad1, accumulatedAvgTriad2)
        assertEquals(0.0, detector.instantaneousAvgX, 0.0)
        val instantaneousAvgX1 = detector.instantaneousAvgXAsMagneticFluxDensity
        assertEquals(0.0, instantaneousAvgX1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, instantaneousAvgX1.unit)
        val instantaneousAvgX2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getInstantaneousAvgXAsMagneticFluxDensity(instantaneousAvgX2)
        assertEquals(instantaneousAvgX1, instantaneousAvgX2)
        assertEquals(0.0, detector.instantaneousAvgY, 0.0)
        val instantaneousAvgY1 = detector.instantaneousAvgYAsMagneticFluxDensity
        assertEquals(0.0, instantaneousAvgY1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, instantaneousAvgY1.unit)
        val instantaneousAvgY2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getInstantaneousAvgYAsMagneticFluxDensity(instantaneousAvgY2)
        assertEquals(instantaneousAvgY1, instantaneousAvgY2)
        assertEquals(0.0, detector.instantaneousAvgZ, 0.0)
        val instantaneousAvgZ1 = detector.instantaneousAvgZAsMagneticFluxDensity
        assertEquals(0.0, instantaneousAvgZ1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, instantaneousAvgZ1.unit)
        val instantaneousAvgZ2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getInstantaneousAvgZAsMagneticFluxDensity(instantaneousAvgZ2)
        assertEquals(instantaneousAvgZ1, instantaneousAvgZ2)
        val instantaneousAvgTriad1 = detector.instantaneousAvgTriad
        assertEquals(0.0, instantaneousAvgTriad1.valueX, 0.0)
        assertEquals(0.0, instantaneousAvgTriad1.valueY, 0.0)
        assertEquals(0.0, instantaneousAvgTriad1.valueZ, 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, instantaneousAvgTriad1.unit)
        val instantaneousAvgTriad2 = MagneticFluxDensityTriad()
        detector.getInstantaneousAvgTriad(instantaneousAvgTriad2)
        assertEquals(instantaneousAvgTriad1, instantaneousAvgTriad2)
        assertEquals(0.0, detector.instantaneousStdX, 0.0)
        val instantaneousStdX1 = detector.instantaneousStdXAsMagneticFluxDensity
        assertEquals(0.0, instantaneousStdX1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, instantaneousStdX1.unit)
        val instantaneousStdX2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getInstantaneousStdXAsMagneticFluxDensity(instantaneousStdX2)
        assertEquals(instantaneousStdX1, instantaneousStdX2)
        assertEquals(0.0, detector.instantaneousStdY, 0.0)
        val instantaneousStdY1 = detector.instantaneousStdYAsMagneticFluxDensity
        assertEquals(0.0, instantaneousStdY1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, instantaneousStdY1.unit)
        val instantaneousStdY2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getInstantaneousStdYAsMagneticFluxDensity(instantaneousStdY2)
        assertEquals(instantaneousStdY1, instantaneousStdY2)
        assertEquals(0.0, detector.instantaneousStdZ, 0.0)
        val instantaneousStdZ1 = detector.instantaneousStdZAsMagneticFluxDensity
        assertEquals(0.0, instantaneousStdZ1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, instantaneousStdZ1.unit)
        val instantaneousStdZ2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getInstantaneousStdZAsMagneticFluxDensity(instantaneousStdZ2)
        assertEquals(instantaneousStdZ1, instantaneousStdZ2)
        val instantaneousStdTriad1 = detector.instantaneousStdTriad
        assertEquals(0.0, instantaneousAvgTriad1.valueX, 0.0)
        assertEquals(0.0, instantaneousStdTriad1.valueY, 0.0)
        assertEquals(0.0, instantaneousStdTriad1.valueZ, 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, instantaneousStdTriad1.unit)
        val instantaneousStdTriad2 = MagneticFluxDensityTriad()
        detector.getInstantaneousStdTriad(instantaneousStdTriad2)
        assertEquals(instantaneousStdTriad1, instantaneousStdTriad2)
        assertNull(detector.averageTimeInterval)
        assertNull(detector.averageTimeIntervalAsTime)
        val time = Time(0.0, TimeUnit.SECOND)
        assertFalse(detector.getAverageTimeIntervalAsTime(time))
        assertNull(detector.timeIntervalVariance)
        assertNull(detector.timeIntervalStandardDeviation)
        assertNull(detector.timeIntervalStandardDeviationAsTime)
        assertFalse(detector.getTimeIntervalStandardDeviationAsTime(time))
        assertEquals(0, detector.numberOfProcessedMeasurements)
        assertFalse(detector.running)
        assertEquals(MagnetometerIntervalDetector.Status.IDLE, detector.status)
    }

    @Test
    fun constructor_whenMeasurementListener_setsExpectedValues() {
        val initializationStartedListener =
            mockk<MagnetometerIntervalDetector.OnInitializationStartedListener>()
        val initializationCompletedListener =
            mockk<MagnetometerIntervalDetector.OnInitializationCompletedListener>()
        val errorListener = mockk<MagnetometerIntervalDetector.OnErrorListener>()
        val staticIntervalDetectedListener =
            mockk<MagnetometerIntervalDetector.OnStaticIntervalDetectedListener>()
        val dynamicIntervalDetectedListener =
            mockk<MagnetometerIntervalDetector.OnDynamicIntervalDetectedListener>()
        val resetListener = mockk<MagnetometerIntervalDetector.OnResetListener>()
        val measurementListener = mockk<MagnetometerSensorCollector.OnMeasurementListener>()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(
            context,
            MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED,
            SensorDelay.NORMAL,
            initializationStartedListener,
            initializationCompletedListener,
            errorListener,
            staticIntervalDetectedListener,
            dynamicIntervalDetectedListener,
            resetListener,
            measurementListener
        )

        // check default values
        assertSame(context, detector.context)
        assertEquals(
            MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED,
            detector.sensorType
        )
        assertEquals(SensorDelay.NORMAL, detector.sensorDelay)
        assertSame(initializationStartedListener, detector.initializationStartedListener)
        assertSame(initializationCompletedListener, detector.initializationCompletedListener)
        assertSame(errorListener, detector.errorListener)
        assertSame(staticIntervalDetectedListener, detector.staticIntervalDetectedListener)
        assertSame(dynamicIntervalDetectedListener, detector.dynamicIntervalDetectedListener)
        assertSame(resetListener, detector.resetListener)
        assertSame(measurementListener, detector.measurementListener)
        assertNull(detector.accuracyChangedListener)
        assertNull(detector.sensor)
        assertEquals(TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE, detector.windowSize)
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES,
            detector.initialStaticSamples
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_THRESHOLD_FACTOR,
            detector.thresholdFactor,
            0.0
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR,
            detector.instantaneousNoiseLevelFactor,
            0.0
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
            detector.baseNoiseLevelAbsoluteThreshold,
            0.0
        )
        val baseNoiseLevelAbsoluteThreshold1 =
            detector.baseNoiseLevelAbsoluteThresholdAsMagneticFluxDensity
        assertEquals(
            detector.baseNoiseLevelAbsoluteThreshold,
            baseNoiseLevelAbsoluteThreshold1.value.toDouble(),
            0.0
        )
        assertEquals(MagneticFluxDensityUnit.TESLA, baseNoiseLevelAbsoluteThreshold1.unit)
        val baseNoiseLevelAbsoluteThreshold2 =
            MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getBaseNoiseLevelAbsoluteThresholdAsMagneticFluxDensity(
            baseNoiseLevelAbsoluteThreshold2
        )
        assertEquals(baseNoiseLevelAbsoluteThreshold1, baseNoiseLevelAbsoluteThreshold2)
        assertNull(detector.baseNoiseLevel)
        assertNull(detector.baseNoiseLevelAsMagneticFluxDensity)
        val baseNoiseLevel = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(detector.getBaseNoiseLevelAsMagneticFluxDensity(baseNoiseLevel))
        assertNull(detector.baseNoiseLevelPsd)
        assertNull(detector.baseNoiseLevelRootPsd)
        assertNull(detector.threshold)
        assertNull(detector.thresholdAsMagneticFluxDensity)
        val threshold = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(detector.getThresholdAsMeasurement(threshold))
        assertEquals(0.0, detector.accumulatedAvgX, 0.0)
        val accumulatedAvgX1 = detector.accumulatedAvgXAsMagneticFluxDensity
        assertEquals(0.0, accumulatedAvgX1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, accumulatedAvgX1.unit)
        val accumulatedAvgX2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getAccumulatedAvgXAsMagneticFluxDensity(accumulatedAvgX2)
        assertEquals(accumulatedAvgX1, accumulatedAvgX2)
        assertEquals(accumulatedAvgX1, accumulatedAvgX2)
        assertEquals(0.0, detector.accumulatedAvgY, 0.0)
        val accumulatedAvgY1 = detector.accumulatedAvgYAsMagneticFluxDensity
        assertEquals(0.0, accumulatedAvgY1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, accumulatedAvgY1.unit)
        val accumulatedAvgY2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getAccumulatedAvgYAsMagneticFluxDensity(accumulatedAvgY2)
        assertEquals(accumulatedAvgY1, accumulatedAvgY2)
        assertEquals(0.0, detector.accumulatedAvgZ, 0.0)
        val accumulatedAvgZ1 = detector.accumulatedAvgZAsMagneticFluxDensity
        assertEquals(0.0, accumulatedAvgZ1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, accumulatedAvgZ1.unit)
        val accumulatedAvgZ2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getAccumulatedAvgZAsMagneticFluxDensity(accumulatedAvgZ2)
        assertEquals(accumulatedAvgZ1, accumulatedAvgZ2)
        val accumulatedAvgTriad1 = detector.accumulatedAvgTriad
        assertEquals(0.0, accumulatedAvgTriad1.valueX, 0.0)
        assertEquals(0.0, accumulatedAvgTriad1.valueY, 0.0)
        assertEquals(0.0, accumulatedAvgTriad1.valueZ, 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, accumulatedAvgTriad1.unit)
        val accumulatedAvgTriad2 = MagneticFluxDensityTriad()
        detector.getAccumulatedAvgTriad(accumulatedAvgTriad2)
        assertEquals(accumulatedAvgTriad1, accumulatedAvgTriad2)
        assertEquals(0.0, detector.accumulatedStdX, 0.0)
        val accumulatedStdX1 = detector.accumulatedStdXAsMagneticFluxDensity
        assertEquals(0.0, accumulatedStdX1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, accumulatedStdX1.unit)
        val accumulatedStdX2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getAccumulatedStdXAsMagneticFluxDensity(accumulatedStdX2)
        assertEquals(accumulatedStdX1, accumulatedStdX2)
        assertEquals(0.0, detector.accumulatedStdY, 0.0)
        val accumulatedStdY1 = detector.accumulatedStdYAsMagneticFluxDensity
        assertEquals(0.0, accumulatedStdY1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, accumulatedStdY1.unit)
        val accumulatedStdY2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getAccumulatedStdYAsMagneticFluxDensity(accumulatedStdY2)
        assertEquals(accumulatedStdY1, accumulatedStdY2)
        assertEquals(0.0, detector.accumulatedStdZ, 0.0)
        val accumulatedStdZ1 = detector.accumulatedStdZAsMagneticFluxDensity
        assertEquals(0.0, accumulatedStdZ1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, accumulatedStdZ1.unit)
        val accumulatedStdZ2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getAccumulatedStdZAsMagneticFluxDensity(accumulatedStdZ2)
        assertEquals(accumulatedStdZ1, accumulatedStdZ2)
        val accumulatedStdTriad1 = detector.accumulatedStdTriad
        assertEquals(0.0, accumulatedStdTriad1.valueX, 0.0)
        assertEquals(0.0, accumulatedStdTriad1.valueY, 0.0)
        assertEquals(0.0, accumulatedStdTriad1.valueZ, 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, accumulatedStdTriad1.unit)
        val accumulatedStdTriad2 = MagneticFluxDensityTriad()
        detector.getAccumulatedStdTriad(accumulatedStdTriad2)
        assertEquals(accumulatedStdTriad1, accumulatedAvgTriad2)
        assertEquals(0.0, detector.instantaneousAvgX, 0.0)
        val instantaneousAvgX1 = detector.instantaneousAvgXAsMagneticFluxDensity
        assertEquals(0.0, instantaneousAvgX1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, instantaneousAvgX1.unit)
        val instantaneousAvgX2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getInstantaneousAvgXAsMagneticFluxDensity(instantaneousAvgX2)
        assertEquals(instantaneousAvgX1, instantaneousAvgX2)
        assertEquals(0.0, detector.instantaneousAvgY, 0.0)
        val instantaneousAvgY1 = detector.instantaneousAvgYAsMagneticFluxDensity
        assertEquals(0.0, instantaneousAvgY1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, instantaneousAvgY1.unit)
        val instantaneousAvgY2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getInstantaneousAvgYAsMagneticFluxDensity(instantaneousAvgY2)
        assertEquals(instantaneousAvgY1, instantaneousAvgY2)
        assertEquals(0.0, detector.instantaneousAvgZ, 0.0)
        val instantaneousAvgZ1 = detector.instantaneousAvgZAsMagneticFluxDensity
        assertEquals(0.0, instantaneousAvgZ1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, instantaneousAvgZ1.unit)
        val instantaneousAvgZ2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getInstantaneousAvgZAsMagneticFluxDensity(instantaneousAvgZ2)
        assertEquals(instantaneousAvgZ1, instantaneousAvgZ2)
        val instantaneousAvgTriad1 = detector.instantaneousAvgTriad
        assertEquals(0.0, instantaneousAvgTriad1.valueX, 0.0)
        assertEquals(0.0, instantaneousAvgTriad1.valueY, 0.0)
        assertEquals(0.0, instantaneousAvgTriad1.valueZ, 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, instantaneousAvgTriad1.unit)
        val instantaneousAvgTriad2 = MagneticFluxDensityTriad()
        detector.getInstantaneousAvgTriad(instantaneousAvgTriad2)
        assertEquals(instantaneousAvgTriad1, instantaneousAvgTriad2)
        assertEquals(0.0, detector.instantaneousStdX, 0.0)
        val instantaneousStdX1 = detector.instantaneousStdXAsMagneticFluxDensity
        assertEquals(0.0, instantaneousStdX1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, instantaneousStdX1.unit)
        val instantaneousStdX2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getInstantaneousStdXAsMagneticFluxDensity(instantaneousStdX2)
        assertEquals(instantaneousStdX1, instantaneousStdX2)
        assertEquals(0.0, detector.instantaneousStdY, 0.0)
        val instantaneousStdY1 = detector.instantaneousStdYAsMagneticFluxDensity
        assertEquals(0.0, instantaneousStdY1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, instantaneousStdY1.unit)
        val instantaneousStdY2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getInstantaneousStdYAsMagneticFluxDensity(instantaneousStdY2)
        assertEquals(instantaneousStdY1, instantaneousStdY2)
        assertEquals(0.0, detector.instantaneousStdZ, 0.0)
        val instantaneousStdZ1 = detector.instantaneousStdZAsMagneticFluxDensity
        assertEquals(0.0, instantaneousStdZ1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, instantaneousStdZ1.unit)
        val instantaneousStdZ2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getInstantaneousStdZAsMagneticFluxDensity(instantaneousStdZ2)
        assertEquals(instantaneousStdZ1, instantaneousStdZ2)
        val instantaneousStdTriad1 = detector.instantaneousStdTriad
        assertEquals(0.0, instantaneousAvgTriad1.valueX, 0.0)
        assertEquals(0.0, instantaneousStdTriad1.valueY, 0.0)
        assertEquals(0.0, instantaneousStdTriad1.valueZ, 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, instantaneousStdTriad1.unit)
        val instantaneousStdTriad2 = MagneticFluxDensityTriad()
        detector.getInstantaneousStdTriad(instantaneousStdTriad2)
        assertEquals(instantaneousStdTriad1, instantaneousStdTriad2)
        assertNull(detector.averageTimeInterval)
        assertNull(detector.averageTimeIntervalAsTime)
        val time = Time(0.0, TimeUnit.SECOND)
        assertFalse(detector.getAverageTimeIntervalAsTime(time))
        assertNull(detector.timeIntervalVariance)
        assertNull(detector.timeIntervalStandardDeviation)
        assertNull(detector.timeIntervalStandardDeviationAsTime)
        assertFalse(detector.getTimeIntervalStandardDeviationAsTime(time))
        assertEquals(0, detector.numberOfProcessedMeasurements)
        assertFalse(detector.running)
        assertEquals(MagnetometerIntervalDetector.Status.IDLE, detector.status)
    }

    @Test
    fun constructor_whenAccuracyChangedListener_setsExpectedValues() {
        val initializationStartedListener =
            mockk<MagnetometerIntervalDetector.OnInitializationStartedListener>()
        val initializationCompletedListener =
            mockk<MagnetometerIntervalDetector.OnInitializationCompletedListener>()
        val errorListener = mockk<MagnetometerIntervalDetector.OnErrorListener>()
        val staticIntervalDetectedListener =
            mockk<MagnetometerIntervalDetector.OnStaticIntervalDetectedListener>()
        val dynamicIntervalDetectedListener =
            mockk<MagnetometerIntervalDetector.OnDynamicIntervalDetectedListener>()
        val resetListener = mockk<MagnetometerIntervalDetector.OnResetListener>()
        val measurementListener = mockk<MagnetometerSensorCollector.OnMeasurementListener>()
        val accuracyChangedListener = mockk<SensorCollector.OnAccuracyChangedListener>()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(
            context,
            MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED,
            SensorDelay.NORMAL,
            initializationStartedListener,
            initializationCompletedListener,
            errorListener,
            staticIntervalDetectedListener,
            dynamicIntervalDetectedListener,
            resetListener,
            measurementListener,
            accuracyChangedListener
        )

        // check default values
        assertSame(context, detector.context)
        assertEquals(
            MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED,
            detector.sensorType
        )
        assertEquals(SensorDelay.NORMAL, detector.sensorDelay)
        assertSame(initializationStartedListener, detector.initializationStartedListener)
        assertSame(initializationCompletedListener, detector.initializationCompletedListener)
        assertSame(errorListener, detector.errorListener)
        assertSame(staticIntervalDetectedListener, detector.staticIntervalDetectedListener)
        assertSame(dynamicIntervalDetectedListener, detector.dynamicIntervalDetectedListener)
        assertSame(resetListener, detector.resetListener)
        assertSame(measurementListener, detector.measurementListener)
        assertSame(accuracyChangedListener, detector.accuracyChangedListener)
        assertNull(detector.sensor)
        assertEquals(TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE, detector.windowSize)
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES,
            detector.initialStaticSamples
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_THRESHOLD_FACTOR,
            detector.thresholdFactor,
            0.0
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR,
            detector.instantaneousNoiseLevelFactor,
            0.0
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
            detector.baseNoiseLevelAbsoluteThreshold,
            0.0
        )
        val baseNoiseLevelAbsoluteThreshold1 =
            detector.baseNoiseLevelAbsoluteThresholdAsMagneticFluxDensity
        assertEquals(
            detector.baseNoiseLevelAbsoluteThreshold,
            baseNoiseLevelAbsoluteThreshold1.value.toDouble(),
            0.0
        )
        assertEquals(MagneticFluxDensityUnit.TESLA, baseNoiseLevelAbsoluteThreshold1.unit)
        val baseNoiseLevelAbsoluteThreshold2 =
            MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getBaseNoiseLevelAbsoluteThresholdAsMagneticFluxDensity(
            baseNoiseLevelAbsoluteThreshold2
        )
        assertEquals(baseNoiseLevelAbsoluteThreshold1, baseNoiseLevelAbsoluteThreshold2)
        assertNull(detector.baseNoiseLevel)
        assertNull(detector.baseNoiseLevelAsMagneticFluxDensity)
        val baseNoiseLevel = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(detector.getBaseNoiseLevelAsMagneticFluxDensity(baseNoiseLevel))
        assertNull(detector.baseNoiseLevelPsd)
        assertNull(detector.baseNoiseLevelRootPsd)
        assertNull(detector.threshold)
        assertNull(detector.thresholdAsMagneticFluxDensity)
        val threshold = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(detector.getThresholdAsMeasurement(threshold))
        assertEquals(0.0, detector.accumulatedAvgX, 0.0)
        val accumulatedAvgX1 = detector.accumulatedAvgXAsMagneticFluxDensity
        assertEquals(0.0, accumulatedAvgX1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, accumulatedAvgX1.unit)
        val accumulatedAvgX2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getAccumulatedAvgXAsMagneticFluxDensity(accumulatedAvgX2)
        assertEquals(accumulatedAvgX1, accumulatedAvgX2)
        assertEquals(accumulatedAvgX1, accumulatedAvgX2)
        assertEquals(0.0, detector.accumulatedAvgY, 0.0)
        val accumulatedAvgY1 = detector.accumulatedAvgYAsMagneticFluxDensity
        assertEquals(0.0, accumulatedAvgY1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, accumulatedAvgY1.unit)
        val accumulatedAvgY2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getAccumulatedAvgYAsMagneticFluxDensity(accumulatedAvgY2)
        assertEquals(accumulatedAvgY1, accumulatedAvgY2)
        assertEquals(0.0, detector.accumulatedAvgZ, 0.0)
        val accumulatedAvgZ1 = detector.accumulatedAvgZAsMagneticFluxDensity
        assertEquals(0.0, accumulatedAvgZ1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, accumulatedAvgZ1.unit)
        val accumulatedAvgZ2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getAccumulatedAvgZAsMagneticFluxDensity(accumulatedAvgZ2)
        assertEquals(accumulatedAvgZ1, accumulatedAvgZ2)
        val accumulatedAvgTriad1 = detector.accumulatedAvgTriad
        assertEquals(0.0, accumulatedAvgTriad1.valueX, 0.0)
        assertEquals(0.0, accumulatedAvgTriad1.valueY, 0.0)
        assertEquals(0.0, accumulatedAvgTriad1.valueZ, 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, accumulatedAvgTriad1.unit)
        val accumulatedAvgTriad2 = MagneticFluxDensityTriad()
        detector.getAccumulatedAvgTriad(accumulatedAvgTriad2)
        assertEquals(accumulatedAvgTriad1, accumulatedAvgTriad2)
        assertEquals(0.0, detector.accumulatedStdX, 0.0)
        val accumulatedStdX1 = detector.accumulatedStdXAsMagneticFluxDensity
        assertEquals(0.0, accumulatedStdX1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, accumulatedStdX1.unit)
        val accumulatedStdX2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getAccumulatedStdXAsMagneticFluxDensity(accumulatedStdX2)
        assertEquals(accumulatedStdX1, accumulatedStdX2)
        assertEquals(0.0, detector.accumulatedStdY, 0.0)
        val accumulatedStdY1 = detector.accumulatedStdYAsMagneticFluxDensity
        assertEquals(0.0, accumulatedStdY1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, accumulatedStdY1.unit)
        val accumulatedStdY2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getAccumulatedStdYAsMagneticFluxDensity(accumulatedStdY2)
        assertEquals(accumulatedStdY1, accumulatedStdY2)
        assertEquals(0.0, detector.accumulatedStdZ, 0.0)
        val accumulatedStdZ1 = detector.accumulatedStdZAsMagneticFluxDensity
        assertEquals(0.0, accumulatedStdZ1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, accumulatedStdZ1.unit)
        val accumulatedStdZ2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getAccumulatedStdZAsMagneticFluxDensity(accumulatedStdZ2)
        assertEquals(accumulatedStdZ1, accumulatedStdZ2)
        val accumulatedStdTriad1 = detector.accumulatedStdTriad
        assertEquals(0.0, accumulatedStdTriad1.valueX, 0.0)
        assertEquals(0.0, accumulatedStdTriad1.valueY, 0.0)
        assertEquals(0.0, accumulatedStdTriad1.valueZ, 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, accumulatedStdTriad1.unit)
        val accumulatedStdTriad2 = MagneticFluxDensityTriad()
        detector.getAccumulatedStdTriad(accumulatedStdTriad2)
        assertEquals(accumulatedStdTriad1, accumulatedAvgTriad2)
        assertEquals(0.0, detector.instantaneousAvgX, 0.0)
        val instantaneousAvgX1 = detector.instantaneousAvgXAsMagneticFluxDensity
        assertEquals(0.0, instantaneousAvgX1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, instantaneousAvgX1.unit)
        val instantaneousAvgX2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getInstantaneousAvgXAsMagneticFluxDensity(instantaneousAvgX2)
        assertEquals(instantaneousAvgX1, instantaneousAvgX2)
        assertEquals(0.0, detector.instantaneousAvgY, 0.0)
        val instantaneousAvgY1 = detector.instantaneousAvgYAsMagneticFluxDensity
        assertEquals(0.0, instantaneousAvgY1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, instantaneousAvgY1.unit)
        val instantaneousAvgY2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getInstantaneousAvgYAsMagneticFluxDensity(instantaneousAvgY2)
        assertEquals(instantaneousAvgY1, instantaneousAvgY2)
        assertEquals(0.0, detector.instantaneousAvgZ, 0.0)
        val instantaneousAvgZ1 = detector.instantaneousAvgZAsMagneticFluxDensity
        assertEquals(0.0, instantaneousAvgZ1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, instantaneousAvgZ1.unit)
        val instantaneousAvgZ2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getInstantaneousAvgZAsMagneticFluxDensity(instantaneousAvgZ2)
        assertEquals(instantaneousAvgZ1, instantaneousAvgZ2)
        val instantaneousAvgTriad1 = detector.instantaneousAvgTriad
        assertEquals(0.0, instantaneousAvgTriad1.valueX, 0.0)
        assertEquals(0.0, instantaneousAvgTriad1.valueY, 0.0)
        assertEquals(0.0, instantaneousAvgTriad1.valueZ, 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, instantaneousAvgTriad1.unit)
        val instantaneousAvgTriad2 = MagneticFluxDensityTriad()
        detector.getInstantaneousAvgTriad(instantaneousAvgTriad2)
        assertEquals(instantaneousAvgTriad1, instantaneousAvgTriad2)
        assertEquals(0.0, detector.instantaneousStdX, 0.0)
        val instantaneousStdX1 = detector.instantaneousStdXAsMagneticFluxDensity
        assertEquals(0.0, instantaneousStdX1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, instantaneousStdX1.unit)
        val instantaneousStdX2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getInstantaneousStdXAsMagneticFluxDensity(instantaneousStdX2)
        assertEquals(instantaneousStdX1, instantaneousStdX2)
        assertEquals(0.0, detector.instantaneousStdY, 0.0)
        val instantaneousStdY1 = detector.instantaneousStdYAsMagneticFluxDensity
        assertEquals(0.0, instantaneousStdY1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, instantaneousStdY1.unit)
        val instantaneousStdY2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getInstantaneousStdYAsMagneticFluxDensity(instantaneousStdY2)
        assertEquals(instantaneousStdY1, instantaneousStdY2)
        assertEquals(0.0, detector.instantaneousStdZ, 0.0)
        val instantaneousStdZ1 = detector.instantaneousStdZAsMagneticFluxDensity
        assertEquals(0.0, instantaneousStdZ1.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, instantaneousStdZ1.unit)
        val instantaneousStdZ2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getInstantaneousStdZAsMagneticFluxDensity(instantaneousStdZ2)
        assertEquals(instantaneousStdZ1, instantaneousStdZ2)
        val instantaneousStdTriad1 = detector.instantaneousStdTriad
        assertEquals(0.0, instantaneousAvgTriad1.valueX, 0.0)
        assertEquals(0.0, instantaneousStdTriad1.valueY, 0.0)
        assertEquals(0.0, instantaneousStdTriad1.valueZ, 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, instantaneousStdTriad1.unit)
        val instantaneousStdTriad2 = MagneticFluxDensityTriad()
        detector.getInstantaneousStdTriad(instantaneousStdTriad2)
        assertEquals(instantaneousStdTriad1, instantaneousStdTriad2)
        assertNull(detector.averageTimeInterval)
        assertNull(detector.averageTimeIntervalAsTime)
        val time = Time(0.0, TimeUnit.SECOND)
        assertFalse(detector.getAverageTimeIntervalAsTime(time))
        assertNull(detector.timeIntervalVariance)
        assertNull(detector.timeIntervalStandardDeviation)
        assertNull(detector.timeIntervalStandardDeviationAsTime)
        assertFalse(detector.getTimeIntervalStandardDeviationAsTime(time))
        assertEquals(0, detector.numberOfProcessedMeasurements)
        assertFalse(detector.running)
        assertEquals(MagnetometerIntervalDetector.Status.IDLE, detector.status)
    }

    @Test
    fun initializationStartedListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        // check default value
        assertNull(detector.initializationStartedListener)

        // set new value
        val initializationStartedListener =
            mockk<MagnetometerIntervalDetector.OnInitializationStartedListener>()
        detector.initializationStartedListener = initializationStartedListener

        // check
        assertSame(initializationStartedListener, detector.initializationStartedListener)
    }

    @Test
    fun initializationCompletedListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        // check default value
        assertNull(detector.initializationCompletedListener)

        // set new value
        val initializationCompletedListener =
            mockk<MagnetometerIntervalDetector.OnInitializationCompletedListener>()
        detector.initializationCompletedListener = initializationCompletedListener

        // check
        assertSame(initializationCompletedListener, detector.initializationCompletedListener)
    }

    @Test
    fun errorListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        // check default value
        assertNull(detector.errorListener)

        // set new value
        val errorListener = mockk<MagnetometerIntervalDetector.OnErrorListener>()
        detector.errorListener = errorListener

        // check
        assertSame(errorListener, detector.errorListener)
    }

    @Test
    fun staticIntervalDetectedlistener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        // check default value
        assertNull(detector.staticIntervalDetectedListener)

        // sets new value
        val staticIntervalDetectedListener =
            mockk<MagnetometerIntervalDetector.OnStaticIntervalDetectedListener>()
        detector.staticIntervalDetectedListener = staticIntervalDetectedListener

        // check
        assertSame(staticIntervalDetectedListener, detector.staticIntervalDetectedListener)
    }

    @Test
    fun dynamicIntervalDetectedListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        // check default value
        assertNull(detector.dynamicIntervalDetectedListener)

        // sets new value
        val dynamicIntervalDetectedListener =
            mockk<MagnetometerIntervalDetector.OnDynamicIntervalDetectedListener>()
        detector.dynamicIntervalDetectedListener = dynamicIntervalDetectedListener

        // check
        assertSame(dynamicIntervalDetectedListener, detector.dynamicIntervalDetectedListener)
    }

    @Test
    fun resetListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        // check default value
        assertNull(detector.resetListener)

        // set new value
        val resetListener = mockk<MagnetometerIntervalDetector.OnResetListener>()
        detector.resetListener = resetListener

        // check
        assertSame(resetListener, detector.resetListener)
    }

    @Test
    fun measurementListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        // check default value
        assertNull(detector.measurementListener)

        // set new value
        val measurementListener = mockk<MagnetometerSensorCollector.OnMeasurementListener>()
        detector.measurementListener = measurementListener

        // check
        assertSame(measurementListener, detector.measurementListener)
    }

    @Test
    fun accuracyChangedListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        // check default value
        assertNull(detector.accuracyChangedListener)

        // set new value
        val accuracyChangedListener = mockk<SensorCollector.OnAccuracyChangedListener>()
        detector.accuracyChangedListener = accuracyChangedListener

        // check
        assertSame(accuracyChangedListener, detector.accuracyChangedListener)
    }

    @Test
    fun sensor_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        val collector: MagnetometerSensorCollector? =
            detector.getPrivateProperty("collector")
        requireNotNull(collector)

        val collectorSpy = spyk(collector)
        val sensor = mockk<Sensor>()
        every { collectorSpy.sensor }.returns(sensor)
        detector.setPrivateProperty("collector", collectorSpy)

        assertSame(sensor, detector.sensor)
    }

    @Test
    fun windowSize_whenValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        // check default value
        assertEquals(TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE, detector.windowSize)

        // set new value
        detector.windowSize = WINDOW_SIZE

        // check
        assertEquals(WINDOW_SIZE, detector.windowSize)
    }

    @Test(expected = IllegalArgumentException::class)
    fun windowSize_whenInvalid_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        detector.windowSize = 0
    }

    @Test(expected = IllegalStateException::class)
    fun windowSize_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        detector.setPrivateProperty("running", true)

        detector.windowSize = WINDOW_SIZE
    }

    @Test
    fun initialStaticSamples_whenValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        // check default value
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES,
            detector.initialStaticSamples
        )

        // set new value
        detector.initialStaticSamples = INITIAL_STATIC_SAMPLES

        // check
        assertEquals(INITIAL_STATIC_SAMPLES, detector.initialStaticSamples)
    }

    @Test(expected = IllegalArgumentException::class)
    fun initialStaticSamples_whenInvalid_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        detector.initialStaticSamples = 0
    }

    @Test(expected = IllegalStateException::class)
    fun initialStaticSamples_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        detector.setPrivateProperty("running", true)

        detector.initialStaticSamples = TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES
    }

    @Test
    fun thresholdFactor_whenValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        // check default value
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_THRESHOLD_FACTOR,
            detector.thresholdFactor,
            0.0
        )

        // set new value
        detector.thresholdFactor = THRESHOLD_FACTOR

        // check
        assertEquals(THRESHOLD_FACTOR, detector.thresholdFactor, 0.0)
    }

    @Test(expected = IllegalArgumentException::class)
    fun thresholdFactor_whenInvalid_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        detector.thresholdFactor = 0.0
    }

    @Test(expected = IllegalStateException::class)
    fun thresholdFactor_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        detector.setPrivateProperty("running", true)

        detector.thresholdFactor = TriadStaticIntervalDetector.DEFAULT_THRESHOLD_FACTOR
    }

    @Test
    fun instantaneousNoiseLevelFactor_whenValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        // check default value
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR,
            detector.instantaneousNoiseLevelFactor,
            0.0
        )

        // set new value
        detector.instantaneousNoiseLevelFactor = INSTANTANEOUS_NOISE_LEVEL_FACTOR

        // check
        assertEquals(INSTANTANEOUS_NOISE_LEVEL_FACTOR, detector.instantaneousNoiseLevelFactor, 0.0)
    }

    @Test(expected = IllegalArgumentException::class)
    fun instantaneousNoiseLevelFactor_whenInvalid_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        detector.instantaneousNoiseLevelFactor = 0.0
    }

    @Test(expected = IllegalStateException::class)
    fun instantaneousNoiseLevelFactor_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        detector.setPrivateProperty("running", true)

        detector.instantaneousNoiseLevelFactor =
            TriadStaticIntervalDetector.DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR
    }

    @Test
    fun baseNoiseLevelAbsoluteThreshold_whenValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        // check default value
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
            detector.baseNoiseLevelAbsoluteThreshold,
            0.0
        )

        // set new value
        detector.baseNoiseLevelAbsoluteThreshold = BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD

        // check
        assertEquals(
            BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
            detector.baseNoiseLevelAbsoluteThreshold,
            0.0
        )
    }

    @Test(expected = IllegalArgumentException::class)
    fun baseNoiseLevelAbsoluteThreshold_whenInvalid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        detector.baseNoiseLevelAbsoluteThreshold = 0.0
    }

    @Test(expected = IllegalStateException::class)
    fun baseNoiseLevelAbsoluteThreshold_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        detector.setPrivateProperty("running", true)

        detector.baseNoiseLevelAbsoluteThreshold =
            TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD
    }

    @Test
    fun baseNoiseLevelAbsoluteThresholdAsMagneticFluxDensity_whenValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        // check default value
        val baseNoiseLevelAbsoluteThreshold1 =
            detector.baseNoiseLevelAbsoluteThresholdAsMagneticFluxDensity
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
            baseNoiseLevelAbsoluteThreshold1.value.toDouble(),
            0.0
        )
        assertEquals(MagneticFluxDensityUnit.TESLA, baseNoiseLevelAbsoluteThreshold1.unit)
        val baseNoiseLevelAbsoluteThreshold2 =
            MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getBaseNoiseLevelAbsoluteThresholdAsMagneticFluxDensity(
            baseNoiseLevelAbsoluteThreshold2
        )
        assertEquals(baseNoiseLevelAbsoluteThreshold1, baseNoiseLevelAbsoluteThreshold2)

        // set new value
        val baseNoiseLevelAbsoluteThreshold3 = MagneticFluxDensity(
            BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD, MagneticFluxDensityUnit.TESLA
        )
        detector.baseNoiseLevelAbsoluteThresholdAsMagneticFluxDensity =
            baseNoiseLevelAbsoluteThreshold3

        // check
        val baseNoiseLevelAbsoluteThreshold4 =
            detector.baseNoiseLevelAbsoluteThresholdAsMagneticFluxDensity
        assertEquals(
            BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
            baseNoiseLevelAbsoluteThreshold4.value.toDouble(),
            0.0
        )
        assertEquals(MagneticFluxDensityUnit.TESLA, baseNoiseLevelAbsoluteThreshold4.unit)
        val baseNoiseLevelAbsoluteThreshold5 =
            MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        detector.getBaseNoiseLevelAbsoluteThresholdAsMagneticFluxDensity(
            baseNoiseLevelAbsoluteThreshold5
        )
        assertEquals(baseNoiseLevelAbsoluteThreshold4, baseNoiseLevelAbsoluteThreshold5)
    }

    @Test(expected = IllegalArgumentException::class)
    fun baseNoiseLevelAbsoluteThresholdAsMagneticFluxDensity_whenInvalid_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        detector.baseNoiseLevelAbsoluteThresholdAsMagneticFluxDensity =
            MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
    }

    @Test(expected = IllegalStateException::class)
    fun baseNoiseLevelAbsoluteThresholdAsMagneticFluxDensity_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        detector.setPrivateProperty("running", true)

        detector.baseNoiseLevelAbsoluteThresholdAsMagneticFluxDensity = MagneticFluxDensity(
            BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD, MagneticFluxDensityUnit.TESLA
        )
    }

    @Test
    fun start_whenSensorAvailable_startsCollector() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        val collector: MagnetometerSensorCollector? =
            detector.getPrivateProperty("collector")
        requireNotNull(collector)
        assertSame(context, collector.context)
        assertEquals(collector.sensorType, detector.sensorType)
        assertEquals(collector.sensorDelay, detector.sensorDelay)
        assertNotNull(collector.measurementListener)
        assertNotNull(collector.accuracyChangedListener)

        val collectorSpy = spyk(collector)
        every { collectorSpy.start() }.returns(true)
        detector.setPrivateProperty("collector", collectorSpy)

        assertFalse(detector.running)

        detector.start()

        assertTrue(detector.running)
        verify(exactly = 1) { collectorSpy.start() }
    }

    @Test(expected = IllegalStateException::class)
    fun start_whenSensorUnavailable_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        val collector: MagnetometerSensorCollector? =
            detector.getPrivateProperty("collector")
        requireNotNull(collector)
        assertSame(context, collector.context)
        assertEquals(collector.sensorType, detector.sensorType)
        assertEquals(collector.sensorDelay, detector.sensorDelay)
        assertNotNull(collector.measurementListener)
        assertNotNull(collector.accuracyChangedListener)

        val collectorSpy = spyk(collector)
        every { collectorSpy.start() }.returns(false)
        detector.setPrivateProperty("collector", collectorSpy)

        assertFalse(detector.running)

        detector.start()
    }

    @Test
    fun start_resets() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        val collector: MagnetometerSensorCollector? =
            detector.getPrivateProperty("collector")
        requireNotNull(collector)
        val collectorSpy = spyk(collector)
        every { collectorSpy.start() }.returns(true)
        detector.setPrivateProperty("collector", collectorSpy)

        val internalDetector: MagneticFluxDensityTriadStaticIntervalDetector? =
            detector.getPrivateProperty("internalDetector")
        requireNotNull(internalDetector)
        val internalDetectorSpy = spyk(internalDetector)
        detector.setPrivateProperty("internalDetector", internalDetectorSpy)

        val timeIntervalEstimator: TimeIntervalEstimator? =
            detector.getPrivateProperty("timeIntervalEstimator")
        requireNotNull(timeIntervalEstimator)
        val timeIntervalEstimatorSpy = spyk(timeIntervalEstimator)
        detector.setPrivateProperty("timeIntervalEstimator", timeIntervalEstimatorSpy)

        detector.setPrivateProperty("unreliable", true)
        detector.setPrivateProperty("initialTimestamp", SystemClock.elapsedRealtimeNanos())
        detector.setPrivateProperty("numberOfProcessedMeasurements", 1)
        detector.setPrivateProperty("initialized", true)

        detector.start()

        assertEquals(Integer.MAX_VALUE, timeIntervalEstimatorSpy.totalSamples)
        verify(exactly = 1) { timeIntervalEstimatorSpy.reset() }
        verify(exactly = 1) { internalDetectorSpy.reset() }

        val unreliable: Boolean? = detector.getPrivateProperty("unreliable")
        requireNotNull(unreliable)
        assertFalse(unreliable)

        val initialTimestamp: Long? = detector.getPrivateProperty("initialTimestamp")
        requireNotNull(initialTimestamp)
        assertEquals(0L, initialTimestamp)

        val numberOfProcessedMeasurements: Int? =
            detector.getPrivateProperty("numberOfProcessedMeasurements")
        requireNotNull(numberOfProcessedMeasurements)
        assertEquals(0, numberOfProcessedMeasurements)

        val initialized: Boolean? = detector.getPrivateProperty("initialized")
        requireNotNull(initialized)
        assertFalse(initialized)
    }

    @Test(expected = IllegalStateException::class)
    fun start_whenAlreadyRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        assertFalse(detector.running)

        detector.start()

        assertTrue(detector.running)

        // start again
        detector.start()
    }

    @Test
    fun stop_whenAlreadyStarted_stopsSensorCollector() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        val collector: MagnetometerSensorCollector? = detector.getPrivateProperty("collector")
        requireNotNull(collector)
        assertSame(context, collector.context)
        assertEquals(collector.sensorType, detector.sensorType)
        assertEquals(collector.sensorDelay, detector.sensorDelay)
        assertNotNull(collector.measurementListener)
        assertNotNull(collector.accuracyChangedListener)

        val collectorSpy = spyk(collector)
        every { collectorSpy.start() }.returns(true)
        detector.setPrivateProperty("collector", collectorSpy)

        assertFalse(detector.running)

        detector.start()

        assertTrue(detector.running)
        verify(exactly = 1) { collectorSpy.start() }

        // stop
        detector.stop()

        assertFalse(detector.running)
        verify(exactly = 1) { collectorSpy.stop() }
    }

    @Test
    fun stop_whenNotAlreadyStarted_stopsSensorCollector() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        val collector: MagnetometerSensorCollector? = detector.getPrivateProperty("collector")
        requireNotNull(collector)
        assertSame(context, collector.context)
        assertEquals(collector.sensorType, detector.sensorType)
        assertEquals(collector.sensorDelay, detector.sensorDelay)
        assertNotNull(collector.measurementListener)
        assertNotNull(collector.accuracyChangedListener)

        val collectorSpy = spyk(collector)
        detector.setPrivateProperty("collector", collectorSpy)

        assertFalse(detector.running)

        // stop
        detector.stop()

        assertFalse(detector.running)
        verify(exactly = 1) { collectorSpy.stop() }
    }

    @Test
    fun mapErrorReason_whenUnreliable_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        detector.setPrivateProperty("unreliable", true)

        var errorReason: MagnetometerIntervalDetector.ErrorReason? =
            detector.callPrivateFuncWithResult(
                "mapErrorReason",
                TriadStaticIntervalDetector.ErrorReason.OVERALL_EXCESSIVE_MOVEMENT_DETECTED
            )
        assertEquals(MagnetometerIntervalDetector.ErrorReason.UNRELIABLE_SENSOR, errorReason)

        errorReason = detector.callPrivateFuncWithResult(
            "mapErrorReason",
            TriadStaticIntervalDetector.ErrorReason.SUDDEN_EXCESSIVE_MOVEMENT_DETECTED
        )
        assertEquals(MagnetometerIntervalDetector.ErrorReason.UNRELIABLE_SENSOR, errorReason)
    }

    @Test
    fun mapErrorReason_whenReliable_returnsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        val unreliable: Boolean? = detector.getPrivateProperty("unreliable")
        requireNotNull(unreliable)
        assertFalse(unreliable)

        var errorReason: MagnetometerIntervalDetector.ErrorReason? =
            detector.callPrivateFuncWithResult(
                "mapErrorReason",
                TriadStaticIntervalDetector.ErrorReason.OVERALL_EXCESSIVE_MOVEMENT_DETECTED
            )
        assertEquals(
            MagnetometerIntervalDetector.ErrorReason.OVERALL_EXCESSIVE_MOVEMENT_DETECTED_DURING_INITIALIZATION,
            errorReason
        )

        errorReason = detector.callPrivateFuncWithResult(
            "mapErrorReason",
            TriadStaticIntervalDetector.ErrorReason.SUDDEN_EXCESSIVE_MOVEMENT_DETECTED
        )
        assertEquals(
            MagnetometerIntervalDetector.ErrorReason.SUDDEN_EXCESSIVE_MOVEMENT_DETECTED_DURING_INITIALIZATION,
            errorReason
        )
    }

    @Test
    fun onMeasurement_whenIdle_processMeasurement() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        val internalDetector: MagneticFluxDensityTriadStaticIntervalDetector? =
            detector.getPrivateProperty("internalDetector")
        requireNotNull(internalDetector)
        val internalDetectorSpy = spyk(internalDetector)
        detector.setPrivateProperty("internalDetector", internalDetectorSpy)

        // check initial status
        assertEquals(MagnetometerIntervalDetector.Status.IDLE, detector.status)
        assertEquals(0, detector.numberOfProcessedMeasurements)

        // process measurement
        val measurementListener: MagnetometerSensorCollector.OnMeasurementListener? =
            detector.getPrivateProperty("internalMeasurementListener")
        requireNotNull(measurementListener)

        val b = getMagneticFluxDensity()
        val bx = b.bx.toFloat()
        val by = b.by.toFloat()
        val bz = b.bz.toFloat()
        val timestamp = SystemClock.elapsedRealtimeNanos()
        val accuracy = SensorAccuracy.HIGH

        measurementListener.onMeasurement(bx, by, bz, null, null, null, timestamp, accuracy)

        // check
        val bxT = MagneticFluxDensityConverter.convert(
            bx.toDouble(),
            MagneticFluxDensityUnit.MICROTESLA,
            MagneticFluxDensityUnit.TESLA
        )
        val byT = MagneticFluxDensityConverter.convert(
            by.toDouble(),
            MagneticFluxDensityUnit.MICROTESLA,
            MagneticFluxDensityUnit.TESLA
        )
        val bzT = MagneticFluxDensityConverter.convert(
            bz.toDouble(),
            MagneticFluxDensityUnit.MICROTESLA,
            MagneticFluxDensityUnit.TESLA
        )
        verify(exactly = 1) { internalDetectorSpy.process(bxT, byT, bzT) }
        assertEquals(1, detector.numberOfProcessedMeasurements)
        assertEquals(MagnetometerIntervalDetector.Status.INITIALIZING, detector.status)
    }

    @Test
    fun onMeasurement_whenInitializingStatusAndZeroProcessedMeasurement_setsInitialTimestamp() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        val internalDetector: MagneticFluxDensityTriadStaticIntervalDetector? =
            detector.getPrivateProperty("internalDetector")
        requireNotNull(internalDetector)
        val internalDetectorSpy = spyk(internalDetector)
        every { internalDetectorSpy.status }.returns(TriadStaticIntervalDetector.Status.INITIALIZING)
        detector.setPrivateProperty("internalDetector", internalDetectorSpy)

        // check initial values
        val initialTimestamp1: Long? = detector.getPrivateProperty("initialTimestamp")
        requireNotNull(initialTimestamp1)
        assertEquals(0L, initialTimestamp1)

        assertEquals(MagnetometerIntervalDetector.Status.INITIALIZING, detector.status)
        assertEquals(0, detector.numberOfProcessedMeasurements)

        // process measurement
        val measurementListener: MagnetometerSensorCollector.OnMeasurementListener? =
            detector.getPrivateProperty("internalMeasurementListener")
        requireNotNull(measurementListener)

        val b = getMagneticFluxDensity()
        val bx = b.bx.toFloat()
        val by = b.by.toFloat()
        val bz = b.bz.toFloat()
        val timestamp = SystemClock.elapsedRealtimeNanos()
        val accuracy = SensorAccuracy.HIGH

        measurementListener.onMeasurement(bx, by, bz, null, null, null, timestamp, accuracy)

        // check
        val bxT = MagneticFluxDensityConverter.convert(
            bx.toDouble(),
            MagneticFluxDensityUnit.MICROTESLA,
            MagneticFluxDensityUnit.TESLA
        )
        val byT = MagneticFluxDensityConverter.convert(
            by.toDouble(),
            MagneticFluxDensityUnit.MICROTESLA,
            MagneticFluxDensityUnit.TESLA
        )
        val bzT = MagneticFluxDensityConverter.convert(
            bz.toDouble(),
            MagneticFluxDensityUnit.MICROTESLA,
            MagneticFluxDensityUnit.TESLA
        )
        verify(exactly = 1) { internalDetectorSpy.process(bxT, byT, bzT) }
        assertEquals(1, detector.numberOfProcessedMeasurements)
        assertEquals(MagnetometerIntervalDetector.Status.INITIALIZING, detector.status)

        val initialTimestamp2: Long? = detector.getPrivateProperty("initialTimestamp")
        requireNotNull(initialTimestamp2)
        assertEquals(timestamp, initialTimestamp2)
    }

    @Test
    fun onMeasurement_whenInitializingStatusAndNonZeroProcessedMeasurement_addsTimestampToTimeIntervalEstimator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        val internalDetector: MagneticFluxDensityTriadStaticIntervalDetector? =
            detector.getPrivateProperty("internalDetector")
        requireNotNull(internalDetector)
        val internalDetectorSpy = spyk(internalDetector)
        every { internalDetectorSpy.status }.returns(TriadStaticIntervalDetector.Status.INITIALIZING)
        detector.setPrivateProperty("internalDetector", internalDetectorSpy)
        detector.setPrivateProperty("numberOfProcessedMeasurements", 1)
        val timestamp1 = SystemClock.elapsedRealtimeNanos()
        detector.setPrivateProperty("initialTimestamp", timestamp1)

        val timeIntervalEstimator: TimeIntervalEstimator? =
            detector.getPrivateProperty("timeIntervalEstimator")
        requireNotNull(timeIntervalEstimator)
        val timeIntervalEstimatorSpy = spyk(timeIntervalEstimator)
        detector.setPrivateProperty("timeIntervalEstimator", timeIntervalEstimatorSpy)

        // check initial values
        val initialTimestamp1: Long? = detector.getPrivateProperty("initialTimestamp")
        requireNotNull(initialTimestamp1)
        assertEquals(timestamp1, initialTimestamp1)

        assertEquals(MagnetometerIntervalDetector.Status.INITIALIZING, detector.status)
        assertEquals(1, detector.numberOfProcessedMeasurements)

        // process measurement
        val measurementListener: MagnetometerSensorCollector.OnMeasurementListener? =
            detector.getPrivateProperty("internalMeasurementListener")
        requireNotNull(measurementListener)

        val b = getMagneticFluxDensity()
        val bx = b.bx.toFloat()
        val by = b.by.toFloat()
        val bz = b.bz.toFloat()
        val timestamp2 = timestamp1 + TIME_INTERVAL_MILLIS * MILLIS_TO_NANOS
        val accuracy = SensorAccuracy.HIGH

        measurementListener.onMeasurement(bx, by, bz, null, null, null, timestamp2, accuracy)

        // check
        verify(exactly = 1) { timeIntervalEstimatorSpy.addTimestamp(TIME_INTERVAL_SECONDS) }
        val bxT = MagneticFluxDensityConverter.convert(
            bx.toDouble(),
            MagneticFluxDensityUnit.MICROTESLA,
            MagneticFluxDensityUnit.TESLA
        )
        val byT = MagneticFluxDensityConverter.convert(
            by.toDouble(),
            MagneticFluxDensityUnit.MICROTESLA,
            MagneticFluxDensityUnit.TESLA
        )
        val bzT = MagneticFluxDensityConverter.convert(
            bz.toDouble(),
            MagneticFluxDensityUnit.MICROTESLA,
            MagneticFluxDensityUnit.TESLA
        )
        verify(exactly = 1) { internalDetectorSpy.process(bxT, byT, bzT) }
        assertEquals(2, detector.numberOfProcessedMeasurements)
        assertEquals(MagnetometerIntervalDetector.Status.INITIALIZING, detector.status)

        val initialTimestamp2: Long? = detector.getPrivateProperty("initialTimestamp")
        requireNotNull(initialTimestamp2)
        assertEquals(timestamp1, initialTimestamp2)
    }

    @Test
    fun onMeasurement_whenInitializingAndListener_notifies() {
        val initializationStartedListener =
            mockk<MagnetometerIntervalDetector.OnInitializationStartedListener>(relaxUnitFun = true)
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(
            context,
            initializationStartedListener = initializationStartedListener
        )

        val internalDetector: MagneticFluxDensityTriadStaticIntervalDetector? =
            detector.getPrivateProperty("internalDetector")
        requireNotNull(internalDetector)
        val internalDetectorSpy = spyk(internalDetector)
        every { internalDetectorSpy.status }.returns(TriadStaticIntervalDetector.Status.INITIALIZING)
        detector.setPrivateProperty("internalDetector", internalDetectorSpy)
        detector.setPrivateProperty("numberOfProcessedMeasurements", 1)
        val timestamp1 = SystemClock.elapsedRealtimeNanos()
        detector.setPrivateProperty("initialTimestamp", timestamp1)

        val timeIntervalEstimator: TimeIntervalEstimator? =
            detector.getPrivateProperty("timeIntervalEstimator")
        requireNotNull(timeIntervalEstimator)
        val timeIntervalEstimatorSpy = spyk(timeIntervalEstimator)
        detector.setPrivateProperty("timeIntervalEstimator", timeIntervalEstimatorSpy)

        // check initial values
        val initialTimestamp1: Long? = detector.getPrivateProperty("initialTimestamp")
        requireNotNull(initialTimestamp1)
        assertEquals(timestamp1, initialTimestamp1)

        assertEquals(MagnetometerIntervalDetector.Status.INITIALIZING, detector.status)
        assertEquals(1, detector.numberOfProcessedMeasurements)

        // process measurement
        val measurementListener: MagnetometerSensorCollector.OnMeasurementListener? =
            detector.getPrivateProperty("internalMeasurementListener")
        requireNotNull(measurementListener)

        val b = getMagneticFluxDensity()
        val bx = b.bx.toFloat()
        val by = b.by.toFloat()
        val bz = b.bz.toFloat()
        val timestamp2 = timestamp1 + TIME_INTERVAL_MILLIS * MILLIS_TO_NANOS
        val accuracy = SensorAccuracy.HIGH

        measurementListener.onMeasurement(bx, by, bz, null, null, null, timestamp2, accuracy)

        // check
        verify(exactly = 1) { timeIntervalEstimatorSpy.addTimestamp(TIME_INTERVAL_SECONDS) }
        val bxT = MagneticFluxDensityConverter.convert(
            bx.toDouble(),
            MagneticFluxDensityUnit.MICROTESLA,
            MagneticFluxDensityUnit.TESLA
        )
        val byT = MagneticFluxDensityConverter.convert(
            by.toDouble(),
            MagneticFluxDensityUnit.MICROTESLA,
            MagneticFluxDensityUnit.TESLA
        )
        val bzT = MagneticFluxDensityConverter.convert(
            bz.toDouble(),
            MagneticFluxDensityUnit.MICROTESLA,
            MagneticFluxDensityUnit.TESLA
        )
        verify(exactly = 1) { internalDetectorSpy.process(bxT, byT, bzT) }
        assertEquals(2, detector.numberOfProcessedMeasurements)
        assertEquals(MagnetometerIntervalDetector.Status.INITIALIZING, detector.status)

        val initialTimestamp2: Long? = detector.getPrivateProperty("initialTimestamp")
        requireNotNull(initialTimestamp2)
        assertEquals(timestamp1, initialTimestamp2)

        verify(exactly = 1) { initializationStartedListener.onInitializationStarted(detector) }
    }

    @Test
    fun onMeasurement_whenInitializationCompleted_setsTimeInterval() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        val internalDetector: MagneticFluxDensityTriadStaticIntervalDetector? =
            detector.getPrivateProperty("internalDetector")
        requireNotNull(internalDetector)
        val internalDetectorSpy = spyk(internalDetector)
        every { internalDetectorSpy.status }.returns(TriadStaticIntervalDetector.Status.INITIALIZATION_COMPLETED)
        detector.setPrivateProperty("internalDetector", internalDetectorSpy)

        val timeIntervalEstimator: TimeIntervalEstimator? =
            detector.getPrivateProperty("timeIntervalEstimator")
        requireNotNull(timeIntervalEstimator)
        val timeIntervalEstimatorSpy = spyk(timeIntervalEstimator)
        every { timeIntervalEstimatorSpy.averageTimeInterval }.returns(2.0 * TIME_INTERVAL_SECONDS)
        detector.setPrivateProperty("timeIntervalEstimator", timeIntervalEstimatorSpy)

        // check initial values
        assertEquals(
            WindowedTriadNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
            internalDetector.timeInterval,
            0.0
        )
        val initialized1: Boolean? = detector.getPrivateProperty("initialized")
        requireNotNull(initialized1)
        assertFalse(initialized1)
        assertNull(detector.averageTimeInterval)

        // process measurement
        val measurementListener: MagnetometerSensorCollector.OnMeasurementListener? =
            detector.getPrivateProperty("internalMeasurementListener")
        requireNotNull(measurementListener)

        val b = getMagneticFluxDensity()
        val bx = b.bx.toFloat()
        val by = b.by.toFloat()
        val bz = b.bz.toFloat()
        val timestamp = SystemClock.elapsedRealtimeNanos()
        val accuracy = SensorAccuracy.HIGH

        measurementListener.onMeasurement(bx, by, bz, null, null, null, timestamp, accuracy)

        // check
        assertEquals(2.0 * TIME_INTERVAL_SECONDS, internalDetectorSpy.timeInterval, 0.0)
        val initialized2: Boolean? = detector.getPrivateProperty("initialized")
        requireNotNull(initialized2)
        assertTrue(initialized2)
        assertEquals(2.0 * TIME_INTERVAL_SECONDS, detector.averageTimeInterval)
    }

    @Test
    fun onMeasurement_whenMeasurementListener_notifiesMeasurement() {
        val measurementListener =
            mockk<MagnetometerSensorCollector.OnMeasurementListener>(relaxUnitFun = true)
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector =
            MagnetometerIntervalDetector(context, measurementListener = measurementListener)

        val internalDetector: MagneticFluxDensityTriadStaticIntervalDetector? =
            detector.getPrivateProperty("internalDetector")
        requireNotNull(internalDetector)
        val internalDetectorSpy = spyk(internalDetector)
        detector.setPrivateProperty("internalDetector", internalDetectorSpy)

        // check initial status
        assertEquals(MagnetometerIntervalDetector.Status.IDLE, detector.status)
        assertEquals(0, detector.numberOfProcessedMeasurements)

        // process measurement
        val internalMeasurementListener: MagnetometerSensorCollector.OnMeasurementListener? =
            detector.getPrivateProperty("internalMeasurementListener")
        requireNotNull(internalMeasurementListener)

        val b = getMagneticFluxDensity()
        val bx = b.bx.toFloat()
        val by = b.by.toFloat()
        val bz = b.bz.toFloat()
        val randomizer = UniformRandomizer()
        val hardIronX = randomizer.nextFloat()
        val hardIronY = randomizer.nextFloat()
        val hardIronZ = randomizer.nextFloat()
        val timestamp = SystemClock.elapsedRealtimeNanos()
        val accuracy = SensorAccuracy.HIGH

        measurementListener.onMeasurement(
            bx,
            by,
            bz,
            hardIronX,
            hardIronY,
            hardIronZ,
            timestamp,
            accuracy
        )

        // check
        verify(exactly = 1) {
            measurementListener.onMeasurement(
                bx,
                by,
                bz,
                hardIronX,
                hardIronY,
                hardIronZ,
                timestamp,
                accuracy
            )
        }
    }

    @Test
    fun onAccuracyChanged_whenUnreliableAndNoListener_setsResultAsUnreliable() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        // check default value
        val unreliable1: Boolean? = detector.getPrivateProperty("unreliable")
        requireNotNull(unreliable1)
        assertFalse(unreliable1)

        val accuracyChangedListener: SensorCollector.OnAccuracyChangedListener? =
            detector.getPrivateProperty("internalAccuracyChangedListener")
        requireNotNull(accuracyChangedListener)

        accuracyChangedListener.onAccuracyChanged(SensorAccuracy.UNRELIABLE)

        // check
        val unreliable2: Boolean? = detector.getPrivateProperty("unreliable")
        requireNotNull(unreliable2)
        assertTrue(unreliable2)
    }

    @Test
    fun onAccuracyChanged_whenUnreliableAndListener_setsResultAsUnreliableAndNotifies() {
        val errorListener = mockk<MagnetometerIntervalDetector.OnErrorListener>(relaxUnitFun = true)
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context, errorListener = errorListener)

        // check default value
        val unreliable1: Boolean? = detector.getPrivateProperty("unreliable")
        requireNotNull(unreliable1)
        assertFalse(unreliable1)

        val accuracyChangedListener: SensorCollector.OnAccuracyChangedListener? =
            detector.getPrivateProperty("internalAccuracyChangedListener")
        requireNotNull(accuracyChangedListener)

        accuracyChangedListener.onAccuracyChanged(SensorAccuracy.UNRELIABLE)

        // check
        val unreliable2: Boolean? = detector.getPrivateProperty("unreliable")
        requireNotNull(unreliable2)
        assertTrue(unreliable2)
        verify(exactly = 1) {
            errorListener.onError(
                detector,
                MagnetometerIntervalDetector.ErrorReason.UNRELIABLE_SENSOR
            )
        }
    }

    @Test
    fun onAccuracyChanged_whenNotUnreliable_makesNoAction() {
        val errorListener = mockk<MagnetometerIntervalDetector.OnErrorListener>(relaxUnitFun = true)
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context, errorListener = errorListener)

        // check default value
        val unreliable1: Boolean? = detector.getPrivateProperty("unreliable")
        requireNotNull(unreliable1)
        assertFalse(unreliable1)

        val accuracyChangedListener: SensorCollector.OnAccuracyChangedListener? =
            detector.getPrivateProperty("internalAccuracyChangedListener")
        requireNotNull(accuracyChangedListener)

        accuracyChangedListener.onAccuracyChanged(SensorAccuracy.HIGH)

        // check
        val unreliable2: Boolean? = detector.getPrivateProperty("unreliable")
        requireNotNull(unreliable2)
        assertFalse(unreliable2)
        verify(exactly = 0) {
            errorListener.onError(any(), any())
        }
    }

    @Test
    fun onAccuracyChanged_whenUnreliableListener_notifiesAccuracyChange() {
        val accuracyChangedListener =
            mockk<SensorCollector.OnAccuracyChangedListener>(relaxUnitFun = true)
        val errorListener = mockk<MagnetometerIntervalDetector.OnErrorListener>(relaxUnitFun = true)
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(
            context,
            errorListener = errorListener,
            accuracyChangedListener = accuracyChangedListener
        )

        // check default value
        val unreliable1: Boolean? = detector.getPrivateProperty("unreliable")
        requireNotNull(unreliable1)
        assertFalse(unreliable1)

        val internalAccuracyChangedListener: SensorCollector.OnAccuracyChangedListener? =
            detector.getPrivateProperty("internalAccuracyChangedListener")
        requireNotNull(internalAccuracyChangedListener)

        internalAccuracyChangedListener.onAccuracyChanged(SensorAccuracy.HIGH)

        // check
        val unreliable2: Boolean? = detector.getPrivateProperty("unreliable")
        requireNotNull(unreliable2)
        assertFalse(unreliable2)
        verify(exactly = 0) {
            errorListener.onError(any(), any())
        }
        verify(exactly = 1) { accuracyChangedListener.onAccuracyChanged(SensorAccuracy.HIGH) }
    }

    @Test
    fun baseNoiseLevel_whenNotInitialized_returnsNull() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        val initialized: Boolean? = detector.getPrivateProperty("initialized")
        requireNotNull(initialized)
        assertFalse(initialized)

        assertNull(detector.baseNoiseLevel)
    }

    @Test
    fun baseNoiseLevel_whenInitialized_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        val internalDetector: MagneticFluxDensityTriadStaticIntervalDetector? =
            detector.getPrivateProperty("internalDetector")
        requireNotNull(internalDetector)

        val randomizer = UniformRandomizer()
        val baseNoiseLevel1 = randomizer.nextDouble()
        val internalDetectorSpy = spyk(internalDetector)
        every { internalDetectorSpy.baseNoiseLevel }.returns(baseNoiseLevel1)
        detector.setPrivateProperty("internalDetector", internalDetectorSpy)

        detector.setPrivateProperty("initialized", true)

        val baseNoiseLevel2 = detector.baseNoiseLevel
        requireNotNull(baseNoiseLevel2)
        assertEquals(baseNoiseLevel1, baseNoiseLevel2, 0.0)
    }

    @Test
    fun baseNoiseLevelAsMagneticFluxDensity_whenNotInitialized_returnsNull() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        val initialized: Boolean? = detector.getPrivateProperty("initialized")
        requireNotNull(initialized)
        assertFalse(initialized)

        assertNull(detector.baseNoiseLevelAsMagneticFluxDensity)
    }

    @Test
    fun baseNoiseLevelAsMagneticFluxDensity_whenInitialized_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        val internalDetector: MagneticFluxDensityTriadStaticIntervalDetector? =
            detector.getPrivateProperty("internalDetector")
        requireNotNull(internalDetector)

        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        val baseNoiseLevel1 = MagneticFluxDensity(value, MagneticFluxDensityUnit.TESLA)
        val internalDetectorSpy = spyk(internalDetector)
        every { internalDetectorSpy.baseNoiseLevelAsMeasurement }.returns(baseNoiseLevel1)
        detector.setPrivateProperty("internalDetector", internalDetectorSpy)

        detector.setPrivateProperty("initialized", true)

        val baseNoiseLevel2 = detector.baseNoiseLevelAsMagneticFluxDensity
        requireNotNull(baseNoiseLevel2)
        assertEquals(baseNoiseLevel1, baseNoiseLevel2)
    }

    @Test
    fun getBaseNoiseLevelAsMagneticFluxDensity_whenNotInitialized_returnsFalse() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        val initialized: Boolean? = detector.getPrivateProperty("initialized")
        requireNotNull(initialized)
        assertFalse(initialized)

        val result = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(detector.getBaseNoiseLevelAsMagneticFluxDensity(result))
    }

    @Test
    fun getBaseNoiseLevelAsMagneticFluxDensity_whenInitialized_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        val internalDetector: MagneticFluxDensityTriadStaticIntervalDetector? =
            detector.getPrivateProperty("internalDetector")
        requireNotNull(internalDetector)
        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        val internalDetectorSpy = spyk(internalDetector)
        every { internalDetectorSpy.getBaseNoiseLevelAsMeasurement(any()) }.answers { answer ->
            val result = answer.invocation.args[0] as MagneticFluxDensity
            result.value = value
            result.unit = MagneticFluxDensityUnit.TESLA
        }
        detector.setPrivateProperty("internalDetector", internalDetectorSpy)

        detector.setPrivateProperty("initialized", true)

        val result = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertTrue(detector.getBaseNoiseLevelAsMagneticFluxDensity(result))
        assertEquals(value, result.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, result.unit)
    }

    @Test
    fun baseNoiseLevelPsd_whenNotInitialized_returnsNull() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        val initialized: Boolean? = detector.getPrivateProperty("initialized")
        requireNotNull(initialized)
        assertFalse(initialized)

        assertNull(detector.baseNoiseLevelPsd)
    }

    @Test
    fun baseNoiseLevelPsd_whenInitialized_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        val internalDetector: MagneticFluxDensityTriadStaticIntervalDetector? =
            detector.getPrivateProperty("internalDetector")
        requireNotNull(internalDetector)

        val randomizer = UniformRandomizer()
        val baseNoiseLevelPsd1 = randomizer.nextDouble()
        val internalDetectorSpy = spyk(internalDetector)
        every { internalDetectorSpy.baseNoiseLevelPsd }.returns(baseNoiseLevelPsd1)
        detector.setPrivateProperty("internalDetector", internalDetectorSpy)

        detector.setPrivateProperty("initialized", true)

        val baseNoiseLevelPsd2 = detector.baseNoiseLevelPsd
        requireNotNull(baseNoiseLevelPsd2)
        assertEquals(baseNoiseLevelPsd1, baseNoiseLevelPsd2, 0.0)
    }

    @Test
    fun baseNoiseLevelRootPsd_whenNotInitialized_returnsNull() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        val initialized: Boolean? = detector.getPrivateProperty("initialized")
        requireNotNull(initialized)
        assertFalse(initialized)

        assertNull(detector.baseNoiseLevelRootPsd)
    }

    @Test
    fun baseNoiseLevelRootPsd_whenInitialized_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        val internalDetector: MagneticFluxDensityTriadStaticIntervalDetector? =
            detector.getPrivateProperty("internalDetector")
        requireNotNull(internalDetector)

        val randomizer = UniformRandomizer()
        val baseNoiseLevelRootPsd1 = randomizer.nextDouble()
        val internalDetectorSpy = spyk(internalDetector)
        every { internalDetectorSpy.baseNoiseLevelRootPsd }.returns(baseNoiseLevelRootPsd1)
        detector.setPrivateProperty("internalDetector", internalDetectorSpy)

        detector.setPrivateProperty("initialized", true)

        val baseNoiseLevelRootPsd2 = detector.baseNoiseLevelRootPsd
        requireNotNull(baseNoiseLevelRootPsd2)
        assertEquals(baseNoiseLevelRootPsd1, baseNoiseLevelRootPsd2, 0.0)
    }

    @Test
    fun threshold_whenNotInitialized_returnsNull() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        val initialized: Boolean? = detector.getPrivateProperty("initialized")
        requireNotNull(initialized)
        assertFalse(initialized)

        assertNull(detector.threshold)
    }

    @Test
    fun threshold_whenInitialized_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        val internalDetector: MagneticFluxDensityTriadStaticIntervalDetector? =
            detector.getPrivateProperty("internalDetector")
        requireNotNull(internalDetector)

        val randomizer = UniformRandomizer()
        val threshold1 = randomizer.nextDouble()
        val internalDetectorSpy = spyk(internalDetector)
        every { internalDetectorSpy.threshold }.returns(threshold1)
        detector.setPrivateProperty("internalDetector", internalDetectorSpy)

        detector.setPrivateProperty("initialized", true)

        val threshold2 = detector.threshold
        requireNotNull(threshold2)
        assertEquals(threshold1, threshold2, 0.0)
    }

    @Test
    fun thresholdAsMagneticFluxDensity_whenNotInitialized_returnsNull() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        val initialized: Boolean? = detector.getPrivateProperty("initialized")
        requireNotNull(initialized)
        assertFalse(initialized)

        assertNull(detector.thresholdAsMagneticFluxDensity)
    }

    @Test
    fun thresholdAsMagneticFluxDensity_whenInitialized_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        val internalDetector: MagneticFluxDensityTriadStaticIntervalDetector? =
            detector.getPrivateProperty("internalDetector")
        requireNotNull(internalDetector)

        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        val threshold1 = MagneticFluxDensity(value, MagneticFluxDensityUnit.TESLA)
        val internalDetectorSpy = spyk(internalDetector)
        every { internalDetectorSpy.thresholdAsMeasurement }.returns(threshold1)
        detector.setPrivateProperty("internalDetector", internalDetectorSpy)

        detector.setPrivateProperty("initialized", true)

        val threshold2 = detector.thresholdAsMagneticFluxDensity
        requireNotNull(threshold2)
        assertEquals(threshold1, threshold2)
    }

    @Test
    fun getThresholdAsMeasurement_whenNotInitialized_returnsFalse() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        val initialized: Boolean? = detector.getPrivateProperty("initialized")
        requireNotNull(initialized)
        assertFalse(initialized)

        val result = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(detector.getThresholdAsMeasurement(result))
    }

    @Test
    fun getThresholdAsMeasurement_whenInitialized_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        val internalDetector: MagneticFluxDensityTriadStaticIntervalDetector? =
            detector.getPrivateProperty("internalDetector")
        requireNotNull(internalDetector)

        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        val internalDetectorSpy = spyk(internalDetector)
        every { internalDetectorSpy.getThresholdAsMeasurement(any()) }.answers { answer ->
            val result = answer.invocation.args[0] as MagneticFluxDensity
            result.value = value
            result.unit = MagneticFluxDensityUnit.TESLA
        }
        detector.setPrivateProperty("internalDetector", internalDetectorSpy)

        detector.setPrivateProperty("initialized", true)

        val result = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertTrue(detector.getThresholdAsMeasurement(result))
        assertEquals(value, result.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, result.unit)
    }

    @Test
    fun accumulatedAvgX_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        val internalDetector: MagneticFluxDensityTriadStaticIntervalDetector? =
            detector.getPrivateProperty("internalDetector")
        requireNotNull(internalDetector)

        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        val internalDetectorSpy = spyk(internalDetector)
        every { internalDetectorSpy.accumulatedAvgX }.returns(value)
        detector.setPrivateProperty("internalDetector", internalDetectorSpy)

        assertEquals(value, detector.accumulatedAvgX, 0.0)
    }

    @Test
    fun accumulatedAvgXAsMagneticFluxDensity_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        val internalDetector: MagneticFluxDensityTriadStaticIntervalDetector? =
            detector.getPrivateProperty("internalDetector")
        requireNotNull(internalDetector)

        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        val b = MagneticFluxDensity(value, MagneticFluxDensityUnit.TESLA)
        val internalDetectorSpy = spyk(internalDetector)
        every { internalDetectorSpy.accumulatedAvgXAsMeasurement }.returns(b)
        detector.setPrivateProperty("internalDetector", internalDetectorSpy)

        assertSame(b, detector.accumulatedAvgXAsMagneticFluxDensity)
    }

    @Test
    fun getAccumulatedAvgXAsMagneticFluxDensity_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        val internalDetector: MagneticFluxDensityTriadStaticIntervalDetector? =
            detector.getPrivateProperty("internalDetector")
        requireNotNull(internalDetector)

        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        val result = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        val internalDetectorSpy = spyk(internalDetector)
        every { internalDetectorSpy.getAccumulatedAvgXAsMeasurement(any()) }.answers { answer ->
            val avgX = answer.invocation.args[0] as MagneticFluxDensity
            avgX.value = value
            avgX.unit = MagneticFluxDensityUnit.TESLA
        }
        detector.setPrivateProperty("internalDetector", internalDetectorSpy)

        detector.getAccumulatedAvgXAsMagneticFluxDensity(result)

        assertEquals(value, result.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, result.unit)
    }

    @Test
    fun accumulatedAvgY_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        val internalDetector: MagneticFluxDensityTriadStaticIntervalDetector? =
            detector.getPrivateProperty("internalDetector")
        requireNotNull(internalDetector)

        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        val internalDetectorSpy = spyk(internalDetector)
        every { internalDetectorSpy.accumulatedAvgY }.returns(value)
        detector.setPrivateProperty("internalDetector", internalDetectorSpy)

        assertEquals(value, detector.accumulatedAvgY, 0.0)
    }

    @Test
    fun accumulatedAvgYAsMagneticFluxDensity_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        val internalDetector: MagneticFluxDensityTriadStaticIntervalDetector? =
            detector.getPrivateProperty("internalDetector")
        requireNotNull(internalDetector)

        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        val b = MagneticFluxDensity(value, MagneticFluxDensityUnit.TESLA)
        val internalDetectorSpy = spyk(internalDetector)
        every { internalDetectorSpy.accumulatedAvgYAsMeasurement }.returns(b)
        detector.setPrivateProperty("internalDetector", internalDetectorSpy)

        assertSame(b, detector.accumulatedAvgYAsMagneticFluxDensity)
    }

    @Test
    fun getAccumulatedAvgYAsMagneticFluxDensity_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        val internalDetector: MagneticFluxDensityTriadStaticIntervalDetector? =
            detector.getPrivateProperty("internalDetector")
        requireNotNull(internalDetector)

        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        val result = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        val internalDetectorSpy = spyk(internalDetector)
        every { internalDetectorSpy.getAccumulatedAvgYAsMeasurement(any()) }.answers { answer ->
            val avgY = answer.invocation.args[0] as MagneticFluxDensity
            avgY.value = value
            avgY.unit = MagneticFluxDensityUnit.TESLA
        }
        detector.setPrivateProperty("internalDetector", internalDetectorSpy)

        detector.getAccumulatedAvgYAsMagneticFluxDensity(result)

        assertEquals(value, result.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, result.unit)
    }

    @Test
    fun accumulatedAvgZ_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        val internalDetector: MagneticFluxDensityTriadStaticIntervalDetector? =
            detector.getPrivateProperty("internalDetector")
        requireNotNull(internalDetector)

        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        val internalDetectorSpy = spyk(internalDetector)
        every { internalDetectorSpy.accumulatedAvgZ }.returns(value)
        detector.setPrivateProperty("internalDetector", internalDetectorSpy)

        assertEquals(value, detector.accumulatedAvgZ, 0.0)
    }

    @Test
    fun accumulatedAvgZAsMagneticFluxDensity_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        val internalDetector: MagneticFluxDensityTriadStaticIntervalDetector? =
            detector.getPrivateProperty("internalDetector")
        requireNotNull(internalDetector)

        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        val b = MagneticFluxDensity(value, MagneticFluxDensityUnit.TESLA)
        val internalDetectorSpy = spyk(internalDetector)
        every { internalDetectorSpy.accumulatedAvgZAsMeasurement }.returns(b)
        detector.setPrivateProperty("internalDetector", internalDetectorSpy)

        assertSame(b, detector.accumulatedAvgZAsMagneticFluxDensity)
    }

    @Test
    fun getAccumulatedAvgZAsMagneticFluxDensity_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        val internalDetector: MagneticFluxDensityTriadStaticIntervalDetector? =
            detector.getPrivateProperty("internalDetector")
        requireNotNull(internalDetector)

        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        val result = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        val internalDetectorSpy = spyk(internalDetector)
        every { internalDetectorSpy.getAccumulatedAvgZAsMeasurement(any()) }.answers { answer ->
            val avgZ = answer.invocation.args[0] as MagneticFluxDensity
            avgZ.value = value
            avgZ.unit = MagneticFluxDensityUnit.TESLA
        }
        detector.setPrivateProperty("internalDetector", internalDetectorSpy)

        detector.getAccumulatedAvgZAsMagneticFluxDensity(result)

        assertEquals(value, result.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, result.unit)
    }

    @Test
    fun accumulatedAvgTriad_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        val internalDetector: MagneticFluxDensityTriadStaticIntervalDetector? =
            detector.getPrivateProperty("internalDetector")
        requireNotNull(internalDetector)

        val randomizer = UniformRandomizer()
        val valueX = randomizer.nextDouble()
        val valueY = randomizer.nextDouble()
        val valueZ = randomizer.nextDouble()
        val triad = MagneticFluxDensityTriad(MagneticFluxDensityUnit.TESLA, valueX, valueY, valueZ)
        val internalDetectorSpy = spyk(internalDetector)
        every { internalDetectorSpy.accumulatedAvgTriad }.returns(triad)
        detector.setPrivateProperty("internalDetector", internalDetectorSpy)

        assertSame(triad, detector.accumulatedAvgTriad)
    }

    @Test
    fun getAccumulatedAvgTriad_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        val internalDetector: MagneticFluxDensityTriadStaticIntervalDetector? =
            detector.getPrivateProperty("internalDetector")
        requireNotNull(internalDetector)

        val randomizer = UniformRandomizer()
        val valueX = randomizer.nextDouble()
        val valueY = randomizer.nextDouble()
        val valueZ = randomizer.nextDouble()
        val result = MagneticFluxDensityTriad(MagneticFluxDensityUnit.TESLA)
        val internalDetectorSpy = spyk(internalDetector)
        every { internalDetectorSpy.getAccumulatedAvgTriad(any()) }.answers { answer ->
            val triad = answer.invocation.args[0] as MagneticFluxDensityTriad
            triad.setValueCoordinatesAndUnit(valueX, valueY, valueZ, MagneticFluxDensityUnit.TESLA)
        }
        detector.setPrivateProperty("internalDetector", internalDetectorSpy)

        detector.getAccumulatedAvgTriad(result)

        assertEquals(valueX, result.valueX, 0.0)
        assertEquals(valueY, result.valueY, 0.0)
        assertEquals(valueZ, result.valueZ, 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, result.unit)
    }

    @Test
    fun accumulatedStdX_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        val internalDetector: MagneticFluxDensityTriadStaticIntervalDetector? =
            detector.getPrivateProperty("internalDetector")
        requireNotNull(internalDetector)

        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        val internalDetectorSpy = spyk(internalDetector)
        every { internalDetectorSpy.accumulatedStdX }.returns(value)
        detector.setPrivateProperty("internalDetector", internalDetectorSpy)

        assertEquals(value, detector.accumulatedStdX, 0.0)
    }

    @Test
    fun accumulatedStdXAsMagneticFluxDensity_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        val internalDetector: MagneticFluxDensityTriadStaticIntervalDetector? =
            detector.getPrivateProperty("internalDetector")
        requireNotNull(internalDetector)

        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        val b = MagneticFluxDensity(value, MagneticFluxDensityUnit.TESLA)
        val internalDetectorSpy = spyk(internalDetector)
        every { internalDetectorSpy.accumulatedStdXAsMeasurement }.returns(b)
        detector.setPrivateProperty("internalDetector", internalDetectorSpy)

        assertSame(b, detector.accumulatedStdXAsMagneticFluxDensity)
    }

    @Test
    fun getAccumulatedStdXAsMagneticFluxDensity_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        val internalDetector: MagneticFluxDensityTriadStaticIntervalDetector? =
            detector.getPrivateProperty("internalDetector")
        requireNotNull(internalDetector)

        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        val result = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        val internalDetectorSpy = spyk(internalDetector)
        every { internalDetectorSpy.getAccumulatedStdXAsMeasurement(any()) }.answers { answer ->
            val stdX = answer.invocation.args[0] as MagneticFluxDensity
            stdX.value = value
            stdX.unit = MagneticFluxDensityUnit.TESLA
        }
        detector.setPrivateProperty("internalDetector", internalDetectorSpy)

        detector.getAccumulatedStdXAsMagneticFluxDensity(result)

        assertEquals(value, result.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, result.unit)
    }

    @Test
    fun accumulatedStdY_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        val internalDetector: MagneticFluxDensityTriadStaticIntervalDetector? =
            detector.getPrivateProperty("internalDetector")
        requireNotNull(internalDetector)

        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        val internalDetectorSpy = spyk(internalDetector)
        every { internalDetectorSpy.accumulatedStdY }.returns(value)
        detector.setPrivateProperty("internalDetector", internalDetectorSpy)

        assertEquals(value, detector.accumulatedStdY, 0.0)
    }

    @Test
    fun accumulatedStdYAsMagneticFluxDensity_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        val internalDetector: MagneticFluxDensityTriadStaticIntervalDetector? =
            detector.getPrivateProperty("internalDetector")
        requireNotNull(internalDetector)

        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        val b = MagneticFluxDensity(value, MagneticFluxDensityUnit.TESLA)
        val internalDetectorSpy = spyk(internalDetector)
        every { internalDetectorSpy.accumulatedStdYAsMeasurement }.returns(b)
        detector.setPrivateProperty("internalDetector", internalDetectorSpy)

        assertSame(b, detector.accumulatedStdYAsMagneticFluxDensity)
    }

    @Test
    fun getAccumulatedStdYAsMagneticFluxDensity_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        val internalDetector: MagneticFluxDensityTriadStaticIntervalDetector? =
            detector.getPrivateProperty("internalDetector")
        requireNotNull(internalDetector)

        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        val result = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        val internalDetectorSpy = spyk(internalDetector)
        every { internalDetectorSpy.getAccumulatedStdYAsMeasurement(any()) }.answers { answer ->
            val stdY = answer.invocation.args[0] as MagneticFluxDensity
            stdY.value = value
            stdY.unit = MagneticFluxDensityUnit.TESLA
        }
        detector.setPrivateProperty("internalDetector", internalDetectorSpy)

        detector.getAccumulatedStdYAsMagneticFluxDensity(result)

        assertEquals(value, result.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, result.unit)
    }

    @Test
    fun accumulatedStdZ_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        val internalDetector: MagneticFluxDensityTriadStaticIntervalDetector? =
            detector.getPrivateProperty("internalDetector")
        requireNotNull(internalDetector)

        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        val internalDetectorSpy = spyk(internalDetector)
        every { internalDetectorSpy.accumulatedStdZ }.returns(value)
        detector.setPrivateProperty("internalDetector", internalDetectorSpy)

        assertEquals(value, detector.accumulatedStdZ, 0.0)
    }

    @Test
    fun accumulatedStdZAsMagneticFluxDensity_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        val internalDetector: MagneticFluxDensityTriadStaticIntervalDetector? =
            detector.getPrivateProperty("internalDetector")
        requireNotNull(internalDetector)

        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        val b = MagneticFluxDensity(value, MagneticFluxDensityUnit.TESLA)
        val internalDetectorSpy = spyk(internalDetector)
        every { internalDetectorSpy.accumulatedStdZAsMeasurement }.returns(b)
        detector.setPrivateProperty("internalDetector", internalDetectorSpy)

        assertSame(b, detector.accumulatedStdZAsMagneticFluxDensity)
    }

    @Test
    fun getAccumulatedStdZAsMagneticFluxDensity_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        val internalDetector: MagneticFluxDensityTriadStaticIntervalDetector? =
            detector.getPrivateProperty("internalDetector")
        requireNotNull(internalDetector)

        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        val result = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        val internalDetectorSpy = spyk(internalDetector)
        every { internalDetectorSpy.getAccumulatedStdZAsMeasurement(any()) }.answers { answer ->
            val stdZ = answer.invocation.args[0] as MagneticFluxDensity
            stdZ.value = value
            stdZ.unit = MagneticFluxDensityUnit.TESLA
        }
        detector.setPrivateProperty("internalDetector", internalDetectorSpy)

        detector.getAccumulatedStdZAsMagneticFluxDensity(result)

        assertEquals(value, result.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, result.unit)
    }

    @Test
    fun accumulatedStdTriad_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        val internalDetector: MagneticFluxDensityTriadStaticIntervalDetector? =
            detector.getPrivateProperty("internalDetector")
        requireNotNull(internalDetector)

        val randomizer = UniformRandomizer()
        val valueX = randomizer.nextDouble()
        val valueY = randomizer.nextDouble()
        val valueZ = randomizer.nextDouble()
        val triad = MagneticFluxDensityTriad(MagneticFluxDensityUnit.TESLA, valueX, valueY, valueZ)
        val internalDetectorSpy = spyk(internalDetector)
        every { internalDetectorSpy.accumulatedStdTriad }.returns(triad)
        detector.setPrivateProperty("internalDetector", internalDetectorSpy)

        assertSame(triad, detector.accumulatedStdTriad)
    }

    @Test
    fun getAccumulatedStdTriad_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        val internalDetector: MagneticFluxDensityTriadStaticIntervalDetector? =
            detector.getPrivateProperty("internalDetector")
        requireNotNull(internalDetector)

        val randomizer = UniformRandomizer()
        val valueX = randomizer.nextDouble()
        val valueY = randomizer.nextDouble()
        val valueZ = randomizer.nextDouble()
        val result = MagneticFluxDensityTriad(MagneticFluxDensityUnit.TESLA)
        val internalDetectorSpy = spyk(internalDetector)
        every { internalDetectorSpy.getAccumulatedStdTriad(any()) }.answers { answer ->
            val triad = answer.invocation.args[0] as MagneticFluxDensityTriad
            triad.setValueCoordinatesAndUnit(valueX, valueY, valueZ, MagneticFluxDensityUnit.TESLA)
        }
        detector.setPrivateProperty("internalDetector", internalDetectorSpy)

        detector.getAccumulatedStdTriad(result)

        assertEquals(valueX, result.valueX, 0.0)
        assertEquals(valueY, result.valueY, 0.0)
        assertEquals(valueZ, result.valueZ, 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, result.unit)
    }

    @Test
    fun instantaneousAvgX_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        val internalDetector: MagneticFluxDensityTriadStaticIntervalDetector? =
            detector.getPrivateProperty("internalDetector")
        requireNotNull(internalDetector)

        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        val internalDetectorSpy = spyk(internalDetector)
        every { internalDetectorSpy.instantaneousAvgX }.returns(value)
        detector.setPrivateProperty("internalDetector", internalDetectorSpy)

        assertEquals(value, detector.instantaneousAvgX, 0.0)
    }

    @Test
    fun instantaneousAvgXAsMagneticFluxDensity_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        val internalDetector: MagneticFluxDensityTriadStaticIntervalDetector? =
            detector.getPrivateProperty("internalDetector")
        requireNotNull(internalDetector)

        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        val b = MagneticFluxDensity(value, MagneticFluxDensityUnit.TESLA)
        val internalDetectorSpy = spyk(internalDetector)
        every { internalDetectorSpy.instantaneousAvgXAsMeasurement }.returns(b)
        detector.setPrivateProperty("internalDetector", internalDetectorSpy)

        assertSame(b, detector.instantaneousAvgXAsMagneticFluxDensity)
    }

    @Test
    fun getInstantaneousAvgXAsMagneticFluxDensity_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        val internalDetector: MagneticFluxDensityTriadStaticIntervalDetector? =
            detector.getPrivateProperty("internalDetector")
        requireNotNull(internalDetector)

        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        val result = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        val internalDetectorSpy = spyk(internalDetector)
        every { internalDetectorSpy.getInstantaneousAvgXAsMeasurement(any()) }.answers { answer ->
            val avgX = answer.invocation.args[0] as MagneticFluxDensity
            avgX.value = value
            avgX.unit = MagneticFluxDensityUnit.TESLA
        }
        detector.setPrivateProperty("internalDetector", internalDetectorSpy)

        detector.getInstantaneousAvgXAsMagneticFluxDensity(result)

        assertEquals(value, result.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, result.unit)
    }

    @Test
    fun instantaneousAvgY_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        val internalDetector: MagneticFluxDensityTriadStaticIntervalDetector? =
            detector.getPrivateProperty("internalDetector")
        requireNotNull(internalDetector)

        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        val internalDetectorSpy = spyk(internalDetector)
        every { internalDetectorSpy.instantaneousAvgY }.returns(value)
        detector.setPrivateProperty("internalDetector", internalDetectorSpy)

        assertEquals(value, detector.instantaneousAvgY, 0.0)
    }

    @Test
    fun instantaneousAvgYAsMagneticFluxDensity_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        val internalDetector: MagneticFluxDensityTriadStaticIntervalDetector? =
            detector.getPrivateProperty("internalDetector")
        requireNotNull(internalDetector)

        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        val b = MagneticFluxDensity(value, MagneticFluxDensityUnit.TESLA)
        val internalDetectorSpy = spyk(internalDetector)
        every { internalDetectorSpy.instantaneousAvgYAsMeasurement }.returns(b)
        detector.setPrivateProperty("internalDetector", internalDetectorSpy)

        assertSame(b, detector.instantaneousAvgYAsMagneticFluxDensity)
    }

    @Test
    fun getInstantaneousAvgYAsMagneticFluxDensity_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        val internalDetector: MagneticFluxDensityTriadStaticIntervalDetector? =
            detector.getPrivateProperty("internalDetector")
        requireNotNull(internalDetector)

        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        val result = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        val internalDetectorSpy = spyk(internalDetector)
        every { internalDetectorSpy.getInstantaneousAvgYAsMeasurement(any()) }.answers { answer ->
            val avgY = answer.invocation.args[0] as MagneticFluxDensity
            avgY.value = value
            avgY.unit = MagneticFluxDensityUnit.TESLA
        }
        detector.setPrivateProperty("internalDetector", internalDetectorSpy)

        detector.getInstantaneousAvgYAsMagneticFluxDensity(result)

        assertEquals(value, result.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, result.unit)
    }

    @Test
    fun instantaneousAvgZ_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        val internalDetector: MagneticFluxDensityTriadStaticIntervalDetector? =
            detector.getPrivateProperty("internalDetector")
        requireNotNull(internalDetector)

        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        val internalDetectorSpy = spyk(internalDetector)
        every { internalDetectorSpy.instantaneousAvgZ }.returns(value)
        detector.setPrivateProperty("internalDetector", internalDetectorSpy)

        assertEquals(value, detector.instantaneousAvgZ, 0.0)
    }

    @Test
    fun instantaneousAvgZAsMagneticFluxDensity_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        val internalDetector: MagneticFluxDensityTriadStaticIntervalDetector? =
            detector.getPrivateProperty("internalDetector")
        requireNotNull(internalDetector)

        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        val b = MagneticFluxDensity(value, MagneticFluxDensityUnit.TESLA)
        val internalDetectorSpy = spyk(internalDetector)
        every { internalDetectorSpy.instantaneousAvgZAsMeasurement }.returns(b)
        detector.setPrivateProperty("internalDetector", internalDetectorSpy)

        assertSame(b, detector.instantaneousAvgZAsMagneticFluxDensity)
    }

    @Test
    fun getInstantaneousAvgZAsMagneticFluxDensity_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        val internalDetector: MagneticFluxDensityTriadStaticIntervalDetector? =
            detector.getPrivateProperty("internalDetector")
        requireNotNull(internalDetector)

        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        val result = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        val internalDetectorSpy = spyk(internalDetector)
        every { internalDetectorSpy.getInstantaneousAvgZAsMeasurement(any()) }.answers { answer ->
            val avgZ = answer.invocation.args[0] as MagneticFluxDensity
            avgZ.value = value
            avgZ.unit = MagneticFluxDensityUnit.TESLA
        }
        detector.setPrivateProperty("internalDetector", internalDetectorSpy)

        detector.getInstantaneousAvgZAsMagneticFluxDensity(result)

        assertEquals(value, result.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, result.unit)
    }

    @Test
    fun instantaneousAvgTriad_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        val internalDetector: MagneticFluxDensityTriadStaticIntervalDetector? =
            detector.getPrivateProperty("internalDetector")
        requireNotNull(internalDetector)

        val randomizer = UniformRandomizer()
        val valueX = randomizer.nextDouble()
        val valueY = randomizer.nextDouble()
        val valueZ = randomizer.nextDouble()
        val internalDetectorSpy = spyk(internalDetector)
        val triad = MagneticFluxDensityTriad(MagneticFluxDensityUnit.TESLA, valueX, valueY, valueZ)
        every { internalDetectorSpy.instantaneousAvgTriad }.returns(triad)
        detector.setPrivateProperty("internalDetector", internalDetectorSpy)

        assertSame(triad, detector.instantaneousAvgTriad)
    }

    @Test
    fun getInstantaneousAvgTriad_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        val internalDetector: MagneticFluxDensityTriadStaticIntervalDetector? =
            detector.getPrivateProperty("internalDetector")
        requireNotNull(internalDetector)

        val randomizer = UniformRandomizer()
        val valueX = randomizer.nextDouble()
        val valueY = randomizer.nextDouble()
        val valueZ = randomizer.nextDouble()
        val result = MagneticFluxDensityTriad()
        val internalDetectorSpy = spyk(internalDetector)
        every { internalDetectorSpy.getInstantaneousAvgTriad(any()) }.answers { answer ->
            val triad = answer.invocation.args[0] as MagneticFluxDensityTriad
            triad.setValueCoordinatesAndUnit(valueX, valueY, valueZ, MagneticFluxDensityUnit.TESLA)
        }
        detector.setPrivateProperty("internalDetector", internalDetectorSpy)

        detector.getInstantaneousAvgTriad(result)

        assertEquals(valueX, result.valueX, 0.0)
        assertEquals(valueY, result.valueY, 0.0)
        assertEquals(valueZ, result.valueZ, 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, result.unit)
    }

    @Test
    fun instantaneousStdX_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        val internalDetector: MagneticFluxDensityTriadStaticIntervalDetector? =
            detector.getPrivateProperty("internalDetector")
        requireNotNull(internalDetector)

        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        val internalDetectorSpy = spyk(internalDetector)
        every { internalDetectorSpy.instantaneousStdX }.returns(value)
        detector.setPrivateProperty("internalDetector", internalDetectorSpy)

        assertEquals(value, detector.instantaneousStdX, 0.0)
    }

    @Test
    fun instantaneousStdXAsMagneticFluxDensity_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        val internalDetector: MagneticFluxDensityTriadStaticIntervalDetector? =
            detector.getPrivateProperty("internalDetector")
        requireNotNull(internalDetector)

        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        val b = MagneticFluxDensity(value, MagneticFluxDensityUnit.TESLA)
        val internalDetectorSpy = spyk(internalDetector)
        every { internalDetectorSpy.instantaneousStdXAsMeasurement }.returns(b)
        detector.setPrivateProperty("internalDetector", internalDetectorSpy)

        assertSame(b, detector.instantaneousStdXAsMagneticFluxDensity)
    }

    @Test
    fun getInstantaneousStdXAsMagneticFluxDensity_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        val internalDetector: MagneticFluxDensityTriadStaticIntervalDetector? =
            detector.getPrivateProperty("internalDetector")
        requireNotNull(internalDetector)

        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        val result = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        val internalDetectorSpy = spyk(internalDetector)
        every { internalDetectorSpy.getInstantaneousStdXAsMeasurement(any()) }.answers { answer ->
            val stdX = answer.invocation.args[0] as MagneticFluxDensity
            stdX.value = value
            stdX.unit = MagneticFluxDensityUnit.TESLA
        }
        detector.setPrivateProperty("internalDetector", internalDetectorSpy)

        detector.getInstantaneousStdXAsMagneticFluxDensity(result)

        assertEquals(value, result.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, result.unit)
    }

    @Test
    fun instantaneousStdY_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        val internalDetector: MagneticFluxDensityTriadStaticIntervalDetector? =
            detector.getPrivateProperty("internalDetector")
        requireNotNull(internalDetector)

        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        val internalDetectorSpy = spyk(internalDetector)
        every { internalDetectorSpy.instantaneousStdY }.returns(value)
        detector.setPrivateProperty("internalDetector", internalDetectorSpy)

        assertEquals(value, detector.instantaneousStdY, 0.0)
    }

    @Test
    fun instantaneousStdYAsMagneticFluxDensity_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        val internalDetector: MagneticFluxDensityTriadStaticIntervalDetector? =
            detector.getPrivateProperty("internalDetector")
        requireNotNull(internalDetector)

        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        val b = MagneticFluxDensity(value, MagneticFluxDensityUnit.TESLA)
        val internalDetectorSpy = spyk(internalDetector)
        every { internalDetectorSpy.instantaneousStdYAsMeasurement }.returns(b)
        detector.setPrivateProperty("internalDetector", internalDetectorSpy)

        assertSame(b, detector.instantaneousStdYAsMagneticFluxDensity)
    }

    @Test
    fun getInstantaneousStdYAsMagneticFluxDensity_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        val internalDetector: MagneticFluxDensityTriadStaticIntervalDetector? =
            detector.getPrivateProperty("internalDetector")
        requireNotNull(internalDetector)

        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        val result = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        val internalDetectorSpy = spyk(internalDetector)
        every { internalDetectorSpy.getInstantaneousStdYAsMeasurement(any()) }.answers { answer ->
            val stdY = answer.invocation.args[0] as MagneticFluxDensity
            stdY.value = value
            stdY.unit = MagneticFluxDensityUnit.TESLA
        }
        detector.setPrivateProperty("internalDetector", internalDetectorSpy)

        detector.getInstantaneousStdYAsMagneticFluxDensity(result)

        assertEquals(value, result.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, result.unit)
    }

    @Test
    fun instantaneousStdZ_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        val internalDetector: MagneticFluxDensityTriadStaticIntervalDetector? =
            detector.getPrivateProperty("internalDetector")
        requireNotNull(internalDetector)

        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        val internalDetectorSpy = spyk(internalDetector)
        every { internalDetectorSpy.instantaneousStdZ }.returns(value)
        detector.setPrivateProperty("internalDetector", internalDetectorSpy)
    }

    @Test
    fun instantaneousStdZAsMagneticFluxDensity_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        val internalDetector: MagneticFluxDensityTriadStaticIntervalDetector? =
            detector.getPrivateProperty("internalDetector")
        requireNotNull(internalDetector)

        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        val b = MagneticFluxDensity(value, MagneticFluxDensityUnit.TESLA)
        val internalDetectorSpy = spyk(internalDetector)
        every { internalDetectorSpy.instantaneousStdZAsMeasurement }.returns(b)
        detector.setPrivateProperty("internalDetector", internalDetectorSpy)

        assertSame(b, detector.instantaneousStdZAsMagneticFluxDensity)
    }

    @Test
    fun getInstantaneousStdZAsMagneticFluxDensity_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        val internalDetector: MagneticFluxDensityTriadStaticIntervalDetector? =
            detector.getPrivateProperty("internalDetector")
        requireNotNull(internalDetector)

        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        val result = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        val internalDetectorSpy = spyk(internalDetector)
        every { internalDetectorSpy.getInstantaneousStdZAsMeasurement(any()) }.answers { answer ->
            val stdZ = answer.invocation.args[0] as MagneticFluxDensity
            stdZ.value = value
            stdZ.unit = MagneticFluxDensityUnit.TESLA
        }
        detector.setPrivateProperty("internalDetector", internalDetectorSpy)

        detector.getInstantaneousStdZAsMagneticFluxDensity(result)

        assertEquals(value, result.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, result.unit)
    }

    @Test
    fun instantaneousStdTriad_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        val internalDetector: MagneticFluxDensityTriadStaticIntervalDetector? =
            detector.getPrivateProperty("internalDetector")
        requireNotNull(internalDetector)

        val randomizer = UniformRandomizer()
        val valueX = randomizer.nextDouble()
        val valueY = randomizer.nextDouble()
        val valueZ = randomizer.nextDouble()
        val internalDetectorSpy = spyk(internalDetector)
        val triad = MagneticFluxDensityTriad(MagneticFluxDensityUnit.TESLA, valueX, valueY, valueZ)
        every { internalDetectorSpy.instantaneousStdTriad }.returns(triad)
        detector.setPrivateProperty("internalDetector", internalDetectorSpy)

        assertSame(triad, detector.instantaneousStdTriad)
    }

    @Test
    fun getInstantaneousStdTriad_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        val internalDetector: MagneticFluxDensityTriadStaticIntervalDetector? =
            detector.getPrivateProperty("internalDetector")
        requireNotNull(internalDetector)

        val randomizer = UniformRandomizer()
        val valueX = randomizer.nextDouble()
        val valueY = randomizer.nextDouble()
        val valueZ = randomizer.nextDouble()
        val result = MagneticFluxDensityTriad()
        val internalDetectorSpy = spyk(internalDetector)
        every { internalDetectorSpy.getInstantaneousStdTriad(any()) }.answers { answer ->
            val triad = answer.invocation.args[0] as MagneticFluxDensityTriad
            triad.setValueCoordinatesAndUnit(valueX, valueY, valueZ, MagneticFluxDensityUnit.TESLA)
        }
        detector.setPrivateProperty("internalDetector", internalDetectorSpy)

        detector.getInstantaneousStdTriad(result)

        assertEquals(valueX, result.valueX, 0.0)
        assertEquals(valueY, result.valueY, 0.0)
        assertEquals(valueZ, result.valueZ, 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, result.unit)
    }

    @Test
    fun averageTimeInterval_whenNotInitialized_returnsNull() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        val initialized: Boolean? = detector.getPrivateProperty("initialized")
        requireNotNull(initialized)
        assertFalse(initialized)

        assertNull(detector.averageTimeInterval)
    }

    @Test
    fun averageTimeInterval_whenInitialized_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        val timeIntervalEstimator: TimeIntervalEstimator? =
            detector.getPrivateProperty("timeIntervalEstimator")
        requireNotNull(timeIntervalEstimator)

        val randomizer = UniformRandomizer()
        val averageTimeInterval1 = randomizer.nextDouble()
        val timeIntervalEstimatorSpy = spyk(timeIntervalEstimator)
        every { timeIntervalEstimatorSpy.averageTimeInterval }.returns(averageTimeInterval1)
        detector.setPrivateProperty("timeIntervalEstimator", timeIntervalEstimatorSpy)

        detector.setPrivateProperty("initialized", true)

        val averageTimeInterval2 = detector.averageTimeInterval
        requireNotNull(averageTimeInterval2)
        assertEquals(averageTimeInterval1, averageTimeInterval2, 0.0)
    }

    @Test
    fun averageTimeIntervalAsTime_whenNotInitialized_returnsNull() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        val initialized: Boolean? = detector.getPrivateProperty("initialized")
        requireNotNull(initialized)
        assertFalse(initialized)

        assertNull(detector.averageTimeIntervalAsTime)
    }

    @Test
    fun averageTimeIntervalAsTime_whenInitialized_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        val timeIntervalEstimator: TimeIntervalEstimator? =
            detector.getPrivateProperty("timeIntervalEstimator")
        requireNotNull(timeIntervalEstimator)

        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        val averageTimeInterval1 = Time(value, TimeUnit.SECOND)
        val timeIntervalEstimatorSpy = spyk(timeIntervalEstimator)
        every { timeIntervalEstimatorSpy.averageTimeIntervalAsTime }.returns(averageTimeInterval1)
        detector.setPrivateProperty("timeIntervalEstimator", timeIntervalEstimatorSpy)

        detector.setPrivateProperty("initialized", true)

        val averageTimeInterval2 = detector.averageTimeIntervalAsTime
        requireNotNull(averageTimeInterval2)
        assertSame(averageTimeInterval1, averageTimeInterval2)
    }

    @Test
    fun getAverageTimeIntervalAsTime_whenNotInitialized_returnsFalse() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        val initialized: Boolean? = detector.getPrivateProperty("initialized")
        requireNotNull(initialized)
        assertFalse(initialized)

        val result = Time(0.0, TimeUnit.SECOND)
        assertFalse(detector.getAverageTimeIntervalAsTime(result))
    }

    @Test
    fun getAverageTimeIntervalAsTime_whenInitialized_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        val timeIntervalEstimator: TimeIntervalEstimator? =
            detector.getPrivateProperty("timeIntervalEstimator")
        requireNotNull(timeIntervalEstimator)

        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        val timeIntervalEstimatorSpy = spyk(timeIntervalEstimator)
        every { timeIntervalEstimatorSpy.getAverageTimeIntervalAsTime(any()) }.answers { answer ->
            val result = answer.invocation.args[0] as Time
            result.value = value
            result.unit = TimeUnit.SECOND
        }
        detector.setPrivateProperty("timeIntervalEstimator", timeIntervalEstimatorSpy)

        detector.setPrivateProperty("initialized", true)

        val result = Time(1.0, TimeUnit.NANOSECOND)
        assertTrue(detector.getAverageTimeIntervalAsTime(result))

        assertEquals(value, result.value.toDouble(), 0.0)
        assertEquals(TimeUnit.SECOND, result.unit)
    }

    @Test
    fun timeIntervalVariance_whenNotInitialized_returnsNull() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        val initialized: Boolean? = detector.getPrivateProperty("initialized")
        requireNotNull(initialized)
        assertFalse(initialized)

        assertNull(detector.timeIntervalVariance)
    }

    @Test
    fun timeIntervalVariance_whenInitialized_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        val timeIntervalEstimator: TimeIntervalEstimator? =
            detector.getPrivateProperty("timeIntervalEstimator")
        requireNotNull(timeIntervalEstimator)

        val randomizer = UniformRandomizer()
        val timeIntervalVariance1 = randomizer.nextDouble()
        val timeIntervalEstimatorSpy = spyk(timeIntervalEstimator)
        every { timeIntervalEstimatorSpy.timeIntervalVariance }.returns(timeIntervalVariance1)
        detector.setPrivateProperty("timeIntervalEstimator", timeIntervalEstimatorSpy)

        detector.setPrivateProperty("initialized", true)

        val timeIntervalVariance2 = detector.timeIntervalVariance
        requireNotNull(timeIntervalVariance2)
        assertEquals(timeIntervalVariance1, timeIntervalVariance2, 0.0)
    }

    @Test
    fun timeIntervalStandardDeviation_whenNotInitialized_returnsNull() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        val initialized: Boolean? = detector.getPrivateProperty("initialized")
        requireNotNull(initialized)
        assertFalse(initialized)

        assertNull(detector.timeIntervalStandardDeviation)
    }

    @Test
    fun timeIntervalStandardDeviation_whenInitialized_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        val timeIntervalEstimator: TimeIntervalEstimator? =
            detector.getPrivateProperty("timeIntervalEstimator")
        requireNotNull(timeIntervalEstimator)

        val randomizer = UniformRandomizer()
        val timeIntervalStandardDeviation1 = randomizer.nextDouble()
        val timeIntervalEstimatorSpy = spyk(timeIntervalEstimator)
        every { timeIntervalEstimatorSpy.timeIntervalStandardDeviation }.returns(
            timeIntervalStandardDeviation1
        )
        detector.setPrivateProperty("timeIntervalEstimator", timeIntervalEstimatorSpy)

        detector.setPrivateProperty("initialized", true)

        val timeIntervalStandardDeviation2 = detector.timeIntervalStandardDeviation
        requireNotNull(timeIntervalStandardDeviation2)
        assertEquals(timeIntervalStandardDeviation1, timeIntervalStandardDeviation2, 0.0)
    }

    @Test
    fun timeIntervalStandardDeviationAsTime_whenNotInitialized_returnsNull() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        val initialized: Boolean? = detector.getPrivateProperty("initialized")
        requireNotNull(initialized)
        assertFalse(initialized)

        assertNull(detector.timeIntervalStandardDeviationAsTime)
    }

    @Test
    fun timeIntervalStandardDeviationAsTime_whenInitialized_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        val timeIntervalEstimator: TimeIntervalEstimator? =
            detector.getPrivateProperty("timeIntervalEstimator")
        requireNotNull(timeIntervalEstimator)

        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        val timeIntervalStd1 = Time(value, TimeUnit.SECOND)
        val timeIntervalEstimatorSpy = spyk(timeIntervalEstimator)
        every { timeIntervalEstimatorSpy.timeIntervalStandardDeviationAsTime }.returns(
            timeIntervalStd1
        )
        detector.setPrivateProperty("timeIntervalEstimator", timeIntervalEstimatorSpy)

        detector.setPrivateProperty("initialized", true)

        val timeIntervalStd2 = detector.timeIntervalStandardDeviationAsTime
        requireNotNull(timeIntervalStd2)
        assertSame(timeIntervalStd1, timeIntervalStd2)
    }

    @Test
    fun getTimeIntervalStandardDeviationAstime_whenNotInitialized_returnsFalse() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        val initialized: Boolean? = detector.getPrivateProperty("initialized")
        requireNotNull(initialized)
        assertFalse(initialized)

        val result = Time(0.0, TimeUnit.SECOND)
        assertFalse(detector.getTimeIntervalStandardDeviationAsTime((result)))
    }

    @Test
    fun getTimeIntervalStandardDeviationAsTime_whenInitialized_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        val timeIntervalEstimator: TimeIntervalEstimator? =
            detector.getPrivateProperty("timeIntervalEstimator")
        requireNotNull(timeIntervalEstimator)

        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        val timeIntervalEstimatorSpy = spyk(timeIntervalEstimator)
        every { timeIntervalEstimatorSpy.getTimeIntervalStandardDeviationAsTime(any()) }.answers { answer ->
            val result = answer.invocation.args[0] as Time
            result.value = value
            result.unit = TimeUnit.SECOND
        }
        detector.setPrivateProperty("timeIntervalEstimator", timeIntervalEstimatorSpy)

        detector.setPrivateProperty("initialized", true)

        val result = Time(1.0, TimeUnit.NANOSECOND)
        assertTrue(detector.getTimeIntervalStandardDeviationAsTime(result))

        assertEquals(value, result.value.toDouble(), 0.0)
        assertEquals(TimeUnit.SECOND, result.unit)
    }

    @Test
    fun status_whenUnreliable_returnsFailed() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        detector.setPrivateProperty("unreliable", true)

        assertEquals(MagnetometerIntervalDetector.Status.FAILED, detector.status)
    }

    @Test
    fun status_whenReliableAndIdle__returnsIdle() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        val unreliable: Boolean? = detector.getPrivateProperty("unreliable")
        requireNotNull(unreliable)
        assertFalse(unreliable)

        val internalDetector: MagneticFluxDensityTriadStaticIntervalDetector? =
            detector.getPrivateProperty("internalDetector")
        requireNotNull(internalDetector)
        val internalDetectorSpy = spyk(internalDetector)
        every { internalDetectorSpy.status }.returns(TriadStaticIntervalDetector.Status.IDLE)
        detector.setPrivateProperty("internalDetector", internalDetectorSpy)

        assertEquals(MagnetometerIntervalDetector.Status.IDLE, detector.status)
    }

    @Test
    fun status_whenReliableAndInitializing_returnsIdle() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        val unreliable: Boolean? = detector.getPrivateProperty("unreliable")
        requireNotNull(unreliable)

        val internalDetector: MagneticFluxDensityTriadStaticIntervalDetector? =
            detector.getPrivateProperty("internalDetector")
        requireNotNull(internalDetector)
        val internalDetectorSpy = spyk(internalDetector)
        every { internalDetectorSpy.status }.returns(TriadStaticIntervalDetector.Status.INITIALIZING)
        detector.setPrivateProperty("internalDetector", internalDetectorSpy)

        assertEquals(MagnetometerIntervalDetector.Status.INITIALIZING, detector.status)
    }

    @Test
    fun status_whenReliableAndInitializationCompleted_returnsIdle() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        val unreliable: Boolean? = detector.getPrivateProperty("unreliable")
        requireNotNull(unreliable)
        assertFalse(unreliable)

        val internalDetector: MagneticFluxDensityTriadStaticIntervalDetector? =
            detector.getPrivateProperty("internalDetector")
        requireNotNull(internalDetector)
        val internalDetectorSpy = spyk(internalDetector)
        every { internalDetectorSpy.status }
            .returns(TriadStaticIntervalDetector.Status.INITIALIZATION_COMPLETED)
        detector.setPrivateProperty("internalDetector", internalDetectorSpy)

        assertEquals(MagnetometerIntervalDetector.Status.INITIALIZATION_COMPLETED, detector.status)
    }

    @Test
    fun status_whenReliableAndStaticInterval_returnsIdle() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        val unreliable: Boolean? = detector.getPrivateProperty("unreliable")
        requireNotNull(unreliable)
        assertFalse(unreliable)

        val internalDetector: MagneticFluxDensityTriadStaticIntervalDetector? =
            detector.getPrivateProperty("internalDetector")
        requireNotNull(internalDetector)
        val internalDetectorSpy = spyk(internalDetector)
        every { internalDetectorSpy.status }
            .returns(TriadStaticIntervalDetector.Status.STATIC_INTERVAL)
        detector.setPrivateProperty("internalDetector", internalDetectorSpy)

        assertEquals(MagnetometerIntervalDetector.Status.STATIC_INTERVAL, detector.status)
    }

    @Test
    fun status_whenReliableAndDynamicInterval_returnsIdle() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        val unreliable: Boolean? = detector.getPrivateProperty("unreliable")
        requireNotNull(unreliable)
        assertFalse(unreliable)

        val internalDetector: MagneticFluxDensityTriadStaticIntervalDetector? =
            detector.getPrivateProperty("internalDetector")
        requireNotNull(internalDetector)
        val internalDetectorSpy = spyk(internalDetector)
        every { internalDetectorSpy.status }
            .returns(TriadStaticIntervalDetector.Status.DYNAMIC_INTERVAL)
        detector.setPrivateProperty("internalDetector", internalDetectorSpy)

        assertEquals(MagnetometerIntervalDetector.Status.DYNAMIC_INTERVAL, detector.status)
    }

    @Test
    fun status_whenReliableAndFailed_returnsIdle() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        val unreliable: Boolean? = detector.getPrivateProperty("unreliable")
        requireNotNull(unreliable)
        assertFalse(unreliable)

        val internalDetector: MagneticFluxDensityTriadStaticIntervalDetector? =
            detector.getPrivateProperty("internalDetector")
        requireNotNull(internalDetector)
        val internalDetectorSpy = spyk(internalDetector)
        every { internalDetectorSpy.status }.returns(TriadStaticIntervalDetector.Status.FAILED)
        detector.setPrivateProperty("internalDetector", internalDetectorSpy)

        assertEquals(MagnetometerIntervalDetector.Status.FAILED, detector.status)
    }

    @Test
    fun status_whenReliableAndNoStatus_returnsIdle() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val detector = MagnetometerIntervalDetector(context)

        val unreliable: Boolean? = detector.getPrivateProperty("unreliable")
        requireNotNull(unreliable)
        assertFalse(unreliable)

        val internalDetector: MagneticFluxDensityTriadStaticIntervalDetector? =
            detector.getPrivateProperty("internalDetector")
        requireNotNull(internalDetector)
        val internalDetectorSpy = spyk(internalDetector)
        every { internalDetectorSpy.status }.returns(null)
        detector.setPrivateProperty("internalDetector", internalDetectorSpy)

        assertEquals(MagnetometerIntervalDetector.Status.IDLE, detector.status)
    }

    @Test
    fun onInitializationStarted_whenNoListener_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val intervalDetector = MagnetometerIntervalDetector(context)

        val internalDetectorListener: MagneticFluxDensityTriadStaticIntervalDetectorListener? =
            intervalDetector.getPrivateProperty("internalDetectorListener")
        requireNotNull(internalDetectorListener)

        val internalDetector: MagneticFluxDensityTriadStaticIntervalDetector? =
            intervalDetector.getPrivateProperty("internalDetector")
        requireNotNull(internalDetector)
        internalDetectorListener.onInitializationStarted(internalDetector)
    }

    @Test
    fun onInitializationStarted_whenListener_notifies() {
        val initializationStartedListener =
            mockk<MagnetometerIntervalDetector.OnInitializationStartedListener>(relaxUnitFun = true)
        val context = ApplicationProvider.getApplicationContext<Context>()
        val intervalDetector = MagnetometerIntervalDetector(
            context,
            initializationStartedListener = initializationStartedListener
        )

        val internalDetectorListener: MagneticFluxDensityTriadStaticIntervalDetectorListener? =
            intervalDetector.getPrivateProperty("internalDetectorListener")
        requireNotNull(internalDetectorListener)

        val internalDetector: MagneticFluxDensityTriadStaticIntervalDetector? =
            intervalDetector.getPrivateProperty("internalDetector")
        requireNotNull(internalDetector)
        internalDetectorListener.onInitializationStarted(internalDetector)

        verify(exactly = 1) { initializationStartedListener.onInitializationStarted(intervalDetector) }
    }

    @Test
    fun onInitializationCompleted_whenNoListener_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val intervalDetector = MagnetometerIntervalDetector(context)

        val internalDetectorListener: MagneticFluxDensityTriadStaticIntervalDetectorListener? =
            intervalDetector.getPrivateProperty("internalDetectorListener")
        requireNotNull(internalDetectorListener)

        val internalDetector: MagneticFluxDensityTriadStaticIntervalDetector? =
            intervalDetector.getPrivateProperty("internalDetector")
        requireNotNull(internalDetector)
        val randomizer = UniformRandomizer()
        val baseNoiseLevel = randomizer.nextDouble()
        internalDetectorListener.onInitializationCompleted(internalDetector, baseNoiseLevel)
    }

    @Test
    fun onInitializationCompleted_whenListener_notifies() {
        val initializationCompletedListener =
            mockk<MagnetometerIntervalDetector.OnInitializationCompletedListener>(relaxUnitFun = true)
        val context = ApplicationProvider.getApplicationContext<Context>()
        val intervalDetector = MagnetometerIntervalDetector(
            context,
            initializationCompletedListener = initializationCompletedListener
        )

        val internalDetectorListener: MagneticFluxDensityTriadStaticIntervalDetectorListener? =
            intervalDetector.getPrivateProperty("internalDetectorListener")
        requireNotNull(internalDetectorListener)

        val internalDetector: MagneticFluxDensityTriadStaticIntervalDetector? =
            intervalDetector.getPrivateProperty("internalDetector")
        requireNotNull(internalDetector)
        val randomizer = UniformRandomizer()
        val baseNoiseLevel = randomizer.nextDouble()
        internalDetectorListener.onInitializationCompleted(internalDetector, baseNoiseLevel)

        verify(exactly = 1) {
            initializationCompletedListener.onInitializationCompleted(
                intervalDetector,
                baseNoiseLevel
            )
        }
    }

    @Test
    fun onError_whenNoListener_stops() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val intervalDetector = MagnetometerIntervalDetector(context)

        val internalDetectorListener: MagneticFluxDensityTriadStaticIntervalDetectorListener? =
            intervalDetector.getPrivateProperty("internalDetectorListener")
        requireNotNull(internalDetectorListener)

        val internalDetector: MagneticFluxDensityTriadStaticIntervalDetector? =
            intervalDetector.getPrivateProperty("internalDetector")
        requireNotNull(internalDetector)
        val randomizer = UniformRandomizer()
        val accumulatedNoiseLevel = randomizer.nextDouble()
        val instantaneousNoiseLevel = randomizer.nextDouble()

        val collector: MagnetometerSensorCollector? =
            intervalDetector.getPrivateProperty("collector")
        requireNotNull(collector)
        val collectorSpy = spyk(collector)
        intervalDetector.setPrivateProperty("collector", collectorSpy)

        internalDetectorListener.onError(
            internalDetector,
            accumulatedNoiseLevel,
            instantaneousNoiseLevel,
            TriadStaticIntervalDetector.ErrorReason.SUDDEN_EXCESSIVE_MOVEMENT_DETECTED
        )

        verify(exactly = 1) { collectorSpy.stop() }
    }

    @Test
    fun onError_whenListener_stopsAndNotifies() {
        val errorListener = mockk<MagnetometerIntervalDetector.OnErrorListener>(relaxUnitFun = true)
        val context = ApplicationProvider.getApplicationContext<Context>()
        val intervalDetector = MagnetometerIntervalDetector(context, errorListener = errorListener)

        val internalDetectorListener: MagneticFluxDensityTriadStaticIntervalDetectorListener? =
            intervalDetector.getPrivateProperty("internalDetectorListener")
        requireNotNull(internalDetectorListener)

        val internalDetector: MagneticFluxDensityTriadStaticIntervalDetector? =
            intervalDetector.getPrivateProperty("internalDetector")
        requireNotNull(internalDetector)
        val randomizer = UniformRandomizer()
        val accumulatedNoiseLevel = randomizer.nextDouble()
        val instantaneousNoiseLevel = randomizer.nextDouble()

        val collector: MagnetometerSensorCollector? =
            intervalDetector.getPrivateProperty("collector")
        requireNotNull(collector)
        val collectorSpy = spyk(collector)
        intervalDetector.setPrivateProperty("collector", collectorSpy)

        internalDetectorListener.onError(
            internalDetector,
            accumulatedNoiseLevel,
            instantaneousNoiseLevel,
            TriadStaticIntervalDetector.ErrorReason.SUDDEN_EXCESSIVE_MOVEMENT_DETECTED
        )

        verify(exactly = 1) { collectorSpy.stop() }
        verify(exactly = 1) {
            errorListener.onError(
                intervalDetector,
                MagnetometerIntervalDetector.ErrorReason.SUDDEN_EXCESSIVE_MOVEMENT_DETECTED_DURING_INITIALIZATION
            )
        }
    }

    @Test
    fun onStaticIntervalDetected_whenNoListener_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val intervalDetector = MagnetometerIntervalDetector(context)

        val internalDetectorListener: MagneticFluxDensityTriadStaticIntervalDetectorListener? =
            intervalDetector.getPrivateProperty("internalDetectorListener")
        requireNotNull(internalDetectorListener)

        val internalDetector: MagneticFluxDensityTriadStaticIntervalDetector? =
            intervalDetector.getPrivateProperty("internalDetector")
        requireNotNull(internalDetector)
        val randomizer = UniformRandomizer()
        val instantaneousAvgX = randomizer.nextDouble()
        val instantaneousAvgY = randomizer.nextDouble()
        val instantaneousAvgZ = randomizer.nextDouble()
        val instantaneousStdX = randomizer.nextDouble()
        val instantaneousStdY = randomizer.nextDouble()
        val instantaneousStdZ = randomizer.nextDouble()
        internalDetectorListener.onStaticIntervalDetected(
            internalDetector,
            instantaneousAvgX,
            instantaneousAvgY,
            instantaneousAvgZ,
            instantaneousStdX,
            instantaneousStdY,
            instantaneousStdZ
        )
    }

    @Test
    fun onStaticIntervalDetected_whenListener_notifies() {
        val staticIntervalDetectedListener =
            mockk<MagnetometerIntervalDetector.OnStaticIntervalDetectedListener>(relaxUnitFun = true)
        val context = ApplicationProvider.getApplicationContext<Context>()
        val intervalDetector = MagnetometerIntervalDetector(
            context,
            staticIntervalDetectedListener = staticIntervalDetectedListener
        )

        val internalDetectorListener: MagneticFluxDensityTriadStaticIntervalDetectorListener? =
            intervalDetector.getPrivateProperty("internalDetectorListener")
        requireNotNull(internalDetectorListener)

        val internalDetector: MagneticFluxDensityTriadStaticIntervalDetector? =
            intervalDetector.getPrivateProperty("internalDetector")
        requireNotNull(internalDetector)

        val randomizer = UniformRandomizer()
        val instantaneousAvgX = randomizer.nextDouble()
        val instantaneousAvgY = randomizer.nextDouble()
        val instantaneousAvgZ = randomizer.nextDouble()
        val instantaneousStdX = randomizer.nextDouble()
        val instantaneousStdY = randomizer.nextDouble()
        val instantaneousStdZ = randomizer.nextDouble()
        internalDetectorListener.onStaticIntervalDetected(
            internalDetector,
            instantaneousAvgX,
            instantaneousAvgY,
            instantaneousAvgZ,
            instantaneousStdX,
            instantaneousStdY,
            instantaneousStdZ
        )

        verify(exactly = 1) {
            staticIntervalDetectedListener.onStaticIntervalDetected(
                intervalDetector,
                instantaneousAvgX,
                instantaneousAvgY,
                instantaneousAvgZ,
                instantaneousStdX,
                instantaneousStdY,
                instantaneousStdZ
            )
        }
    }

    @Test
    fun onDynamicIntervalDetected_whenNoListener_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val intervalDetector = MagnetometerIntervalDetector(context)

        val internalDetectorListener: MagneticFluxDensityTriadStaticIntervalDetectorListener? =
            intervalDetector.getPrivateProperty("internalDetectorListener")
        requireNotNull(internalDetectorListener)

        val internalDetector: MagneticFluxDensityTriadStaticIntervalDetector? =
            intervalDetector.getPrivateProperty("internalDetector")
        requireNotNull(internalDetector)
        val randomizer = UniformRandomizer()
        val instantaneousAvgX = randomizer.nextDouble()
        val instantaneousAvgY = randomizer.nextDouble()
        val instantaneousAvgZ = randomizer.nextDouble()
        val instantaneousStdX = randomizer.nextDouble()
        val instantaneousStdY = randomizer.nextDouble()
        val instantaneousStdZ = randomizer.nextDouble()
        val accumulatedAvgX = randomizer.nextDouble()
        val accumulatedAvgY = randomizer.nextDouble()
        val accumulatedAvgZ = randomizer.nextDouble()
        val accumulatedStdX = randomizer.nextDouble()
        val accumulatedStdY = randomizer.nextDouble()
        val accumulatedStdZ = randomizer.nextDouble()
        internalDetectorListener.onDynamicIntervalDetected(
            internalDetector,
            instantaneousAvgX,
            instantaneousAvgY,
            instantaneousAvgZ,
            instantaneousStdX,
            instantaneousStdY,
            instantaneousStdZ,
            accumulatedAvgX,
            accumulatedAvgY,
            accumulatedAvgZ,
            accumulatedStdX,
            accumulatedStdY,
            accumulatedStdZ
        )
    }

    @Test
    fun onDynamicIntervalDetected_whenListener_notifies() {
        val dynamicIntervalDetectedListener =
            mockk<MagnetometerIntervalDetector.OnDynamicIntervalDetectedListener>(relaxUnitFun = true)
        val context = ApplicationProvider.getApplicationContext<Context>()
        val intervalDetector = MagnetometerIntervalDetector(
            context,
            dynamicIntervalDetectedListener = dynamicIntervalDetectedListener
        )

        val internalDetectorListener: MagneticFluxDensityTriadStaticIntervalDetectorListener? =
            intervalDetector.getPrivateProperty("internalDetectorListener")
        requireNotNull(internalDetectorListener)

        val internalDetector: MagneticFluxDensityTriadStaticIntervalDetector? =
            intervalDetector.getPrivateProperty("internalDetector")
        requireNotNull(internalDetector)
        val randomizer = UniformRandomizer()
        val instantaneousAvgX = randomizer.nextDouble()
        val instantaneousAvgY = randomizer.nextDouble()
        val instantaneousAvgZ = randomizer.nextDouble()
        val instantaneousStdX = randomizer.nextDouble()
        val instantaneousStdY = randomizer.nextDouble()
        val instantaneousStdZ = randomizer.nextDouble()
        val accumulatedAvgX = randomizer.nextDouble()
        val accumulatedAvgY = randomizer.nextDouble()
        val accumulatedAvgZ = randomizer.nextDouble()
        val accumulatedStdX = randomizer.nextDouble()
        val accumulatedStdY = randomizer.nextDouble()
        val accumulatedStdZ = randomizer.nextDouble()
        internalDetectorListener.onDynamicIntervalDetected(
            internalDetector,
            instantaneousAvgX,
            instantaneousAvgY,
            instantaneousAvgZ,
            instantaneousStdX,
            instantaneousStdY,
            instantaneousStdZ,
            accumulatedAvgX,
            accumulatedAvgY,
            accumulatedAvgZ,
            accumulatedStdX,
            accumulatedStdY,
            accumulatedStdZ
        )

        verify(exactly = 1) {
            dynamicIntervalDetectedListener.onDynamicIntervalDetected(
                intervalDetector,
                instantaneousAvgX,
                instantaneousAvgY,
                instantaneousAvgZ,
                instantaneousStdX,
                instantaneousStdY,
                instantaneousStdZ,
                accumulatedAvgX,
                accumulatedAvgY,
                accumulatedAvgZ,
                accumulatedStdX,
                accumulatedStdY,
                accumulatedStdZ
            )
        }
    }

    @Test
    fun onReset_whenNoListener_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val intervalDetector = MagnetometerIntervalDetector(context)

        val internalDetectorListener: MagneticFluxDensityTriadStaticIntervalDetectorListener? =
            intervalDetector.getPrivateProperty("internalDetectorListener")
        requireNotNull(internalDetectorListener)

        val internalDetector: MagneticFluxDensityTriadStaticIntervalDetector? =
            intervalDetector.getPrivateProperty("internalDetector")
        requireNotNull(internalDetector)

        internalDetectorListener.onReset(internalDetector)
    }

    @Test
    fun onReset_whenListener_notifies() {
        val resetListener = mockk<MagnetometerIntervalDetector.OnResetListener>(relaxUnitFun = true)
        val context = ApplicationProvider.getApplicationContext<Context>()
        val intervalDetector = MagnetometerIntervalDetector(context, resetListener = resetListener)

        val internalDetectorListener: MagneticFluxDensityTriadStaticIntervalDetectorListener? =
            intervalDetector.getPrivateProperty("internalDetectorListener")
        requireNotNull(internalDetectorListener)

        val internalDetector: MagneticFluxDensityTriadStaticIntervalDetector? =
            intervalDetector.getPrivateProperty("internalDetector")
        requireNotNull(internalDetector)

        internalDetectorListener.onReset(internalDetector)

        verify(exactly = 1) { resetListener.onReset(intervalDetector) }
    }

    private fun getMagneticFluxDensity(): BodyMagneticFluxDensity {
        val randomizer = UniformRandomizer()
        val latitude =
            Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES))
        val longitude =
            Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES))
        val height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT)
        val nedPosition = NEDPosition(latitude, longitude, height)

        val earthEstimator = WMMEarthMagneticFluxDensityEstimator()
        val earthB = earthEstimator.estimate(nedPosition, Date())
        val c = CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, FrameType.BODY_FRAME)

        return BodyMagneticFluxDensityEstimator.estimate(earthB, c)
    }

    private companion object {
        const val WINDOW_SIZE = 201

        const val INITIAL_STATIC_SAMPLES = 10000

        const val THRESHOLD_FACTOR = 3.0

        const val INSTANTANEOUS_NOISE_LEVEL_FACTOR = 3.0

        const val BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD = 100.0

        const val MIN_LATITUDE_DEGREES = -90.0
        const val MAX_LATITUDE_DEGREES = 90.0

        const val MIN_LONGITUDE_DEGREES = -180.0
        const val MAX_LONGITUDE_DEGREES = 180.0

        const val MIN_HEIGHT = -50.0
        const val MAX_HEIGHT = 400.0

        const val TIME_INTERVAL_MILLIS = 20L

        const val TIME_INTERVAL_SECONDS = 0.02

        const val MILLIS_TO_NANOS = 1000000L
    }
}