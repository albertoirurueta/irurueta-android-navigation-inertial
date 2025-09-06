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
package com.irurueta.android.navigation.inertial.calibration

import android.content.Context
import android.hardware.Sensor
import android.location.Location
import androidx.test.core.app.ApplicationProvider
import com.irurueta.algebra.Matrix
import com.irurueta.android.navigation.inertial.GravityHelper
import com.irurueta.android.navigation.inertial.collectors.*
import com.irurueta.android.testutils.getPrivateProperty
import com.irurueta.android.testutils.setPrivateProperty
import com.irurueta.navigation.inertial.BodyKinematics
import com.irurueta.navigation.inertial.calibration.*
import com.irurueta.navigation.inertial.calibration.intervals.TriadStaticIntervalDetector
import com.irurueta.navigation.inertial.calibration.intervals.thresholdfactor.DefaultAccelerometerQualityScoreMapper
import com.irurueta.navigation.inertial.calibration.intervals.thresholdfactor.DefaultGyroscopeQualityScoreMapper
import com.irurueta.navigation.inertial.calibration.intervals.thresholdfactor.DefaultMagnetometerQualityScoreMapper
import com.irurueta.navigation.inertial.wmm.WorldMagneticModel
import com.irurueta.numerical.robust.RobustEstimatorMethod
import com.irurueta.statistics.UniformRandomizer
import com.irurueta.units.*
//import io.mockk.*
//import io.mockk.impl.annotations.MockK
//import io.mockk.junit4.MockKRule
//import org.junit.After
import org.junit.Assert.*
import org.junit.Rule
import org.junit.Test
import org.junit.runner.RunWith
import org.mockito.Mock
import org.mockito.junit.MockitoJUnit
import org.mockito.junit.MockitoRule
import org.mockito.kotlin.any
import org.mockito.kotlin.doAnswer
import org.mockito.kotlin.doReturn
import org.mockito.kotlin.never
import org.mockito.kotlin.only
import org.mockito.kotlin.spy
import org.mockito.kotlin.times
import org.mockito.kotlin.verify
import org.mockito.kotlin.verifyNoInteractions
import org.mockito.kotlin.whenever
import org.robolectric.RobolectricTestRunner
import java.util.*
import kotlin.math.max

@RunWith(RobolectricTestRunner::class)
class StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2Test {

    @get:Rule
    val mockitoRule: MockitoRule = MockitoJUnit.rule()

//    @get:Rule
//    val mockkRule = MockKRule(this)

//    @MockK(relaxUnitFun = true)
    @Mock
    private lateinit var initializationStartedListener:
            StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationStartedListener<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2>

//    @MockK(relaxUnitFun = true)
    @Mock
    private lateinit var initializationCompletedListener:
            StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationCompletedListener<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2>

//    @MockK(relaxUnitFun = true)
    @Mock
    private lateinit var errorListener:
            StaticIntervalWithMeasurementGeneratorCalibrator.OnErrorListener<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2>

//    @MockK(relaxUnitFun = true)
    @Mock
    private lateinit var staticIntervalDetectedListener:
            StaticIntervalWithMeasurementGeneratorCalibrator.OnStaticIntervalDetectedListener<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2>

//    @MockK(relaxUnitFun = true)
    @Mock
    private lateinit var dynamicIntervalDetectedListener:
            StaticIntervalWithMeasurementGeneratorCalibrator.OnDynamicIntervalDetectedListener<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2>

//    @MockK(relaxUnitFun = true)
    @Mock
    private lateinit var staticIntervalSkippedListener:
            StaticIntervalWithMeasurementGeneratorCalibrator.OnStaticIntervalSkippedListener<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2>

//    @MockK(relaxUnitFun = true)
    @Mock
    private lateinit var dynamicIntervalSkippedListener:
            StaticIntervalWithMeasurementGeneratorCalibrator.OnDynamicIntervalSkippedListener<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2>

//    @MockK(relaxUnitFun = true)
    @Mock
    private lateinit var generatedAccelerometerMeasurementListener:
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2.OnGeneratedAccelerometerMeasurementListener

//    @MockK(relaxUnitFun = true)
    @Mock
    private lateinit var generatedGyroscopeMeasurementListener:
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2.OnGeneratedGyroscopeMeasurementListener

//    @MockK(relaxUnitFun = true)
    @Mock
    private lateinit var generatedMagnetometerMeasurementListener:
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2.OnGeneratedMagnetometerMeasurementListener

//    @MockK(relaxUnitFun = true)
    @Mock
    private lateinit var readyToSolveCalibrationListener:
            StaticIntervalWithMeasurementGeneratorCalibrator.OnReadyToSolveCalibrationListener<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2>

//    @MockK(relaxUnitFun = true)
    @Mock
    private lateinit var calibrationSolvingStartedListener:
            StaticIntervalWithMeasurementGeneratorCalibrator.OnCalibrationSolvingStartedListener<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2>

//    @MockK(relaxUnitFun = true)
    @Mock
    private lateinit var calibrationCompletedListener:
            StaticIntervalWithMeasurementGeneratorCalibrator.OnCalibrationCompletedListener<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2>

//    @MockK(relaxUnitFun = true)
    @Mock
    private lateinit var stoppedListener:
            StaticIntervalWithMeasurementGeneratorCalibrator.OnStoppedListener<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2>

//    @MockK(relaxUnitFun = true)
    @Mock
    private lateinit var unreliableGravityNormEstimationListener:
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2.OnUnreliableGravityEstimationListener

//    @MockK(relaxUnitFun = true)
    @Mock
    private lateinit var initialAccelerometerBiasAvailableListener:
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2.OnInitialAccelerometerBiasAvailableListener

//    @MockK(relaxUnitFun = true)
    @Mock
    private lateinit var initialGyroscopeBiasAvailableListener:
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2.OnInitialGyroscopeBiasAvailableListener

//    @MockK(relaxUnitFun = true)
    @Mock
    private lateinit var initialMagnetometerHardIronAvailableListener:
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2.OnInitialMagnetometerHardIronAvailableListener

//    @MockK(relaxUnitFun = true)
    @Mock
    private lateinit var accuracyChangedListener: SensorCollector.OnAccuracyChangedListener

//    @MockK(relaxUnitFun = true, relaxed = true)
    @Mock
    private lateinit var accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator

//    @MockK(relaxUnitFun = true, relaxed = true)
    @Mock
    private lateinit var magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator

//    @MockK
    @Mock
    private lateinit var location: Location

//    @MockK
    @Mock
    private lateinit var sensor: Sensor

//    @MockK
    @Mock
    private lateinit var matrix: Matrix

    /*@After
    fun tearDown() {
        unmockkAll()
        clearAllMocks()
        System.gc()
    }*/

    @Test
    fun constructor_whenContext_returnsDefaultValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default values
        assertSame(context, calibrator.context)
        assertNull(calibrator.location)
        assertNotNull(calibrator.timestamp)
        assertNull(calibrator.worldMagneticModel)
        assertTrue(calibrator.isInitialMagneticFluxDensityNormMeasured)
        assertTrue(calibrator.isGravityNormEstimated)
        assertEquals(
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            calibrator.accelerometerSensorType
        )
        assertEquals(
            GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
            calibrator.gyroscopeSensorType
        )
        assertEquals(
            MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED,
            calibrator.magnetometerSensorType
        )
        assertEquals(SensorDelay.FASTEST, calibrator.accelerometerSensorDelay)
        assertEquals(SensorDelay.FASTEST, calibrator.gyroscopeSensorDelay)
        assertEquals(SensorDelay.FASTEST, calibrator.magnetometerSensorDelay)
        assertTrue(calibrator.solveCalibrationWhenEnoughMeasurements)
        assertFalse(calibrator.isAccelerometerGroundTruthInitialBias)
        assertFalse(calibrator.isGyroscopeGroundTruthInitialBias)
        assertFalse(calibrator.isMagnetometerGroundTruthInitialHardIron)
        assertNull(calibrator.initializationStartedListener)
        assertNull(calibrator.initializationCompletedListener)
        assertNull(calibrator.errorListener)
        assertNull(calibrator.staticIntervalDetectedListener)
        assertNull(calibrator.dynamicIntervalDetectedListener)
        assertNull(calibrator.staticIntervalSkippedListener)
        assertNull(calibrator.dynamicIntervalSkippedListener)
        assertNull(calibrator.generatedAccelerometerMeasurementListener)
        assertNull(calibrator.generatedGyroscopeMeasurementListener)
        assertNull(calibrator.generatedMagnetometerMeasurementListener)
        assertNull(calibrator.readyToSolveCalibrationListener)
        assertNull(calibrator.calibrationSolvingStartedListener)
        assertNull(calibrator.calibrationCompletedListener)
        assertNull(calibrator.stoppedListener)
        assertNull(calibrator.unreliableGravityNormEstimationListener)
        assertNull(calibrator.initialAccelerometerBiasAvailableListener)
        assertNull(calibrator.initialGyroscopeBiasAvailableListener)
        assertNull(calibrator.initialMagnetometerHardIronAvailableListener)
        assertNull(calibrator.accuracyChangedListener)
        assertNotNull(calibrator.accelerometerQualityScoreMapper)
        assertNotNull(calibrator.gyroscopeQualityScoreMapper)
        assertNotNull(calibrator.magnetometerQualityScoreMapper)
        assertNull(calibrator.initialMagneticFluxDensityNorm)
        assertNull(calibrator.gravityNorm)
        assertFalse(calibrator.accelerometerResultUnreliable)
        assertNull(calibrator.accelerometerInitialBiasX)
        assertNull(calibrator.accelerometerInitialBiasY)
        assertNull(calibrator.accelerometerInitialBiasZ)
        assertNull(calibrator.accelerometerInitialBiasXAsMeasurement)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getAccelerometerInitialBiasXAsMeasurement(acceleration))
        assertNull(calibrator.accelerometerInitialBiasYAsMeasurement)
        assertFalse(calibrator.getAccelerometerInitialBiasYAsMeasurement(acceleration))
        assertNull(calibrator.accelerometerInitialBiasZAsMeasurement)
        assertFalse(calibrator.getAccelerometerInitialBiasZAsMeasurement(acceleration))
        assertNull(calibrator.accelerometerInitialBiasAsTriad)
        val accelerationTriad = AccelerationTriad()
        assertFalse(calibrator.getAccelerometerInitialBiasAsTriad(accelerationTriad))
        assertNull(calibrator.gyroscopeInitialBiasX)
        assertNull(calibrator.gyroscopeInitialBiasY)
        assertNull(calibrator.gyroscopeInitialBiasZ)
        assertNull(calibrator.gyroscopeInitialBiasXAsMeasurement)
        val angularSpeed = AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND)
        assertFalse(calibrator.getGyroscopeInitialBiasXAsMeasurement(angularSpeed))
        assertNull(calibrator.gyroscopeInitialBiasYAsMeasurement)
        assertFalse(calibrator.getGyroscopeInitialBiasYAsMeasurement(angularSpeed))
        assertNull(calibrator.gyroscopeInitialBiasZAsMeasurement)
        assertFalse(calibrator.getGyroscopeInitialBiasZAsMeasurement(angularSpeed))
        assertNull(calibrator.gyroscopeInitialBiasAsTriad)
        val angularSpeedTriad = AngularSpeedTriad()
        assertFalse(calibrator.getGyroscopeInitialBiasAsTriad(angularSpeedTriad))
        assertNull(calibrator.magnetometerInitialHardIronX)
        assertNull(calibrator.magnetometerInitialHardIronY)
        assertNull(calibrator.magnetometerInitialHardIronZ)
        assertNull(calibrator.magnetometerInitialHardIronXAsMeasurement)
        val b = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(calibrator.getMagnetometerInitialHardIronXAsMeasurement(b))
        assertNull(calibrator.magnetometerInitialHardIronYAsMeasurement)
        assertFalse(calibrator.getMagnetometerInitialHardIronYAsMeasurement(b))
        assertNull(calibrator.magnetometerInitialHardIronZAsMeasurement)
        assertFalse(calibrator.getMagnetometerInitialHardIronZAsMeasurement(b))
        assertNull(calibrator.magnetometerInitialHardIronAsTriad)
        val triad = MagneticFluxDensityTriad()
        assertFalse(calibrator.getMagnetometerInitialHardIronAsTriad(triad))
        assertNull(calibrator.accelerometerSensor)
        assertNull(calibrator.gyroscopeSensor)
        assertNull(calibrator.magnetometerSensor)
        assertNull(calibrator.gravitySensor)
        val accelerometerInitialMa1 = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        val accelerometerInitialMa2 = calibrator.accelerometerInitialMa
        assertEquals(accelerometerInitialMa1, accelerometerInitialMa2)
        val accelerometerInitialMa3 =
            Matrix.identity(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        calibrator.getAccelerometerInitialMa(accelerometerInitialMa3)
        assertEquals(accelerometerInitialMa1, accelerometerInitialMa3)
        assertEquals(0.0, calibrator.accelerometerInitialSx, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialSy, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialSz, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMxy, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMxz, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMyx, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMyz, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMzx, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMzy, 0.0)
        val gyroscopeInitialMg1 = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        val gyroscopeInitialMg2 = calibrator.gyroscopeInitialMg
        assertEquals(gyroscopeInitialMg1, gyroscopeInitialMg2)
        val gyroscopeInitialMg3 =
            Matrix.identity(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        calibrator.getGyroscopeInitialMg(gyroscopeInitialMg3)
        assertEquals(gyroscopeInitialMg1, gyroscopeInitialMg3)
        assertEquals(0.0, calibrator.gyroscopeInitialSx, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialSy, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialSz, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMxy, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMxz, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMyx, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMyz, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMzx, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMzy, 0.0)
        val gyroscopeInitialGg1 = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        val gyroscopeInitialGg2 = calibrator.gyroscopeInitialGg
        assertEquals(gyroscopeInitialGg1, gyroscopeInitialGg2)
        val mm1 = Matrix(MM_SIZE, MM_SIZE)
        assertEquals(mm1, calibrator.magnetometerInitialMm)
        val mm2 = Matrix.identity(MM_SIZE, MM_SIZE)
        calibrator.getMagnetometerInitialMm(mm2)
        assertEquals(mm1, mm2)
        assertEquals(0.0, calibrator.magnetometerInitialSx, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialSy, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialSz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMxy, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMxz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMyx, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMyz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMzx, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMzy, 0.0)
        assertFalse(calibrator.isAccelerometerCommonAxisUsed)
        assertFalse(calibrator.isGyroscopeCommonAxisUsed)
        assertFalse(calibrator.isMagnetometerCommonAxisUsed)
        assertFalse(calibrator.isGDependentCrossBiasesEstimated)
        assertEquals(
            StaticIntervalAccelerometerCalibrator.ACCELEROMETER_UNKNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.minimumRequiredAccelerometerMeasurements
        )
        assertEquals(
            StaticIntervalGyroscopeCalibrator.GYROSCOPE_UNKNOWN_BIAS_MINIMUM_SEQUENCES_GENERAL,
            calibrator.minimumRequiredGyroscopeMeasurements
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.MAGNETOMETER_UNKNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.minimumRequiredMagnetometerMeasurements
        )
        assertEquals(
            max(
                max(
                    calibrator.minimumRequiredAccelerometerMeasurements,
                    calibrator.minimumRequiredGyroscopeMeasurements
                ), calibrator.minimumRequiredMagnetometerMeasurements
            ), calibrator.minimumRequiredMeasurements
        )
        assertNull(calibrator.averageGravityNorm)
        assertNull(calibrator.averageGravityNormAsMeasurement)
        assertFalse(calibrator.getAverageGravityNormAsMeasurement(acceleration))
        assertNull(calibrator.gravityNormVariance)
        assertNull(calibrator.gravityNormStandardDeviation)
        assertNull(calibrator.gravityNormStandardDeviationAsMeasurement)
        assertFalse(calibrator.getGravityNormStandardDeviationAsMeasurement(acceleration))
        assertNull(calibrator.gravityPsd)
        assertNull(calibrator.gravityRootPsd)
        assertNull(calibrator.accelerometerRobustMethod)
        assertEquals(
            StaticIntervalAccelerometerCalibrator.ROBUST_DEFAULT_CONFIDENCE,
            calibrator.accelerometerRobustConfidence,
            0.0
        )
        assertEquals(
            StaticIntervalAccelerometerCalibrator.ROBUST_DEFAULT_MAX_ITERATIONS,
            calibrator.accelerometerRobustMaxIterations
        )
        assertEquals(
            calibrator.minimumRequiredAccelerometerMeasurements,
            calibrator.accelerometerRobustPreliminarySubsetSize
        )
        assertNull(calibrator.accelerometerRobustThreshold)
        assertEquals(
            StaticIntervalAccelerometerCalibrator.DEFAULT_ROBUST_THRESHOLD_FACTOR,
            calibrator.accelerometerRobustThresholdFactor,
            0.0
        )
        assertEquals(
            StaticIntervalAccelerometerCalibrator.DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR,
            calibrator.accelerometerRobustStopThresholdFactor,
            0.0
        )
        assertNull(calibrator.gyroscopeRobustMethod)
        assertEquals(
            StaticIntervalGyroscopeCalibrator.ROBUST_DEFAULT_CONFIDENCE,
            calibrator.gyroscopeRobustConfidence,
            0.0
        )
        assertEquals(
            StaticIntervalGyroscopeCalibrator.ROBUST_DEFAULT_MAX_ITERATIONS,
            calibrator.gyroscopeRobustMaxIterations
        )
        assertEquals(
            calibrator.minimumRequiredGyroscopeMeasurements,
            calibrator.gyroscopeRobustPreliminarySubsetSize
        )
        assertNull(calibrator.gyroscopeRobustThreshold)
        assertEquals(
            StaticIntervalGyroscopeCalibrator.DEFAULT_ROBUST_THRESHOLD_FACTOR,
            calibrator.gyroscopeRobustThresholdFactor,
            0.0
        )
        assertEquals(
            StaticIntervalGyroscopeCalibrator.DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR,
            calibrator.gyroscopeRobustStopThresholdFactor,
            0.0
        )
        assertNull(calibrator.magnetometerRobustMethod)
        assertEquals(
            SingleSensorStaticIntervalMagnetometerCalibrator.ROBUST_DEFAULT_CONFIDENCE,
            calibrator.magnetometerRobustConfidence,
            0.0
        )
        assertEquals(
            SingleSensorStaticIntervalMagnetometerCalibrator.ROBUST_DEFAULT_MAX_ITERATIONS,
            calibrator.magnetometerRobustMaxIterations
        )
        assertEquals(
            SingleSensorStaticIntervalMagnetometerCalibrator.UNKNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.magnetometerRobustPreliminarySubsetSize
        )
        assertNull(calibrator.magnetometerRobustThreshold)
        assertEquals(
            SingleSensorStaticIntervalMagnetometerCalibrator.DEFAULT_ROBUST_THRESHOLD_FACTOR,
            calibrator.magnetometerRobustThresholdFactor,
            0.0
        )
        assertEquals(
            SingleSensorStaticIntervalMagnetometerCalibrator.DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR,
            calibrator.magnetometerRobustStopThresholdFactor,
            0.0
        )
        assertNull(calibrator.estimatedAccelerometerMa)
        assertNull(calibrator.estimatedAccelerometerSx)
        assertNull(calibrator.estimatedAccelerometerSy)
        assertNull(calibrator.estimatedAccelerometerSz)
        assertNull(calibrator.estimatedAccelerometerMxy)
        assertNull(calibrator.estimatedAccelerometerMxz)
        assertNull(calibrator.estimatedAccelerometerMyx)
        assertNull(calibrator.estimatedAccelerometerMyz)
        assertNull(calibrator.estimatedAccelerometerMzx)
        assertNull(calibrator.estimatedAccelerometerMzy)
        assertNull(calibrator.estimatedAccelerometerCovariance)
        assertNull(calibrator.estimatedAccelerometerChiSq)
        assertNull(calibrator.estimatedAccelerometerMse)
        assertNull(calibrator.estimatedAccelerometerBiasX)
        assertNull(calibrator.estimatedAccelerometerBiasY)
        assertNull(calibrator.estimatedAccelerometerBiasZ)
        assertNull(calibrator.estimatedAccelerometerBiasXAsMeasurement)
        assertFalse(calibrator.getEstimatedAccelerometerBiasXAsMeasurement(acceleration))
        assertNull(calibrator.estimatedAccelerometerBiasYAsMeasurement)
        assertFalse(calibrator.getEstimatedAccelerometerBiasYAsMeasurement(acceleration))
        assertNull(calibrator.estimatedAccelerometerBiasZAsMeasurement)
        assertFalse(calibrator.getEstimatedAccelerometerBiasZAsMeasurement(acceleration))
        assertNull(calibrator.estimatedAccelerometerBiasAsTriad)
        assertFalse(calibrator.getEstimatedAccelerometerBiasAsTriad(accelerationTriad))
        assertNull(calibrator.estimatedAccelerometerBiasStandardDeviationNorm)
        assertNull(calibrator.estimatedGyroscopeMg)
        assertNull(calibrator.estimatedGyroscopeSx)
        assertNull(calibrator.estimatedGyroscopeSy)
        assertNull(calibrator.estimatedGyroscopeSz)
        assertNull(calibrator.estimatedGyroscopeMxy)
        assertNull(calibrator.estimatedGyroscopeMxz)
        assertNull(calibrator.estimatedGyroscopeMyx)
        assertNull(calibrator.estimatedGyroscopeMyz)
        assertNull(calibrator.estimatedGyroscopeMzx)
        assertNull(calibrator.estimatedGyroscopeMzy)
        assertNull(calibrator.estimatedGyroscopeGg)
        assertNull(calibrator.estimatedGyroscopeCovariance)
        assertNull(calibrator.estimatedGyroscopeChiSq)
        assertNull(calibrator.estimatedGyroscopeMse)
        assertNull(calibrator.estimatedGyroscopeBiasX)
        assertNull(calibrator.estimatedGyroscopeBiasY)
        assertNull(calibrator.estimatedGyroscopeBiasZ)
        assertNull(calibrator.estimatedGyroscopeBiasXAsMeasurement)
        assertFalse(calibrator.getEstimatedGyroscopeBiasXAsMeasurement(angularSpeed))
        assertNull(calibrator.estimatedGyroscopeBiasYAsMeasurement)
        assertFalse(calibrator.getEstimatedGyroscopeBiasYAsMeasurement(angularSpeed))
        assertNull(calibrator.estimatedGyroscopeBiasZAsMeasurement)
        assertFalse(calibrator.getEstimatedGyroscopeBiasZAsMeasurement(angularSpeed))
        assertNull(calibrator.estimatedGyroscopeBiasAsTriad)
        assertFalse(calibrator.getEstimatedGyroscopeBiasAsTriad(angularSpeedTriad))
        assertNull(calibrator.estimatedGyroscopeBiasStandardDeviationNorm)
        assertNull(calibrator.estimatedMagnetometerMm)
        assertNull(calibrator.estimatedMagnetometerSx)
        assertNull(calibrator.estimatedMagnetometerSy)
        assertNull(calibrator.estimatedMagnetometerSz)
        assertNull(calibrator.estimatedMagnetometerMxy)
        assertNull(calibrator.estimatedMagnetometerMxz)
        assertNull(calibrator.estimatedMagnetometerMyx)
        assertNull(calibrator.estimatedMagnetometerMyz)
        assertNull(calibrator.estimatedMagnetometerMzx)
        assertNull(calibrator.estimatedMagnetometerMzy)
        assertNull(calibrator.estimatedMagnetometerCovariance)
        assertNull(calibrator.estimatedMagnetometerChiSq)
        assertNull(calibrator.estimatedMagnetometerMse)
        assertNull(calibrator.estimatedMagnetometerHardIronX)
        assertNull(calibrator.estimatedMagnetometerHardIronY)
        assertNull(calibrator.estimatedMagnetometerHardIronZ)
        assertNull(calibrator.estimatedMagnetometerHardIronXAsMeasurement)
        assertFalse(calibrator.getEstimatedMagnetometerHardIronXAsMeasurement(b))
        assertNull(calibrator.estimatedMagnetometerHardIronYAsMeasurement)
        assertFalse(calibrator.getEstimatedMagnetometerHardIronYAsMeasurement(b))
        assertNull(calibrator.estimatedMagnetometerHardIronZAsMeasurement)
        assertFalse(calibrator.getEstimatedMagnetometerHardIronZAsMeasurement(b))
        assertNull(calibrator.estimatedMagnetometerHardIronAsTriad)
        assertFalse(calibrator.getEstimatedMagnetometerHardIronAsTriad(triad))
        assertNull(calibrator.gyroscopeBaseNoiseLevel)
        assertNull(calibrator.gyroscopeBaseNoiseLevelAsMeasurement)
        assertFalse(calibrator.getGyroscopeBaseNoiseLevelAsMeasurement(angularSpeed))
        assertNull(calibrator.magnetometerBaseNoiseLevel)
        assertNull(calibrator.magnetometerBaseNoiseLevelAsMeasurement)
        assertFalse(calibrator.getMagnetometerBaseNoiseLevelAsMeasurement(b))
        assertEquals(0, calibrator.numberOfProcessedGyroscopeMeasurements)
        assertTrue(calibrator.accelerometerMeasurements.isEmpty())
        assertTrue(calibrator.gyroscopeMeasurements.isEmpty())
        assertTrue(calibrator.magnetometerMeasurements.isEmpty())
        assertFalse(calibrator.isReadyToSolveAccelerometerCalibration)
        assertFalse(calibrator.isReadyToSolveGyroscopeCalibration)
        assertFalse(calibrator.isReadyToSolveMagnetometerCalibration)
        assertFalse(calibrator.isReadyToSolveCalibration)
        assertFalse(calibrator.running)
        assertEquals(TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE, calibrator.windowSize)
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES,
            calibrator.initialStaticSamples
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_THRESHOLD_FACTOR,
            calibrator.thresholdFactor,
            0.0
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR,
            calibrator.instantaneousNoiseLevelFactor,
            0.0
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
            calibrator.baseNoiseLevelAbsoluteThreshold,
            0.0
        )
        val baseNoiseLevel1 = calibrator.baseNoiseLevelAbsoluteThresholdAsMeasurement
        assertEquals(
            calibrator.baseNoiseLevelAbsoluteThreshold,
            baseNoiseLevel1.value.toDouble(),
            0.0
        )
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baseNoiseLevel1.unit)
        val baseNoiseLevel2 = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        calibrator.getBaseNoiseLevelAbsoluteThresholdAsMeasurement(baseNoiseLevel2)
        assertEquals(baseNoiseLevel1, baseNoiseLevel2)
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
            calibrator.magnetometerBaseNoiseLevelAbsoluteThreshold,
            0.0
        )
        val magnetometerBaseNoiseLevel1 =
            calibrator.magnetometerBaseNoiseLevelAbsoluteThresholdAsMeasurement
        assertEquals(
            calibrator.magnetometerBaseNoiseLevelAbsoluteThreshold,
            magnetometerBaseNoiseLevel1.value.toDouble(),
            0.0
        )
        assertEquals(MagneticFluxDensityUnit.TESLA, magnetometerBaseNoiseLevel1.unit)
        val magnetometerBaseNoiseLevel2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        calibrator.getMagnetometerBaseNoiseLevelAbsoluteThresholdAsMeasurement(
            magnetometerBaseNoiseLevel2
        )
        assertEquals(magnetometerBaseNoiseLevel1, magnetometerBaseNoiseLevel2)
        assertNull(calibrator.accelerometerBaseNoiseLevel)
        assertNull(calibrator.accelerometerBaseNoiseLevelAsMeasurement)
        assertFalse(calibrator.getAccelerometerBaseNoiseLevelAsMeasurement(acceleration))
        assertNull(calibrator.accelerometerBaseNoiseLevelPsd)
        assertNull(calibrator.accelerometerBaseNoiseLevelRootPsd)
        assertNull(calibrator.magnetometerBaseNoiseLevelPsd)
        assertNull(calibrator.magnetometerBaseNoiseLevelRootPsd)
        assertNull(calibrator.threshold)
        assertNull(calibrator.thresholdAsMeasurement)
        assertFalse(calibrator.getThresholdAsMeasurement(acceleration))
        assertNull(calibrator.magnetometerThreshold)
        assertNull(calibrator.magnetometerThresholdAsMeasurement)
        assertFalse(calibrator.getMagnetometerThresholdAsMeasurement(b))
        assertEquals(0, calibrator.processedStaticSamples)
        assertEquals(0, calibrator.processedDynamicSamples)
        assertFalse(calibrator.isStaticIntervalSkipped)
        assertFalse(calibrator.isDynamicIntervalSkipped)
        assertNull(calibrator.accelerometerAverageTimeInterval)
        assertNull(calibrator.accelerometerAverageTimeIntervalAsTime)
        val time = Time(0.0, TimeUnit.SECOND)
        assertFalse(calibrator.getAccelerometerAverageTimeIntervalAsTime(time))
        assertNull(calibrator.magnetometerAverageTimeInterval)
        assertNull(calibrator.magnetometerAverageTimeIntervalAsTime)
        assertFalse(calibrator.getMagnetometerAverageTimeIntervalAsTime(time))
        assertNull(calibrator.accelerometerTimeIntervalVariance)
        assertNull(calibrator.accelerometerTimeIntervalStandardDeviation)
        assertNull(calibrator.accelerometerTimeIntervalStandardDeviationAsTime)
        assertFalse(calibrator.getAccelerometerTimeIntervalStandardDeviationAsTime(time))
        assertEquals(
            StaticIntervalAccelerometerCalibrator.ACCELEROMETER_UNKNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.requiredMeasurements
        )
        assertNull(calibrator.magnetometerTimeIntervalVariance)
        assertNull(calibrator.magnetometerTimeIntervalStandardDeviation)
        assertNull(calibrator.magnetometerTimeIntervalStandardDeviationAsTime)
        assertFalse(calibrator.getMagnetometerTimeIntervalStandardDeviationAsTime(time))
        assertEquals(0, calibrator.numberOfProcessedAccelerometerMeasurements)
    }

    @Test
    fun constructor_whenAllParameters_returnsExpectedValues() {
        val location = getLocation()
        val timestamp = Date()
        val worldMagneticModel = WorldMagneticModel()
        val accelerometerQualityScoreMapper = DefaultAccelerometerQualityScoreMapper()
        val gyroscopeQualityScoreMapper = DefaultGyroscopeQualityScoreMapper()
        val magnetometerQualityScoreMapper = DefaultMagnetometerQualityScoreMapper()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(
            context,
            location,
            timestamp,
            worldMagneticModel,
            AccelerometerSensorType.ACCELEROMETER,
            GyroscopeSensorType.GYROSCOPE,
            MagnetometerSensorType.MAGNETOMETER,
            SensorDelay.NORMAL,
            SensorDelay.NORMAL,
            SensorDelay.NORMAL,
            solveCalibrationWhenEnoughMeasurements = false,
            isAccelerometerGroundTruthInitialBias = true,
            isGyroscopeGroundTruthInitialBias = true,
            isMagnetometerGroundTruthInitialHardIron = true,
            initializationStartedListener,
            initializationCompletedListener,
            errorListener,
            staticIntervalDetectedListener,
            dynamicIntervalDetectedListener,
            staticIntervalSkippedListener,
            dynamicIntervalSkippedListener,
            generatedAccelerometerMeasurementListener,
            generatedGyroscopeMeasurementListener,
            generatedMagnetometerMeasurementListener,
            readyToSolveCalibrationListener,
            calibrationSolvingStartedListener,
            calibrationCompletedListener,
            stoppedListener,
            unreliableGravityNormEstimationListener,
            initialAccelerometerBiasAvailableListener,
            initialGyroscopeBiasAvailableListener,
            initialMagnetometerHardIronAvailableListener,
            accuracyChangedListener,
            accelerometerQualityScoreMapper,
            gyroscopeQualityScoreMapper,
            magnetometerQualityScoreMapper
        )

        // check default values
        assertSame(context, calibrator.context)
        assertSame(location, calibrator.location)
        assertSame(timestamp, calibrator.timestamp)
        assertSame(worldMagneticModel, calibrator.worldMagneticModel)
        assertFalse(calibrator.isInitialMagneticFluxDensityNormMeasured)
        assertFalse(calibrator.isGravityNormEstimated)
        assertEquals(
            AccelerometerSensorType.ACCELEROMETER,
            calibrator.accelerometerSensorType
        )
        assertEquals(
            GyroscopeSensorType.GYROSCOPE,
            calibrator.gyroscopeSensorType
        )
        assertEquals(
            MagnetometerSensorType.MAGNETOMETER,
            calibrator.magnetometerSensorType
        )
        assertEquals(SensorDelay.NORMAL, calibrator.accelerometerSensorDelay)
        assertEquals(SensorDelay.NORMAL, calibrator.gyroscopeSensorDelay)
        assertEquals(SensorDelay.NORMAL, calibrator.magnetometerSensorDelay)
        assertFalse(calibrator.solveCalibrationWhenEnoughMeasurements)
        assertTrue(calibrator.isAccelerometerGroundTruthInitialBias)
        assertTrue(calibrator.isGyroscopeGroundTruthInitialBias)
        assertTrue(calibrator.isMagnetometerGroundTruthInitialHardIron)
        assertSame(initializationStartedListener, calibrator.initializationStartedListener)
        assertSame(initializationCompletedListener, calibrator.initializationCompletedListener)
        assertSame(errorListener, calibrator.errorListener)
        assertSame(staticIntervalDetectedListener, calibrator.staticIntervalDetectedListener)
        assertSame(dynamicIntervalDetectedListener, calibrator.dynamicIntervalDetectedListener)
        assertSame(staticIntervalSkippedListener, calibrator.staticIntervalSkippedListener)
        assertSame(dynamicIntervalSkippedListener, calibrator.dynamicIntervalSkippedListener)
        assertSame(
            generatedAccelerometerMeasurementListener,
            calibrator.generatedAccelerometerMeasurementListener
        )
        assertSame(
            generatedGyroscopeMeasurementListener,
            calibrator.generatedGyroscopeMeasurementListener
        )
        assertSame(
            generatedMagnetometerMeasurementListener,
            calibrator.generatedMagnetometerMeasurementListener
        )
        assertSame(readyToSolveCalibrationListener, calibrator.readyToSolveCalibrationListener)
        assertSame(calibrationSolvingStartedListener, calibrator.calibrationSolvingStartedListener)
        assertSame(calibrationCompletedListener, calibrator.calibrationCompletedListener)
        assertSame(stoppedListener, calibrator.stoppedListener)
        assertSame(
            unreliableGravityNormEstimationListener,
            calibrator.unreliableGravityNormEstimationListener
        )
        assertSame(
            initialAccelerometerBiasAvailableListener,
            calibrator.initialAccelerometerBiasAvailableListener
        )
        assertSame(
            initialGyroscopeBiasAvailableListener,
            calibrator.initialGyroscopeBiasAvailableListener
        )
        assertSame(
            initialMagnetometerHardIronAvailableListener,
            calibrator.initialMagnetometerHardIronAvailableListener
        )
        assertSame(accuracyChangedListener, calibrator.accuracyChangedListener)
        assertSame(accelerometerQualityScoreMapper, calibrator.accelerometerQualityScoreMapper)
        assertSame(gyroscopeQualityScoreMapper, calibrator.gyroscopeQualityScoreMapper)
        assertSame(magnetometerQualityScoreMapper, calibrator.magnetometerQualityScoreMapper)
        assertNull(calibrator.initialMagneticFluxDensityNorm)
        val gravityNorm = GravityHelper.getGravityNormForLocation(location)
        assertEquals(gravityNorm, calibrator.gravityNorm)
        assertNull(calibrator.accelerometerInitialBiasX)
        assertNull(calibrator.accelerometerInitialBiasY)
        assertNull(calibrator.accelerometerInitialBiasZ)
        assertNull(calibrator.accelerometerInitialBiasXAsMeasurement)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getAccelerometerInitialBiasXAsMeasurement(acceleration))
        assertNull(calibrator.accelerometerInitialBiasYAsMeasurement)
        assertFalse(calibrator.getAccelerometerInitialBiasYAsMeasurement(acceleration))
        assertNull(calibrator.accelerometerInitialBiasZAsMeasurement)
        assertFalse(calibrator.getAccelerometerInitialBiasZAsMeasurement(acceleration))
        assertNull(calibrator.accelerometerInitialBiasAsTriad)
        val accelerationTriad = AccelerationTriad()
        assertFalse(calibrator.getAccelerometerInitialBiasAsTriad(accelerationTriad))
        assertNull(calibrator.gyroscopeInitialBiasX)
        assertNull(calibrator.gyroscopeInitialBiasY)
        assertNull(calibrator.gyroscopeInitialBiasZ)
        assertNull(calibrator.gyroscopeInitialBiasXAsMeasurement)
        val angularSpeed = AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND)
        assertFalse(calibrator.getGyroscopeInitialBiasXAsMeasurement(angularSpeed))
        assertNull(calibrator.gyroscopeInitialBiasYAsMeasurement)
        assertFalse(calibrator.getGyroscopeInitialBiasYAsMeasurement(angularSpeed))
        assertNull(calibrator.gyroscopeInitialBiasZAsMeasurement)
        assertFalse(calibrator.getGyroscopeInitialBiasZAsMeasurement(angularSpeed))
        assertNull(calibrator.gyroscopeInitialBiasAsTriad)
        val angularSpeedTriad = AngularSpeedTriad()
        assertFalse(calibrator.getGyroscopeInitialBiasAsTriad(angularSpeedTriad))
        assertNull(calibrator.magnetometerInitialHardIronX)
        assertNull(calibrator.magnetometerInitialHardIronY)
        assertNull(calibrator.magnetometerInitialHardIronZ)
        assertNull(calibrator.magnetometerInitialHardIronXAsMeasurement)
        val b = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(calibrator.getMagnetometerInitialHardIronXAsMeasurement(b))
        assertNull(calibrator.magnetometerInitialHardIronYAsMeasurement)
        assertFalse(calibrator.getMagnetometerInitialHardIronYAsMeasurement(b))
        assertNull(calibrator.magnetometerInitialHardIronZAsMeasurement)
        assertFalse(calibrator.getMagnetometerInitialHardIronZAsMeasurement(b))
        assertNull(calibrator.magnetometerInitialHardIronAsTriad)
        val triad = MagneticFluxDensityTriad()
        assertFalse(calibrator.getMagnetometerInitialHardIronAsTriad(triad))
        assertNull(calibrator.accelerometerSensor)
        assertNull(calibrator.gyroscopeSensor)
        assertNull(calibrator.magnetometerSensor)
        assertNull(calibrator.gravitySensor)
        val accelerometerInitialMa1 = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        val accelerometerInitialMa2 = calibrator.accelerometerInitialMa
        assertEquals(accelerometerInitialMa1, accelerometerInitialMa2)
        val accelerometerInitialMa3 =
            Matrix.identity(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        calibrator.getAccelerometerInitialMa(accelerometerInitialMa3)
        assertEquals(accelerometerInitialMa1, accelerometerInitialMa3)
        assertEquals(0.0, calibrator.accelerometerInitialSx, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialSy, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialSz, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMxy, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMxz, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMyx, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMyz, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMzx, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMzy, 0.0)
        val gyroscopeInitialMg1 = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        val gyroscopeInitialMg2 = calibrator.gyroscopeInitialMg
        assertEquals(gyroscopeInitialMg1, gyroscopeInitialMg2)
        val gyroscopeInitialMg3 =
            Matrix.identity(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        calibrator.getGyroscopeInitialMg(gyroscopeInitialMg3)
        assertEquals(gyroscopeInitialMg1, gyroscopeInitialMg3)
        assertEquals(0.0, calibrator.gyroscopeInitialSx, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialSy, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialSz, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMxy, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMxz, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMyx, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMyz, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMzx, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMzy, 0.0)
        val gyroscopeInitialGg1 = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        val gyroscopeInitialGg2 = calibrator.gyroscopeInitialGg
        assertEquals(gyroscopeInitialGg1, gyroscopeInitialGg2)
        val mm1 = Matrix(MM_SIZE, MM_SIZE)
        assertEquals(mm1, calibrator.magnetometerInitialMm)
        val mm2 = Matrix.identity(MM_SIZE, MM_SIZE)
        calibrator.getMagnetometerInitialMm(mm2)
        assertEquals(mm1, mm2)
        assertEquals(0.0, calibrator.magnetometerInitialSx, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialSy, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialSz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMxy, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMxz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMyx, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMyz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMzx, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMzy, 0.0)
        assertFalse(calibrator.isAccelerometerCommonAxisUsed)
        assertFalse(calibrator.isGyroscopeCommonAxisUsed)
        assertFalse(calibrator.isMagnetometerCommonAxisUsed)
        assertFalse(calibrator.isGDependentCrossBiasesEstimated)
        assertEquals(
            StaticIntervalAccelerometerCalibrator.ACCELEROMETER_KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.minimumRequiredAccelerometerMeasurements
        )
        assertEquals(
            StaticIntervalGyroscopeCalibrator.GYROSCOPE_KNOWN_BIAS_MINIMUM_SEQUENCES_GENERAL,
            calibrator.minimumRequiredGyroscopeMeasurements
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.MAGNETOMETER_KNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.minimumRequiredMagnetometerMeasurements
        )
        assertEquals(
            max(
                max(
                    calibrator.minimumRequiredAccelerometerMeasurements,
                    calibrator.minimumRequiredGyroscopeMeasurements
                ), calibrator.minimumRequiredMagnetometerMeasurements
            ), calibrator.minimumRequiredMeasurements
        )
        assertNull(calibrator.averageGravityNorm)
        assertNull(calibrator.averageGravityNormAsMeasurement)
        assertFalse(calibrator.getAverageGravityNormAsMeasurement(acceleration))
        assertNull(calibrator.gravityNormVariance)
        assertNull(calibrator.gravityNormStandardDeviation)
        assertNull(calibrator.gravityNormStandardDeviationAsMeasurement)
        assertFalse(calibrator.getGravityNormStandardDeviationAsMeasurement(acceleration))
        assertNull(calibrator.gravityPsd)
        assertNull(calibrator.gravityRootPsd)
        assertNull(calibrator.accelerometerRobustMethod)
        assertEquals(
            StaticIntervalAccelerometerCalibrator.ROBUST_DEFAULT_CONFIDENCE,
            calibrator.accelerometerRobustConfidence,
            0.0
        )
        assertEquals(
            StaticIntervalAccelerometerCalibrator.ROBUST_DEFAULT_MAX_ITERATIONS,
            calibrator.accelerometerRobustMaxIterations
        )
        assertEquals(
            calibrator.minimumRequiredAccelerometerMeasurements,
            calibrator.accelerometerRobustPreliminarySubsetSize
        )
        assertNull(calibrator.accelerometerRobustThreshold)
        assertEquals(
            StaticIntervalAccelerometerCalibrator.DEFAULT_ROBUST_THRESHOLD_FACTOR,
            calibrator.accelerometerRobustThresholdFactor,
            0.0
        )
        assertEquals(
            StaticIntervalAccelerometerCalibrator.DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR,
            calibrator.accelerometerRobustStopThresholdFactor,
            0.0
        )
        assertNull(calibrator.gyroscopeRobustMethod)
        assertEquals(
            StaticIntervalGyroscopeCalibrator.ROBUST_DEFAULT_CONFIDENCE,
            calibrator.gyroscopeRobustConfidence,
            0.0
        )
        assertEquals(
            StaticIntervalGyroscopeCalibrator.ROBUST_DEFAULT_MAX_ITERATIONS,
            calibrator.gyroscopeRobustMaxIterations
        )
        assertEquals(
            calibrator.minimumRequiredGyroscopeMeasurements,
            calibrator.gyroscopeRobustPreliminarySubsetSize
        )
        assertNull(calibrator.gyroscopeRobustThreshold)
        assertEquals(
            StaticIntervalGyroscopeCalibrator.DEFAULT_ROBUST_THRESHOLD_FACTOR,
            calibrator.gyroscopeRobustThresholdFactor,
            0.0
        )
        assertEquals(
            StaticIntervalGyroscopeCalibrator.DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR,
            calibrator.gyroscopeRobustStopThresholdFactor,
            0.0
        )
        assertNull(calibrator.magnetometerRobustMethod)
        assertEquals(
            SingleSensorStaticIntervalMagnetometerCalibrator.ROBUST_DEFAULT_CONFIDENCE,
            calibrator.magnetometerRobustConfidence,
            0.0
        )
        assertEquals(
            SingleSensorStaticIntervalMagnetometerCalibrator.ROBUST_DEFAULT_MAX_ITERATIONS,
            calibrator.magnetometerRobustMaxIterations
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.MAGNETOMETER_KNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.magnetometerRobustPreliminarySubsetSize
        )
        assertNull(calibrator.magnetometerRobustThreshold)
        assertEquals(
            SingleSensorStaticIntervalMagnetometerCalibrator.DEFAULT_ROBUST_THRESHOLD_FACTOR,
            calibrator.magnetometerRobustThresholdFactor,
            0.0
        )
        assertEquals(
            SingleSensorStaticIntervalMagnetometerCalibrator.DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR,
            calibrator.magnetometerRobustStopThresholdFactor,
            0.0
        )
        assertNull(calibrator.estimatedAccelerometerMa)
        assertNull(calibrator.estimatedAccelerometerSx)
        assertNull(calibrator.estimatedAccelerometerSy)
        assertNull(calibrator.estimatedAccelerometerSz)
        assertNull(calibrator.estimatedAccelerometerMxy)
        assertNull(calibrator.estimatedAccelerometerMxz)
        assertNull(calibrator.estimatedAccelerometerMyx)
        assertNull(calibrator.estimatedAccelerometerMyz)
        assertNull(calibrator.estimatedAccelerometerMzx)
        assertNull(calibrator.estimatedAccelerometerMzy)
        assertNull(calibrator.estimatedAccelerometerCovariance)
        assertNull(calibrator.estimatedAccelerometerChiSq)
        assertNull(calibrator.estimatedAccelerometerMse)
        assertNull(calibrator.estimatedAccelerometerBiasX)
        assertNull(calibrator.estimatedAccelerometerBiasY)
        assertNull(calibrator.estimatedAccelerometerBiasZ)
        assertNull(calibrator.estimatedAccelerometerBiasXAsMeasurement)
        assertFalse(calibrator.getEstimatedAccelerometerBiasXAsMeasurement(acceleration))
        assertNull(calibrator.estimatedAccelerometerBiasYAsMeasurement)
        assertFalse(calibrator.getEstimatedAccelerometerBiasYAsMeasurement(acceleration))
        assertNull(calibrator.estimatedAccelerometerBiasZAsMeasurement)
        assertFalse(calibrator.getEstimatedAccelerometerBiasZAsMeasurement(acceleration))
        assertNull(calibrator.estimatedAccelerometerBiasAsTriad)
        assertFalse(calibrator.getEstimatedAccelerometerBiasAsTriad(accelerationTriad))
        assertNull(calibrator.estimatedAccelerometerBiasStandardDeviationNorm)
        assertNull(calibrator.estimatedGyroscopeMg)
        assertNull(calibrator.estimatedGyroscopeSx)
        assertNull(calibrator.estimatedGyroscopeSy)
        assertNull(calibrator.estimatedGyroscopeSz)
        assertNull(calibrator.estimatedGyroscopeMxy)
        assertNull(calibrator.estimatedGyroscopeMxz)
        assertNull(calibrator.estimatedGyroscopeMyx)
        assertNull(calibrator.estimatedGyroscopeMyz)
        assertNull(calibrator.estimatedGyroscopeMzx)
        assertNull(calibrator.estimatedGyroscopeMzy)
        assertNull(calibrator.estimatedGyroscopeGg)
        assertNull(calibrator.estimatedGyroscopeCovariance)
        assertNull(calibrator.estimatedGyroscopeChiSq)
        assertNull(calibrator.estimatedGyroscopeMse)
        assertNull(calibrator.estimatedGyroscopeBiasX)
        assertNull(calibrator.estimatedGyroscopeBiasY)
        assertNull(calibrator.estimatedGyroscopeBiasZ)
        assertNull(calibrator.estimatedGyroscopeBiasXAsMeasurement)
        assertFalse(calibrator.getEstimatedGyroscopeBiasXAsMeasurement(angularSpeed))
        assertNull(calibrator.estimatedGyroscopeBiasYAsMeasurement)
        assertFalse(calibrator.getEstimatedGyroscopeBiasYAsMeasurement(angularSpeed))
        assertNull(calibrator.estimatedGyroscopeBiasZAsMeasurement)
        assertFalse(calibrator.getEstimatedGyroscopeBiasZAsMeasurement(angularSpeed))
        assertNull(calibrator.estimatedGyroscopeBiasAsTriad)
        assertFalse(calibrator.getEstimatedGyroscopeBiasAsTriad(angularSpeedTriad))
        assertNull(calibrator.estimatedGyroscopeBiasStandardDeviationNorm)
        assertNull(calibrator.estimatedMagnetometerMm)
        assertNull(calibrator.estimatedMagnetometerSx)
        assertNull(calibrator.estimatedMagnetometerSy)
        assertNull(calibrator.estimatedMagnetometerSz)
        assertNull(calibrator.estimatedMagnetometerMxy)
        assertNull(calibrator.estimatedMagnetometerMxz)
        assertNull(calibrator.estimatedMagnetometerMyx)
        assertNull(calibrator.estimatedMagnetometerMyz)
        assertNull(calibrator.estimatedMagnetometerMzx)
        assertNull(calibrator.estimatedMagnetometerMzy)
        assertNull(calibrator.estimatedMagnetometerCovariance)
        assertNull(calibrator.estimatedMagnetometerChiSq)
        assertNull(calibrator.estimatedMagnetometerMse)
        assertNull(calibrator.estimatedMagnetometerHardIronX)
        assertNull(calibrator.estimatedMagnetometerHardIronY)
        assertNull(calibrator.estimatedMagnetometerHardIronZ)
        assertNull(calibrator.estimatedMagnetometerHardIronXAsMeasurement)
        assertFalse(calibrator.getEstimatedMagnetometerHardIronXAsMeasurement(b))
        assertNull(calibrator.estimatedMagnetometerHardIronYAsMeasurement)
        assertFalse(calibrator.getEstimatedMagnetometerHardIronYAsMeasurement(b))
        assertNull(calibrator.estimatedMagnetometerHardIronZAsMeasurement)
        assertFalse(calibrator.getEstimatedMagnetometerHardIronZAsMeasurement(b))
        assertNull(calibrator.estimatedMagnetometerHardIronAsTriad)
        assertFalse(calibrator.getEstimatedMagnetometerHardIronAsTriad(triad))
        assertNull(calibrator.gyroscopeBaseNoiseLevel)
        assertNull(calibrator.gyroscopeBaseNoiseLevelAsMeasurement)
        assertFalse(calibrator.getGyroscopeBaseNoiseLevelAsMeasurement(angularSpeed))
        assertNull(calibrator.magnetometerBaseNoiseLevel)
        assertNull(calibrator.magnetometerBaseNoiseLevelAsMeasurement)
        assertFalse(calibrator.getMagnetometerBaseNoiseLevelAsMeasurement(b))
        assertEquals(0, calibrator.numberOfProcessedGyroscopeMeasurements)
        assertTrue(calibrator.accelerometerMeasurements.isEmpty())
        assertTrue(calibrator.gyroscopeMeasurements.isEmpty())
        assertTrue(calibrator.magnetometerMeasurements.isEmpty())
        assertFalse(calibrator.isReadyToSolveAccelerometerCalibration)
        assertFalse(calibrator.isReadyToSolveGyroscopeCalibration)
        assertFalse(calibrator.isReadyToSolveMagnetometerCalibration)
        assertFalse(calibrator.isReadyToSolveCalibration)
        assertFalse(calibrator.running)
        assertEquals(TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE, calibrator.windowSize)
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES,
            calibrator.initialStaticSamples
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_THRESHOLD_FACTOR,
            calibrator.thresholdFactor,
            0.0
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR,
            calibrator.instantaneousNoiseLevelFactor,
            0.0
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
            calibrator.baseNoiseLevelAbsoluteThreshold,
            0.0
        )
        val baseNoiseLevel1 = calibrator.baseNoiseLevelAbsoluteThresholdAsMeasurement
        assertEquals(
            calibrator.baseNoiseLevelAbsoluteThreshold,
            baseNoiseLevel1.value.toDouble(),
            0.0
        )
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baseNoiseLevel1.unit)
        val baseNoiseLevel2 = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        calibrator.getBaseNoiseLevelAbsoluteThresholdAsMeasurement(baseNoiseLevel2)
        assertEquals(baseNoiseLevel1, baseNoiseLevel2)
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
            calibrator.magnetometerBaseNoiseLevelAbsoluteThreshold,
            0.0
        )
        val magnetometerBaseNoiseLevel1 =
            calibrator.magnetometerBaseNoiseLevelAbsoluteThresholdAsMeasurement
        assertEquals(
            calibrator.magnetometerBaseNoiseLevelAbsoluteThreshold,
            magnetometerBaseNoiseLevel1.value.toDouble(),
            0.0
        )
        assertEquals(MagneticFluxDensityUnit.TESLA, magnetometerBaseNoiseLevel1.unit)
        val magnetometerBaseNoiseLevel2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        calibrator.getMagnetometerBaseNoiseLevelAbsoluteThresholdAsMeasurement(
            magnetometerBaseNoiseLevel2
        )
        assertEquals(magnetometerBaseNoiseLevel1, magnetometerBaseNoiseLevel2)
        assertNull(calibrator.accelerometerBaseNoiseLevel)
        assertNull(calibrator.accelerometerBaseNoiseLevelAsMeasurement)
        assertFalse(calibrator.getAccelerometerBaseNoiseLevelAsMeasurement(acceleration))
        assertNull(calibrator.accelerometerBaseNoiseLevelPsd)
        assertNull(calibrator.accelerometerBaseNoiseLevelRootPsd)
        assertNull(calibrator.magnetometerBaseNoiseLevelPsd)
        assertNull(calibrator.magnetometerBaseNoiseLevelRootPsd)
        assertNull(calibrator.threshold)
        assertNull(calibrator.thresholdAsMeasurement)
        assertFalse(calibrator.getThresholdAsMeasurement(acceleration))
        assertNull(calibrator.magnetometerThreshold)
        assertNull(calibrator.magnetometerThresholdAsMeasurement)
        assertFalse(calibrator.getMagnetometerThresholdAsMeasurement(b))
        assertEquals(0, calibrator.processedStaticSamples)
        assertEquals(0, calibrator.processedDynamicSamples)
        assertFalse(calibrator.isStaticIntervalSkipped)
        assertFalse(calibrator.isDynamicIntervalSkipped)
        assertNull(calibrator.accelerometerAverageTimeInterval)
        assertNull(calibrator.accelerometerAverageTimeIntervalAsTime)
        val time = Time(0.0, TimeUnit.SECOND)
        assertFalse(calibrator.getAccelerometerAverageTimeIntervalAsTime(time))
        assertNull(calibrator.magnetometerAverageTimeInterval)
        assertNull(calibrator.magnetometerAverageTimeIntervalAsTime)
        assertFalse(calibrator.getMagnetometerAverageTimeIntervalAsTime(time))
        assertNull(calibrator.accelerometerTimeIntervalVariance)
        assertNull(calibrator.accelerometerTimeIntervalStandardDeviation)
        assertNull(calibrator.accelerometerTimeIntervalStandardDeviationAsTime)
        assertFalse(calibrator.getAccelerometerTimeIntervalStandardDeviationAsTime(time))
        assertEquals(
            StaticIntervalGyroscopeCalibrator.GYROSCOPE_KNOWN_BIAS_MINIMUM_SEQUENCES_GENERAL,
            calibrator.requiredMeasurements
        )
        assertNull(calibrator.magnetometerTimeIntervalVariance)
        assertNull(calibrator.magnetometerTimeIntervalStandardDeviation)
        assertNull(calibrator.magnetometerTimeIntervalStandardDeviationAsTime)
        assertFalse(calibrator.getMagnetometerTimeIntervalStandardDeviationAsTime(time))
        assertEquals(0, calibrator.numberOfProcessedAccelerometerMeasurements)
    }

    @Test
    fun location_whenNotRunning_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.location)
        assertFalse(calibrator.running)
        assertTrue(calibrator.isGravityNormEstimated)

        // set new value
        val location = getLocation()
        calibrator.location = location

        // check
        assertSame(location, calibrator.location)
        assertFalse(calibrator.isGravityNormEstimated)
    }

    @Test(expected = IllegalStateException::class)
    fun location_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            accelerometerAndGyroscopeCalibrator,
            "running",
            true
        )
        setPrivateProperty(
            SingleSensorStaticIntervalCalibrator::class,
            magnetometerCalibrator,
            "running",
            true
        )
        assertTrue(calibrator.running)

        // set new value
        val location = getLocation()
        calibrator.location = location
    }

    @Test
    fun timestamp_whenNotRunning_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNotNull(calibrator.timestamp)

        // set new value
        val timestamp = Date()
        calibrator.timestamp = timestamp

        // check
        assertSame(timestamp, calibrator.timestamp)
    }

    @Test(expected = IllegalStateException::class)
    fun timestamp_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            accelerometerAndGyroscopeCalibrator,
            "running",
            true
        )
        setPrivateProperty(
            SingleSensorStaticIntervalCalibrator::class,
            magnetometerCalibrator,
            "running",
            true
        )
        assertTrue(calibrator.running)

        calibrator.timestamp = Date()
    }

    @Test
    fun worldMagneticModel_whenNotRunning_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.worldMagneticModel)

        // set new value
        val worldMagneticModel = WorldMagneticModel()
        calibrator.worldMagneticModel = worldMagneticModel

        // check
        assertSame(worldMagneticModel, calibrator.worldMagneticModel)
    }

    @Test(expected = IllegalStateException::class)
    fun worldMagneticModel_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            accelerometerAndGyroscopeCalibrator,
            "running",
            true
        )
        setPrivateProperty(
            SingleSensorStaticIntervalCalibrator::class,
            magnetometerCalibrator,
            "running",
            true
        )
        assertTrue(calibrator.running)

        calibrator.worldMagneticModel = WorldMagneticModel()
    }

    @Test
    fun isInitialMagneticFluxDensityNormMeasured_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default values
        assertTrue(calibrator.isInitialMagneticFluxDensityNormMeasured)
        assertNull(calibrator.location)
        assertNotNull(calibrator.timestamp)

        // set location
        val location = getLocation()
        calibrator.location = location

        // check
        assertFalse(calibrator.isInitialMagneticFluxDensityNormMeasured)
        assertSame(location, calibrator.location)
        assertNotNull(calibrator.timestamp)

        // unset timestamp
        calibrator.timestamp = null

        // check
        assertTrue(calibrator.isInitialMagneticFluxDensityNormMeasured)
        assertSame(location, calibrator.location)
        assertNull(calibrator.timestamp)
    }

    @Test
    fun initializationStartedListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.initializationStartedListener)

        // set new value
        calibrator.initializationStartedListener = initializationStartedListener

        // check
        assertSame(initializationStartedListener, calibrator.initializationStartedListener)
    }

    @Test
    fun initializationCompletedListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.initializationCompletedListener)

        // set new value
        calibrator.initializationCompletedListener = initializationCompletedListener

        // check
        assertSame(initializationCompletedListener, calibrator.initializationCompletedListener)
    }

    @Test
    fun errorListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.errorListener)

        // set new value
        calibrator.errorListener = errorListener

        // check
        assertSame(errorListener, calibrator.errorListener)
    }

    @Test
    fun staticIntervalDetectedListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.staticIntervalDetectedListener)

        // set new value
        calibrator.staticIntervalDetectedListener = staticIntervalDetectedListener

        // check
        assertSame(staticIntervalDetectedListener, calibrator.staticIntervalDetectedListener)
    }

    @Test
    fun dynamicIntervalDetectedListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.dynamicIntervalDetectedListener)

        // set new value
        calibrator.dynamicIntervalDetectedListener = dynamicIntervalDetectedListener

        // check
        assertSame(dynamicIntervalDetectedListener, calibrator.dynamicIntervalDetectedListener)
    }

    @Test
    fun staticIntervalSkippedListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.staticIntervalSkippedListener)

        // set new value
        calibrator.staticIntervalSkippedListener = staticIntervalSkippedListener

        // check
        assertSame(staticIntervalSkippedListener, calibrator.staticIntervalSkippedListener)
    }

    @Test
    fun dynamicIntervalSkippedListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.dynamicIntervalSkippedListener)

        // set new value
        calibrator.dynamicIntervalSkippedListener = dynamicIntervalSkippedListener

        // check
        assertSame(dynamicIntervalSkippedListener, calibrator.dynamicIntervalSkippedListener)
    }

    @Test
    fun generatedAccelerometerMeasurementListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.generatedAccelerometerMeasurementListener)

        // set new value
        calibrator.generatedAccelerometerMeasurementListener =
            generatedAccelerometerMeasurementListener

        // check
        assertSame(
            generatedAccelerometerMeasurementListener,
            calibrator.generatedAccelerometerMeasurementListener
        )
    }

    @Test
    fun generatedGyroscopeMeasurementListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.generatedGyroscopeMeasurementListener)

        // set new value
        calibrator.generatedGyroscopeMeasurementListener = generatedGyroscopeMeasurementListener

        // check
        assertSame(
            generatedGyroscopeMeasurementListener,
            calibrator.generatedGyroscopeMeasurementListener
        )
    }

    @Test
    fun generatedMagnetometerMeasurementListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.generatedMagnetometerMeasurementListener)

        // set new value
        calibrator.generatedMagnetometerMeasurementListener =
            generatedMagnetometerMeasurementListener

        // check
        assertSame(
            generatedMagnetometerMeasurementListener,
            calibrator.generatedMagnetometerMeasurementListener
        )
    }

    @Test
    fun readyToSolveCalibrationListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.readyToSolveCalibrationListener)

        // set new value
        calibrator.readyToSolveCalibrationListener = readyToSolveCalibrationListener

        // check
        assertSame(readyToSolveCalibrationListener, calibrator.readyToSolveCalibrationListener)
    }

    @Test
    fun calibrationSolvingStartedListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.calibrationSolvingStartedListener)

        // set new value
        calibrator.calibrationSolvingStartedListener = calibrationSolvingStartedListener

        // check
        assertSame(calibrationSolvingStartedListener, calibrator.calibrationSolvingStartedListener)
    }

    @Test
    fun calibrationCompletedListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.calibrationCompletedListener)

        // set new value
        calibrator.calibrationCompletedListener = calibrationCompletedListener

        // check
        assertSame(calibrationCompletedListener, calibrator.calibrationCompletedListener)
    }

    @Test
    fun stoppedListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.stoppedListener)

        // set new value
        calibrator.stoppedListener = stoppedListener

        // check
        assertSame(stoppedListener, calibrator.stoppedListener)
    }

    @Test
    fun unreliableGravityNormEstimationListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.unreliableGravityNormEstimationListener)

        // set new value
        calibrator.unreliableGravityNormEstimationListener = unreliableGravityNormEstimationListener

        // check
        assertSame(
            unreliableGravityNormEstimationListener,
            calibrator.unreliableGravityNormEstimationListener
        )
    }

    @Test
    fun initialAccelerometerBiasAvailableListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.initialAccelerometerBiasAvailableListener)

        // set new value
        calibrator.initialAccelerometerBiasAvailableListener =
            initialAccelerometerBiasAvailableListener

        // check
        assertSame(
            initialAccelerometerBiasAvailableListener,
            calibrator.initialAccelerometerBiasAvailableListener
        )
    }

    @Test
    fun initialGyroscopeBiasAvailableListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.initialGyroscopeBiasAvailableListener)

        // set new value
        calibrator.initialGyroscopeBiasAvailableListener = initialGyroscopeBiasAvailableListener

        // check
        assertSame(
            initialGyroscopeBiasAvailableListener,
            calibrator.initialGyroscopeBiasAvailableListener
        )
    }

    @Test
    fun initialMagnetometerHardIronAvailableListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.initialMagnetometerHardIronAvailableListener)

        // set new value
        calibrator.initialMagnetometerHardIronAvailableListener =
            initialMagnetometerHardIronAvailableListener

        // check
        assertSame(
            initialMagnetometerHardIronAvailableListener,
            calibrator.initialMagnetometerHardIronAvailableListener
        )
    }

    @Test
    fun accuracyChangedListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.accuracyChangedListener)

        // set new value
        calibrator.accuracyChangedListener = accuracyChangedListener

        // check
        assertSame(accuracyChangedListener, calibrator.accuracyChangedListener)
    }

    @Test
    fun isAccelerometerGroundTruthInitialBias_whenValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertFalse(calibrator.isAccelerometerGroundTruthInitialBias)

        // set new value
        calibrator.isAccelerometerGroundTruthInitialBias = true

        // check
        assertTrue(calibrator.isAccelerometerGroundTruthInitialBias)
    }

    @Test(expected = IllegalStateException::class)
    fun isAccelerometerGroundTruthInitialBias_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            accelerometerAndGyroscopeCalibrator,
            "running",
            true
        )
        setPrivateProperty(
            SingleSensorStaticIntervalCalibrator::class,
            magnetometerCalibrator,
            "running",
            true
        )
        assertTrue(calibrator.running)

        calibrator.isAccelerometerGroundTruthInitialBias = true
    }

    @Test
    fun isGyroscopeGroundTruthInitialBias_whenValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertFalse(calibrator.isGyroscopeGroundTruthInitialBias)

        // set new value
        calibrator.isGyroscopeGroundTruthInitialBias = true

        // check
        assertTrue(calibrator.isGyroscopeGroundTruthInitialBias)
    }

    @Test(expected = IllegalStateException::class)
    fun isGyroscopeGroundTruthInitialBias_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            accelerometerAndGyroscopeCalibrator,
            "running",
            true
        )
        setPrivateProperty(
            SingleSensorStaticIntervalCalibrator::class,
            magnetometerCalibrator,
            "running",
            true
        )
        assertTrue(calibrator.running)

        calibrator.isGyroscopeGroundTruthInitialBias = true
    }

    @Test
    fun isMagnetometerGroundTruthInitialHardIron_whenValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertFalse(calibrator.isMagnetometerGroundTruthInitialHardIron)

        // set new value
        calibrator.isMagnetometerGroundTruthInitialHardIron = true

        // check
        assertTrue(calibrator.isMagnetometerGroundTruthInitialHardIron)
    }

    @Test(expected = IllegalStateException::class)
    fun isMagnetometerGroundTruthInitialHardIron_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            accelerometerAndGyroscopeCalibrator,
            "running",
            true
        )
        setPrivateProperty(
            SingleSensorStaticIntervalCalibrator::class,
            magnetometerCalibrator,
            "running",
            true
        )
        assertTrue(calibrator.running)

        calibrator.isMagnetometerGroundTruthInitialHardIron = true
    }

    @Test
    fun accelerometerInitialMa_whenValid_setsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertEquals(Matrix(MA_SIZE, MA_SIZE), calibrator.accelerometerInitialMa)
        assertEquals(0.0, calibrator.accelerometerInitialSx, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialSy, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialSz, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMxy, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMxz, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMyx, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMyz, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMzx, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMzy, 0.0)

        // set new values
        val randomizer = UniformRandomizer()
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()

        val ma = Matrix(MA_SIZE, MA_SIZE)
        ma.setElementAtIndex(0, initialSx)
        ma.setElementAtIndex(1, initialMyx)
        ma.setElementAtIndex(2, initialMzx)

        ma.setElementAtIndex(3, initialMxy)
        ma.setElementAtIndex(4, initialSy)
        ma.setElementAtIndex(5, initialMzy)

        ma.setElementAtIndex(6, initialMxz)
        ma.setElementAtIndex(7, initialMyz)
        ma.setElementAtIndex(8, initialSz)

        calibrator.accelerometerInitialMa = ma

        // check
        assertEquals(ma, calibrator.accelerometerInitialMa)
        assertEquals(initialSx, calibrator.accelerometerInitialSx, 0.0)
        assertEquals(initialSy, calibrator.accelerometerInitialSy, 0.0)
        assertEquals(initialSz, calibrator.accelerometerInitialSz, 0.0)
        assertEquals(initialMxy, calibrator.accelerometerInitialMxy, 0.0)
        assertEquals(initialMxz, calibrator.accelerometerInitialMxz, 0.0)
        assertEquals(initialMyx, calibrator.accelerometerInitialMyx, 0.0)
        assertEquals(initialMyz, calibrator.accelerometerInitialMyz, 0.0)
        assertEquals(initialMzx, calibrator.accelerometerInitialMzx, 0.0)
        assertEquals(initialMzy, calibrator.accelerometerInitialMzy, 0.0)
    }

    @Test(expected = IllegalArgumentException::class)
    fun accelerometerInitialMa_whenInvalidRowsSize_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        val ma = Matrix(1, MA_SIZE)
        calibrator.accelerometerInitialMa = ma
    }

    @Test(expected = IllegalArgumentException::class)
    fun accelerometerInitialMa_whenInvalidColumnsSize_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        val ma = Matrix(MA_SIZE, 1)
        calibrator.accelerometerInitialMa = ma
    }

    @Test(expected = IllegalStateException::class)
    fun accelerometerInitialMa_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            accelerometerAndGyroscopeCalibrator,
            "running",
            true
        )
        setPrivateProperty(
            SingleSensorStaticIntervalCalibrator::class,
            magnetometerCalibrator,
            "running",
            true
        )
        assertTrue(calibrator.running)

        calibrator.accelerometerInitialMa = Matrix(MA_SIZE, MA_SIZE)
    }

    @Test
    fun getAccelerometerInitialMa_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // set new value
        val randomizer = UniformRandomizer()
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setAccelerometerInitialScalingFactorsAndCrossCouplingErrors(
            initialSx,
            initialSy,
            initialSz,
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )

        // check
        val ma = Matrix(MA_SIZE, MA_SIZE)
        calibrator.getAccelerometerInitialMa(ma)

        // check
        assertEquals(initialSx, ma.getElementAtIndex(0), 0.0)
        assertEquals(initialMyx, ma.getElementAtIndex(1), 0.0)
        assertEquals(initialMzx, ma.getElementAtIndex(2), 0.0)

        assertEquals(initialMxy, ma.getElementAtIndex(3), 0.0)
        assertEquals(initialSy, ma.getElementAtIndex(4), 0.0)
        assertEquals(initialMzy, ma.getElementAtIndex(5), 0.0)

        assertEquals(initialMxz, ma.getElementAtIndex(6), 0.0)
        assertEquals(initialMyz, ma.getElementAtIndex(7), 0.0)
        assertEquals(initialSz, ma.getElementAtIndex(8), 0.0)
    }

    @Test(expected = IllegalArgumentException::class)
    fun getAccelerometerInitialMa_whenInvalidRowSize_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        val ma = Matrix(1, MA_SIZE)
        calibrator.getAccelerometerInitialMa(ma)
    }

    @Test(expected = IllegalArgumentException::class)
    fun getAccelerometerInitialMa_whenInvalidColumnSize_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        val ma = Matrix(MA_SIZE, 1)
        calibrator.getAccelerometerInitialMa(ma)
    }

    @Test
    fun accelerometerInitialSx_whenNotRunning_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertEquals(0.0, calibrator.accelerometerInitialSx, 0.0)

        // set new value
        val randomizer = UniformRandomizer()
        val initialSx = randomizer.nextDouble()
        calibrator.accelerometerInitialSx = initialSx

        // check
        assertEquals(initialSx, calibrator.accelerometerInitialSx, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun accelerometerInitialSx_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            accelerometerAndGyroscopeCalibrator,
            "running",
            true
        )
        setPrivateProperty(
            SingleSensorStaticIntervalCalibrator::class,
            magnetometerCalibrator,
            "running",
            true
        )
        assertTrue(calibrator.running)

        calibrator.accelerometerInitialSx = 0.0
    }

    @Test
    fun accelerometerInitialSy_whenNotRunning_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertEquals(0.0, calibrator.accelerometerInitialSy, 0.0)

        // set new value
        val randomizer = UniformRandomizer()
        val initialSy = randomizer.nextDouble()
        calibrator.accelerometerInitialSy = initialSy

        // check
        assertEquals(initialSy, calibrator.accelerometerInitialSy, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun accelerometerInitialSy_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            accelerometerAndGyroscopeCalibrator,
            "running",
            true
        )
        setPrivateProperty(
            SingleSensorStaticIntervalCalibrator::class,
            magnetometerCalibrator,
            "running",
            true
        )
        assertTrue(calibrator.running)

        calibrator.accelerometerInitialSy = 0.0
    }

    @Test
    fun accelerometerInitialSz_whenNotRunning_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertEquals(0.0, calibrator.accelerometerInitialSz, 0.0)

        // set new value
        val randomizer = UniformRandomizer()
        val initialSz = randomizer.nextDouble()
        calibrator.accelerometerInitialSz = initialSz

        // check
        assertEquals(initialSz, calibrator.accelerometerInitialSz, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun accelerometerInitialSz_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            accelerometerAndGyroscopeCalibrator,
            "running",
            true
        )
        setPrivateProperty(
            SingleSensorStaticIntervalCalibrator::class,
            magnetometerCalibrator,
            "running",
            true
        )
        assertTrue(calibrator.running)

        calibrator.accelerometerInitialSz = 0.0
    }

    @Test
    fun accelerometerInitialMxy_whenNotRunning_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertEquals(0.0, calibrator.accelerometerInitialMxy, 0.0)

        // set new value
        val randomizer = UniformRandomizer()
        val initialMxy = randomizer.nextDouble()
        calibrator.accelerometerInitialMxy = initialMxy

        // check
        assertEquals(initialMxy, calibrator.accelerometerInitialMxy, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun accelerometerInitialMxy_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            accelerometerAndGyroscopeCalibrator,
            "running",
            true
        )
        setPrivateProperty(
            SingleSensorStaticIntervalCalibrator::class,
            magnetometerCalibrator,
            "running",
            true
        )
        assertTrue(calibrator.running)

        calibrator.accelerometerInitialMxy = 0.0
    }

    @Test
    fun accelerometerInitialMxz_whenNotRunning_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertEquals(0.0, calibrator.accelerometerInitialMxz, 0.0)

        // set new value
        val randomizer = UniformRandomizer()
        val initialMxz = randomizer.nextDouble()
        calibrator.accelerometerInitialMxz = initialMxz

        // check
        assertEquals(initialMxz, calibrator.accelerometerInitialMxz, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun accelerometerInitialMxz_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            accelerometerAndGyroscopeCalibrator,
            "running",
            true
        )
        setPrivateProperty(
            SingleSensorStaticIntervalCalibrator::class,
            magnetometerCalibrator,
            "running",
            true
        )
        assertTrue(calibrator.running)

        calibrator.accelerometerInitialMxz = 0.0
    }

    @Test
    fun accelerometerInitialMyx_whenNotRunning_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertEquals(0.0, calibrator.accelerometerInitialMyx, 0.0)

        // set new value
        val randomizer = UniformRandomizer()
        val initialMyx = randomizer.nextDouble()
        calibrator.accelerometerInitialMyx = initialMyx

        // check
        assertEquals(initialMyx, calibrator.accelerometerInitialMyx, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun accelerometerInitialMyx_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            accelerometerAndGyroscopeCalibrator,
            "running",
            true
        )
        setPrivateProperty(
            SingleSensorStaticIntervalCalibrator::class,
            magnetometerCalibrator,
            "running",
            true
        )
        assertTrue(calibrator.running)

        calibrator.accelerometerInitialMyx = 0.0
    }

    @Test
    fun accelerometerInitialMyz_whenNotRunning_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertEquals(0.0, calibrator.accelerometerInitialMyz, 0.0)

        // set new value
        val randomizer = UniformRandomizer()
        val initialMyz = randomizer.nextDouble()
        calibrator.accelerometerInitialMyz = initialMyz

        // check
        assertEquals(initialMyz, calibrator.accelerometerInitialMyz, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun accelerometerInitialMyz_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            accelerometerAndGyroscopeCalibrator,
            "running",
            true
        )
        setPrivateProperty(
            SingleSensorStaticIntervalCalibrator::class,
            magnetometerCalibrator,
            "running",
            true
        )
        assertTrue(calibrator.running)

        calibrator.accelerometerInitialMyz = 0.0
    }

    @Test
    fun accelerometerInitialMzx_whenNotRunning_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertEquals(0.0, calibrator.accelerometerInitialMzx, 0.0)

        // set new value
        val randomizer = UniformRandomizer()
        val initialMzx = randomizer.nextDouble()
        calibrator.accelerometerInitialMzx = initialMzx

        // check
        assertEquals(initialMzx, calibrator.accelerometerInitialMzx, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun accelerometerInitialMzx_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            accelerometerAndGyroscopeCalibrator,
            "running",
            true
        )
        setPrivateProperty(
            SingleSensorStaticIntervalCalibrator::class,
            magnetometerCalibrator,
            "running",
            true
        )
        assertTrue(calibrator.running)

        calibrator.accelerometerInitialMzx = 0.0
    }

    @Test
    fun accelerometerInitialMzy_whenNotRunning_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertEquals(0.0, calibrator.accelerometerInitialMzy, 0.0)

        // set new value
        val randomizer = UniformRandomizer()
        val initialMzy = randomizer.nextDouble()
        calibrator.accelerometerInitialMzy = initialMzy

        // check
        assertEquals(initialMzy, calibrator.accelerometerInitialMzy, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun accelerometerInitialMzy_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            accelerometerAndGyroscopeCalibrator,
            "running",
            true
        )
        setPrivateProperty(
            SingleSensorStaticIntervalCalibrator::class,
            magnetometerCalibrator,
            "running",
            true
        )
        assertTrue(calibrator.running)

        calibrator.accelerometerInitialMzy = 0.0
    }

    @Test
    fun setAccelerometerInitialScalingFactors_whenNotRunning_setsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default values
        assertEquals(0.0, calibrator.accelerometerInitialSx, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialSy, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialSz, 0.0)

        // set new value
        val randomizer = UniformRandomizer()
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        calibrator.setAccelerometerInitialScalingFactors(initialSx, initialSy, initialSz)

        // check
        assertEquals(initialSx, calibrator.accelerometerInitialSx, 0.0)
        assertEquals(initialSy, calibrator.accelerometerInitialSy, 0.0)
        assertEquals(initialSz, calibrator.accelerometerInitialSz, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun setAccelerometerInitialScalingFactors_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            accelerometerAndGyroscopeCalibrator,
            "running",
            true
        )
        setPrivateProperty(
            SingleSensorStaticIntervalCalibrator::class,
            magnetometerCalibrator,
            "running",
            true
        )
        assertTrue(calibrator.running)

        val randomizer = UniformRandomizer()
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        calibrator.setAccelerometerInitialScalingFactors(initialSx, initialSy, initialSz)
    }

    @Test
    fun setAccelerometerInitialCrossCouplingErrors_whenNotRunning_setsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default values
        assertEquals(0.0, calibrator.accelerometerInitialMxy, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMxz, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMyx, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMyz, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMzx, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMzy, 0.0)

        // set new value
        val randomizer = UniformRandomizer()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setAccelerometerInitialCrossCouplingErrors(
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )

        // check
        assertEquals(initialMxy, calibrator.accelerometerInitialMxy, 0.0)
        assertEquals(initialMxz, calibrator.accelerometerInitialMxz, 0.0)
        assertEquals(initialMyx, calibrator.accelerometerInitialMyx, 0.0)
        assertEquals(initialMyz, calibrator.accelerometerInitialMyz, 0.0)
        assertEquals(initialMzx, calibrator.accelerometerInitialMzx, 0.0)
        assertEquals(initialMzy, calibrator.accelerometerInitialMzy, 0.0)

        assertEquals(0.0, calibrator.accelerometerInitialSx, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialSy, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialSz, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun setAccelerometerInitialCrossCouplingErrors_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            accelerometerAndGyroscopeCalibrator,
            "running",
            true
        )
        setPrivateProperty(
            SingleSensorStaticIntervalCalibrator::class,
            magnetometerCalibrator,
            "running",
            true
        )
        assertTrue(calibrator.running)

        val randomizer = UniformRandomizer()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setAccelerometerInitialCrossCouplingErrors(
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )
    }

    @Test
    fun setAccelerometerInitialScalingFactorsAndCrossCouplingErrors_whenNotRunning_setsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default values
        assertEquals(0.0, calibrator.accelerometerInitialSx, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialSy, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialSz, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMxy, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMxz, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMyx, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMyz, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMzx, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMzy, 0.0)

        // set new value
        val randomizer = UniformRandomizer()
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setAccelerometerInitialScalingFactorsAndCrossCouplingErrors(
            initialSx,
            initialSy,
            initialSz,
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )

        // check
        assertEquals(initialSx, calibrator.accelerometerInitialSx, 0.0)
        assertEquals(initialSy, calibrator.accelerometerInitialSy, 0.0)
        assertEquals(initialSz, calibrator.accelerometerInitialSz, 0.0)
        assertEquals(initialMxy, calibrator.accelerometerInitialMxy, 0.0)
        assertEquals(initialMxz, calibrator.accelerometerInitialMxz, 0.0)
        assertEquals(initialMyx, calibrator.accelerometerInitialMyx, 0.0)
        assertEquals(initialMyz, calibrator.accelerometerInitialMyz, 0.0)
        assertEquals(initialMzx, calibrator.accelerometerInitialMzx, 0.0)
        assertEquals(initialMzy, calibrator.accelerometerInitialMzy, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun setAccelerometerInitialScalingFactorsAndCrossCouplingErrors_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            accelerometerAndGyroscopeCalibrator,
            "running",
            true
        )
        setPrivateProperty(
            SingleSensorStaticIntervalCalibrator::class,
            magnetometerCalibrator,
            "running",
            true
        )
        assertTrue(calibrator.running)

        val randomizer = UniformRandomizer()
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setAccelerometerInitialScalingFactorsAndCrossCouplingErrors(
            initialSx,
            initialSy,
            initialSz,
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )
    }

    @Test
    fun gyroscopeInitialMg_whenNotRunning_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        val mg1 = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        val mg2 = calibrator.gyroscopeInitialMg
        assertEquals(mg1, mg2)

        // set new value
        val mg3 = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        Matrix.fillWithUniformRandomValues(0.0, 1.0, mg3)
        calibrator.gyroscopeInitialMg = mg3

        // check
        val mg4 = calibrator.gyroscopeInitialMg
        assertEquals(mg3, mg4)
        assertEquals(mg3.getElementAt(0, 0), calibrator.gyroscopeInitialSx, 0.0)
        assertEquals(mg3.getElementAt(1, 0), calibrator.gyroscopeInitialMyx, 0.0)
        assertEquals(mg3.getElementAt(2, 0), calibrator.gyroscopeInitialMzx, 0.0)
        assertEquals(mg3.getElementAt(0, 1), calibrator.gyroscopeInitialMxy, 0.0)
        assertEquals(mg3.getElementAt(1, 1), calibrator.gyroscopeInitialSy, 0.0)
        assertEquals(mg3.getElementAt(2, 1), calibrator.gyroscopeInitialMzy, 0.0)
        assertEquals(mg3.getElementAt(0, 2), calibrator.gyroscopeInitialMxz, 0.0)
        assertEquals(mg3.getElementAt(1, 2), calibrator.gyroscopeInitialMyz, 0.0)
        assertEquals(mg3.getElementAt(2, 2), calibrator.gyroscopeInitialSz, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun gyroscopeInitialMg_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            accelerometerAndGyroscopeCalibrator,
            "running",
            true
        )
        setPrivateProperty(
            SingleSensorStaticIntervalCalibrator::class,
            magnetometerCalibrator,
            "running",
            true
        )
        assertTrue(calibrator.running)

        val mg1 = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        calibrator.gyroscopeInitialMg = mg1
    }

    @Test(expected = IllegalArgumentException::class)
    fun gyroscopeInitialMg_whenInvalidRows_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        val mg1 = Matrix(1, BodyKinematics.COMPONENTS)
        calibrator.gyroscopeInitialMg = mg1
    }

    @Test(expected = IllegalArgumentException::class)
    fun gyroscopeInitialMg_whenInvalidColumns_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        val mg1 = Matrix(BodyKinematics.COMPONENTS, 1)
        calibrator.gyroscopeInitialMg = mg1
    }

    @Test
    fun getGyroscopeInitialMg_whenValid_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        val mg1 = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        val mg2 = calibrator.gyroscopeInitialMg
        assertEquals(mg1, mg2)

        // set new value
        val mg3 = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        Matrix.fillWithUniformRandomValues(0.0, 1.0, mg3)
        calibrator.gyroscopeInitialMg = mg3

        // check
        val mg4 = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        calibrator.getGyroscopeInitialMg(mg4)
        assertEquals(mg3, mg4)
    }

    @Test(expected = IllegalArgumentException::class)
    fun getGyroscopeInitialMg_whenInvalidRows_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // set new value
        val mg1 = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        Matrix.fillWithUniformRandomValues(0.0, 1.0, mg1)
        calibrator.gyroscopeInitialMg = mg1

        // get
        val mg2 = Matrix(1, BodyKinematics.COMPONENTS)
        calibrator.getGyroscopeInitialMg(mg2)
    }

    @Test(expected = IllegalArgumentException::class)
    fun getGyroscopeInitialMg_whenInvalidColumns_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // set new value
        val mg1 = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        Matrix.fillWithUniformRandomValues(0.0, 1.0, mg1)
        calibrator.gyroscopeInitialMg = mg1

        // get
        val mg2 = Matrix(BodyKinematics.COMPONENTS, 1)
        calibrator.getGyroscopeInitialMg(mg2)
    }

    @Test
    fun gyroscopeInitialSx_whenNotRunning_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertEquals(0.0, calibrator.gyroscopeInitialSx, 0.0)
        assertFalse(calibrator.running)

        // set new value
        val randomizer = UniformRandomizer()
        val gyroscopeInitialSx = randomizer.nextDouble()
        calibrator.gyroscopeInitialSx = gyroscopeInitialSx

        // check
        assertEquals(gyroscopeInitialSx, calibrator.gyroscopeInitialSx, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun gyroscopeInitialSx_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            accelerometerAndGyroscopeCalibrator,
            "running",
            true
        )
        setPrivateProperty(
            SingleSensorStaticIntervalCalibrator::class,
            magnetometerCalibrator,
            "running",
            true
        )
        assertTrue(calibrator.running)

        // set new value
        val randomizer = UniformRandomizer()
        val gyroscopeInitialSx = randomizer.nextDouble()
        calibrator.gyroscopeInitialSx = gyroscopeInitialSx
    }

    @Test
    fun gyroscopeInitialSy_whenNotRunning_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertEquals(0.0, calibrator.gyroscopeInitialSx, 0.0)
        assertFalse(calibrator.running)

        // set new value
        val randomizer = UniformRandomizer()
        val gyroscopeInitialSy = randomizer.nextDouble()
        calibrator.gyroscopeInitialSy = gyroscopeInitialSy

        // check
        assertEquals(gyroscopeInitialSy, calibrator.gyroscopeInitialSy, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun gyroscopeInitialSy_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            accelerometerAndGyroscopeCalibrator,
            "running",
            true
        )
        setPrivateProperty(
            SingleSensorStaticIntervalCalibrator::class,
            magnetometerCalibrator,
            "running",
            true
        )
        assertTrue(calibrator.running)

        // set new value
        val randomizer = UniformRandomizer()
        val gyroscopeInitialSy = randomizer.nextDouble()
        calibrator.gyroscopeInitialSy = gyroscopeInitialSy
    }

    @Test
    fun gyroscopeInitialSz_whenNotRunning_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertEquals(0.0, calibrator.gyroscopeInitialSx, 0.0)
        assertFalse(calibrator.running)

        // set new value
        val randomizer = UniformRandomizer()
        val gyroscopeInitialSz = randomizer.nextDouble()
        calibrator.gyroscopeInitialSz = gyroscopeInitialSz

        // check
        assertEquals(gyroscopeInitialSz, calibrator.gyroscopeInitialSz, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun gyroscopeInitialSz_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            accelerometerAndGyroscopeCalibrator,
            "running",
            true
        )
        setPrivateProperty(
            SingleSensorStaticIntervalCalibrator::class,
            magnetometerCalibrator,
            "running",
            true
        )
        assertTrue(calibrator.running)

        // set new value
        val randomizer = UniformRandomizer()
        val gyroscopeInitialSz = randomizer.nextDouble()
        calibrator.gyroscopeInitialSz = gyroscopeInitialSz
    }

    @Test
    fun gyroscopeInitialMxy_whenNotRunning_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertEquals(0.0, calibrator.gyroscopeInitialMxy, 0.0)
        assertFalse(calibrator.running)

        // set new value
        val randomizer = UniformRandomizer()
        val gyroscopeInitialMxy = randomizer.nextDouble()
        calibrator.gyroscopeInitialMxy = gyroscopeInitialMxy

        // check
        assertEquals(gyroscopeInitialMxy, calibrator.gyroscopeInitialMxy, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun gyroscopeInitialMxy_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            accelerometerAndGyroscopeCalibrator,
            "running",
            true
        )
        setPrivateProperty(
            SingleSensorStaticIntervalCalibrator::class,
            magnetometerCalibrator,
            "running",
            true
        )
        assertTrue(calibrator.running)

        // set new value
        val randomizer = UniformRandomizer()
        val gyroscopeInitialMxy = randomizer.nextDouble()
        calibrator.gyroscopeInitialMxy = gyroscopeInitialMxy
    }

    @Test
    fun gyroscopeInitialMxz_whenNotRunning_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertEquals(0.0, calibrator.gyroscopeInitialMxz, 0.0)
        assertFalse(calibrator.running)

        // set new value
        val randomizer = UniformRandomizer()
        val gyroscopeInitialMxz = randomizer.nextDouble()
        calibrator.gyroscopeInitialMxz = gyroscopeInitialMxz

        // check
        assertEquals(gyroscopeInitialMxz, calibrator.gyroscopeInitialMxz, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun gyroscopeInitialMxz_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            accelerometerAndGyroscopeCalibrator,
            "running",
            true
        )
        setPrivateProperty(
            SingleSensorStaticIntervalCalibrator::class,
            magnetometerCalibrator,
            "running",
            true
        )
        assertTrue(calibrator.running)

        // set new value
        val randomizer = UniformRandomizer()
        val gyroscopeInitialMxz = randomizer.nextDouble()
        calibrator.gyroscopeInitialMxz = gyroscopeInitialMxz
    }

    @Test
    fun gyroscopeInitialMyx_whenNotRunning_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertEquals(0.0, calibrator.gyroscopeInitialMyx, 0.0)
        assertFalse(calibrator.running)

        // set new value
        val randomizer = UniformRandomizer()
        val gyroscopeInitialMyx = randomizer.nextDouble()
        calibrator.gyroscopeInitialMyx = gyroscopeInitialMyx

        // check
        assertEquals(gyroscopeInitialMyx, calibrator.gyroscopeInitialMyx, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun gyroscopeInitialMyx_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            accelerometerAndGyroscopeCalibrator,
            "running",
            true
        )
        setPrivateProperty(
            SingleSensorStaticIntervalCalibrator::class,
            magnetometerCalibrator,
            "running",
            true
        )
        assertTrue(calibrator.running)

        // set new value
        val randomizer = UniformRandomizer()
        val gyroscopeInitialMyx = randomizer.nextDouble()
        calibrator.gyroscopeInitialMyx = gyroscopeInitialMyx
    }

    @Test
    fun gyroscopeInitialMyz_whenNotRunning_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertEquals(0.0, calibrator.gyroscopeInitialMyz, 0.0)
        assertFalse(calibrator.running)

        // set new value
        val randomizer = UniformRandomizer()
        val gyroscopeInitialMyz = randomizer.nextDouble()
        calibrator.gyroscopeInitialMyz = gyroscopeInitialMyz

        // check
        assertEquals(gyroscopeInitialMyz, calibrator.gyroscopeInitialMyz, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun gyroscopeInitialMyz_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            accelerometerAndGyroscopeCalibrator,
            "running",
            true
        )
        setPrivateProperty(
            SingleSensorStaticIntervalCalibrator::class,
            magnetometerCalibrator,
            "running",
            true
        )
        assertTrue(calibrator.running)

        // set new value
        val randomizer = UniformRandomizer()
        val gyroscopeInitialMyz = randomizer.nextDouble()
        calibrator.gyroscopeInitialMyz = gyroscopeInitialMyz
    }

    @Test
    fun gyroscopeInitialMzx_whenNotRunning_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertEquals(0.0, calibrator.gyroscopeInitialMzx, 0.0)
        assertFalse(calibrator.running)

        // set new value
        val randomizer = UniformRandomizer()
        val gyroscopeInitialMzx = randomizer.nextDouble()
        calibrator.gyroscopeInitialMzx = gyroscopeInitialMzx

        // check
        assertEquals(gyroscopeInitialMzx, calibrator.gyroscopeInitialMzx, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun gyroscopeInitialMzx_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            accelerometerAndGyroscopeCalibrator,
            "running",
            true
        )
        setPrivateProperty(
            SingleSensorStaticIntervalCalibrator::class,
            magnetometerCalibrator,
            "running",
            true
        )
        assertTrue(calibrator.running)

        // set new value
        val randomizer = UniformRandomizer()
        val gyroscopeInitialMzx = randomizer.nextDouble()
        calibrator.gyroscopeInitialMzx = gyroscopeInitialMzx
    }

    @Test
    fun gyroscopeInitialMzy_whenNotRunning_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertEquals(0.0, calibrator.gyroscopeInitialMzy, 0.0)
        assertFalse(calibrator.running)

        // set new value
        val randomizer = UniformRandomizer()
        val gyroscopeInitialMzy = randomizer.nextDouble()
        calibrator.gyroscopeInitialMzy = gyroscopeInitialMzy

        // check
        assertEquals(gyroscopeInitialMzy, calibrator.gyroscopeInitialMzy, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun gyroscopeInitialMzy_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            accelerometerAndGyroscopeCalibrator,
            "running",
            true
        )
        setPrivateProperty(
            SingleSensorStaticIntervalCalibrator::class,
            magnetometerCalibrator,
            "running",
            true
        )
        assertTrue(calibrator.running)

        // set new value
        val randomizer = UniformRandomizer()
        val gyroscopeInitialMzy = randomizer.nextDouble()
        calibrator.gyroscopeInitialMzy = gyroscopeInitialMzy
    }

    @Test
    fun setGyroscopeInitialScalingFactors_whenNotRunning_setsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default values
        assertEquals(0.0, calibrator.gyroscopeInitialSx, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialSy, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialSz, 0.0)
        assertFalse(calibrator.running)

        // set new values
        val randomizer = UniformRandomizer()
        val gyroscopeInitialSx = randomizer.nextDouble()
        val gyroscopeInitialSy = randomizer.nextDouble()
        val gyroscopeInitialSz = randomizer.nextDouble()
        calibrator.setGyroscopeInitialScalingFactors(
            gyroscopeInitialSx,
            gyroscopeInitialSy,
            gyroscopeInitialSz
        )

        // check
        assertEquals(gyroscopeInitialSx, calibrator.gyroscopeInitialSx, 0.0)
        assertEquals(gyroscopeInitialSy, calibrator.gyroscopeInitialSy, 0.0)
        assertEquals(gyroscopeInitialSz, calibrator.gyroscopeInitialSz, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun setGyroscopeInitialScalingFactors_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            accelerometerAndGyroscopeCalibrator,
            "running",
            true
        )
        setPrivateProperty(
            SingleSensorStaticIntervalCalibrator::class,
            magnetometerCalibrator,
            "running",
            true
        )
        assertTrue(calibrator.running)

        // set new values
        val randomizer = UniformRandomizer()
        val gyroscopeInitialSx = randomizer.nextDouble()
        val gyroscopeInitialSy = randomizer.nextDouble()
        val gyroscopeInitialSz = randomizer.nextDouble()
        calibrator.setGyroscopeInitialScalingFactors(
            gyroscopeInitialSx,
            gyroscopeInitialSy,
            gyroscopeInitialSz
        )
    }

    @Test
    fun setGyroscopeInitialCrossCouplingErrors_whenNotRunning_setsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default values
        assertEquals(0.0, calibrator.gyroscopeInitialMxy, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMxz, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMyx, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMyz, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMzx, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMzy, 0.0)
        assertFalse(calibrator.running)

        // set new values
        val randomizer = UniformRandomizer()
        val gyroscopeInitialMxy = randomizer.nextDouble()
        val gyroscopeInitialMxz = randomizer.nextDouble()
        val gyroscopeInitialMyx = randomizer.nextDouble()
        val gyroscopeInitialMyz = randomizer.nextDouble()
        val gyroscopeInitialMzx = randomizer.nextDouble()
        val gyroscopeInitialMzy = randomizer.nextDouble()
        calibrator.setGyroscopeInitialCrossCouplingErrors(
            gyroscopeInitialMxy,
            gyroscopeInitialMxz,
            gyroscopeInitialMyx,
            gyroscopeInitialMyz,
            gyroscopeInitialMzx,
            gyroscopeInitialMzy
        )

        // check
        assertEquals(gyroscopeInitialMxy, calibrator.gyroscopeInitialMxy, 0.0)
        assertEquals(gyroscopeInitialMxz, calibrator.gyroscopeInitialMxz, 0.0)
        assertEquals(gyroscopeInitialMyx, calibrator.gyroscopeInitialMyx, 0.0)
        assertEquals(gyroscopeInitialMyz, calibrator.gyroscopeInitialMyz, 0.0)
        assertEquals(gyroscopeInitialMzx, calibrator.gyroscopeInitialMzx, 0.0)
        assertEquals(gyroscopeInitialMzy, calibrator.gyroscopeInitialMzy, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun setGyroscopeInitialCrossCouplingErrors_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            accelerometerAndGyroscopeCalibrator,
            "running",
            true
        )
        setPrivateProperty(
            SingleSensorStaticIntervalCalibrator::class,
            magnetometerCalibrator,
            "running",
            true
        )
        assertTrue(calibrator.running)

        // set new values
        val randomizer = UniformRandomizer()
        val gyroscopeInitialMxy = randomizer.nextDouble()
        val gyroscopeInitialMxz = randomizer.nextDouble()
        val gyroscopeInitialMyx = randomizer.nextDouble()
        val gyroscopeInitialMyz = randomizer.nextDouble()
        val gyroscopeInitialMzx = randomizer.nextDouble()
        val gyroscopeInitialMzy = randomizer.nextDouble()
        calibrator.setGyroscopeInitialCrossCouplingErrors(
            gyroscopeInitialMxy,
            gyroscopeInitialMxz,
            gyroscopeInitialMyx,
            gyroscopeInitialMyz,
            gyroscopeInitialMzx,
            gyroscopeInitialMzy
        )
    }

    @Test
    fun setGyroscopeInitialScalingFactorsAndCrossCouplingErrors_whenNotRunning_setsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default values
        assertEquals(0.0, calibrator.gyroscopeInitialSx, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialSy, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialSz, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMxy, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMxz, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMyx, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMyz, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMzx, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMzy, 0.0)
        assertFalse(calibrator.running)

        // set new values
        val randomizer = UniformRandomizer()
        val gyroscopeInitialSx = randomizer.nextDouble()
        val gyroscopeInitialSy = randomizer.nextDouble()
        val gyroscopeInitialSz = randomizer.nextDouble()
        val gyroscopeInitialMxy = randomizer.nextDouble()
        val gyroscopeInitialMxz = randomizer.nextDouble()
        val gyroscopeInitialMyx = randomizer.nextDouble()
        val gyroscopeInitialMyz = randomizer.nextDouble()
        val gyroscopeInitialMzx = randomizer.nextDouble()
        val gyroscopeInitialMzy = randomizer.nextDouble()
        calibrator.setGyroscopeInitialScalingFactorsAndCrossCouplingErrors(
            gyroscopeInitialSx,
            gyroscopeInitialSy,
            gyroscopeInitialSz,
            gyroscopeInitialMxy,
            gyroscopeInitialMxz,
            gyroscopeInitialMyx,
            gyroscopeInitialMyz,
            gyroscopeInitialMzx,
            gyroscopeInitialMzy
        )

        // check
        assertEquals(gyroscopeInitialSx, calibrator.gyroscopeInitialSx, 0.0)
        assertEquals(gyroscopeInitialSy, calibrator.gyroscopeInitialSy, 0.0)
        assertEquals(gyroscopeInitialSz, calibrator.gyroscopeInitialSz, 0.0)
        assertEquals(gyroscopeInitialMxy, calibrator.gyroscopeInitialMxy, 0.0)
        assertEquals(gyroscopeInitialMxz, calibrator.gyroscopeInitialMxz, 0.0)
        assertEquals(gyroscopeInitialMyx, calibrator.gyroscopeInitialMyx, 0.0)
        assertEquals(gyroscopeInitialMyz, calibrator.gyroscopeInitialMyz, 0.0)
        assertEquals(gyroscopeInitialMzx, calibrator.gyroscopeInitialMzx, 0.0)
        assertEquals(gyroscopeInitialMzy, calibrator.gyroscopeInitialMzy, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun setGyroscopeInitialScalingFactorsAndCrossCouplingErrors_whenRunning_setsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            accelerometerAndGyroscopeCalibrator,
            "running",
            true
        )
        setPrivateProperty(
            SingleSensorStaticIntervalCalibrator::class,
            magnetometerCalibrator,
            "running",
            true
        )
        assertTrue(calibrator.running)

        // set new values
        val randomizer = UniformRandomizer()
        val gyroscopeInitialSx = randomizer.nextDouble()
        val gyroscopeInitialSy = randomizer.nextDouble()
        val gyroscopeInitialSz = randomizer.nextDouble()
        val gyroscopeInitialMxy = randomizer.nextDouble()
        val gyroscopeInitialMxz = randomizer.nextDouble()
        val gyroscopeInitialMyx = randomizer.nextDouble()
        val gyroscopeInitialMyz = randomizer.nextDouble()
        val gyroscopeInitialMzx = randomizer.nextDouble()
        val gyroscopeInitialMzy = randomizer.nextDouble()
        calibrator.setGyroscopeInitialScalingFactorsAndCrossCouplingErrors(
            gyroscopeInitialSx,
            gyroscopeInitialSy,
            gyroscopeInitialSz,
            gyroscopeInitialMxy,
            gyroscopeInitialMxz,
            gyroscopeInitialMyx,
            gyroscopeInitialMyz,
            gyroscopeInitialMzx,
            gyroscopeInitialMzy
        )
    }

    @Test
    fun gyroscopeInitialGg_whenNotRunning_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        val gg1 = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        val gg2 = calibrator.gyroscopeInitialGg
        assertEquals(gg1, gg2)

        // set new value
        val gg3 = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        Matrix.fillWithUniformRandomValues(0.0, 1.0, gg3)
        calibrator.gyroscopeInitialGg = gg3

        // check
        val gg4 = calibrator.gyroscopeInitialGg
        assertEquals(gg3, gg4)
    }

    @Test(expected = IllegalStateException::class)
    fun gyroscopeInitialGg_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            accelerometerAndGyroscopeCalibrator,
            "running",
            true
        )
        setPrivateProperty(
            SingleSensorStaticIntervalCalibrator::class,
            magnetometerCalibrator,
            "running",
            true
        )
        assertTrue(calibrator.running)

        // set new value
        val gg = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        Matrix.fillWithUniformRandomValues(0.0, 1.0, gg)
        calibrator.gyroscopeInitialGg = gg
    }

    @Test(expected = IllegalArgumentException::class)
    fun gyroscopeInitialGg_whenInvalidRows_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // set new value
        val gg = Matrix(1, BodyKinematics.COMPONENTS)
        Matrix.fillWithUniformRandomValues(0.0, 1.0, gg)
        calibrator.gyroscopeInitialGg = gg
    }

    @Test(expected = IllegalArgumentException::class)
    fun gyroscopeInitialGg_whenInvalidColumns_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // set new value
        val gg = Matrix(BodyKinematics.COMPONENTS, 1)
        Matrix.fillWithUniformRandomValues(0.0, 1.0, gg)
        calibrator.gyroscopeInitialGg = gg
    }

    @Test
    fun magnetometerInitialMm_whenValid_setsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertEquals(Matrix(MM_SIZE, MM_SIZE), calibrator.magnetometerInitialMm)
        assertEquals(0.0, calibrator.magnetometerInitialSx, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialSy, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialSz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMxy, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMxz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMyx, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMyz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMzx, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMzy, 0.0)

        // set new value
        val randomizer = UniformRandomizer()
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()

        val mm = Matrix(MM_SIZE, MM_SIZE)
        mm.setElementAtIndex(0, initialSx)
        mm.setElementAtIndex(1, initialMyx)
        mm.setElementAtIndex(2, initialMzx)

        mm.setElementAtIndex(3, initialMxy)
        mm.setElementAtIndex(4, initialSy)
        mm.setElementAtIndex(5, initialMzy)

        mm.setElementAtIndex(6, initialMxz)
        mm.setElementAtIndex(7, initialMyz)
        mm.setElementAtIndex(8, initialSz)

        calibrator.magnetometerInitialMm = mm

        // check
        assertEquals(mm, calibrator.magnetometerInitialMm)
        assertEquals(initialSx, calibrator.magnetometerInitialSx, 0.0)
        assertEquals(initialSy, calibrator.magnetometerInitialSy, 0.0)
        assertEquals(initialSz, calibrator.magnetometerInitialSz, 0.0)
        assertEquals(initialMxy, calibrator.magnetometerInitialMxy, 0.0)
        assertEquals(initialMxz, calibrator.magnetometerInitialMxz, 0.0)
        assertEquals(initialMyx, calibrator.magnetometerInitialMyx, 0.0)
        assertEquals(initialMyz, calibrator.magnetometerInitialMyz, 0.0)
        assertEquals(initialMzx, calibrator.magnetometerInitialMzx, 0.0)
        assertEquals(initialMzy, calibrator.magnetometerInitialMzy, 0.0)
    }

    @Test(expected = IllegalArgumentException::class)
    fun magnetometerInitialMm_whenInvalidRowSize_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        val mm = Matrix(1, MM_SIZE)
        calibrator.magnetometerInitialMm = mm
    }

    @Test(expected = IllegalArgumentException::class)
    fun magnetometerInitialMm_whenInvalidColumnsSize_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        val mm = Matrix(MM_SIZE, 1)
        calibrator.magnetometerInitialMm = mm
    }

    @Test(expected = IllegalStateException::class)
    fun magnetometerInitialMm_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            accelerometerAndGyroscopeCalibrator,
            "running",
            true
        )
        setPrivateProperty(
            SingleSensorStaticIntervalCalibrator::class,
            magnetometerCalibrator,
            "running",
            true
        )
        assertTrue(calibrator.running)

        calibrator.magnetometerInitialMm = Matrix(MM_SIZE, MM_SIZE)
    }

    @Test
    fun getMagnetometerInitialMm_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // set new value
        val randomizer = UniformRandomizer()
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setMagnetometerInitialScalingFactorsAndCrossCouplingErrors(
            initialSx,
            initialSy,
            initialSz,
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )

        // check
        val mm = Matrix(MM_SIZE, MM_SIZE)
        calibrator.getMagnetometerInitialMm(mm)

        // check
        assertEquals(initialSx, mm.getElementAtIndex(0), 0.0)
        assertEquals(initialMyx, mm.getElementAtIndex(1), 0.0)
        assertEquals(initialMzx, mm.getElementAtIndex(2), 0.0)

        assertEquals(initialMxy, mm.getElementAtIndex(3), 0.0)
        assertEquals(initialSy, mm.getElementAtIndex(4), 0.0)
        assertEquals(initialMzy, mm.getElementAtIndex(5), 0.0)

        assertEquals(initialMxz, mm.getElementAtIndex(6), 0.0)
        assertEquals(initialMyz, mm.getElementAtIndex(7), 0.0)
        assertEquals(initialSz, mm.getElementAtIndex(8), 0.0)
    }

    @Test(expected = IllegalArgumentException::class)
    fun getMagnetometerInitialMm_whenInvalidRowSize_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context)

        val mm = Matrix(1, MM_SIZE)
        calibrator.getMagnetometerInitialMm(mm)
    }

    @Test(expected = IllegalArgumentException::class)
    fun getMagnetometerInitialMm_whenInvalidColumnSize_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context)

        val mm = Matrix(MM_SIZE, 1)
        calibrator.getMagnetometerInitialMm(mm)
    }

    @Test
    fun magnetometerInitialSx_whenNotRunning_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertEquals(0.0, calibrator.magnetometerInitialSx, 0.0)
        assertFalse(calibrator.running)

        // set new value
        val randomizer = UniformRandomizer()
        val magnetometerInitialSx = randomizer.nextDouble()
        calibrator.magnetometerInitialSx = magnetometerInitialSx

        // check
        assertEquals(magnetometerInitialSx, calibrator.magnetometerInitialSx, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun magnetometerInitialSx_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            accelerometerAndGyroscopeCalibrator,
            "running",
            true
        )
        setPrivateProperty(
            SingleSensorStaticIntervalCalibrator::class,
            magnetometerCalibrator,
            "running",
            true
        )
        assertTrue(calibrator.running)

        // set new value
        val randomizer = UniformRandomizer()
        val magnetometerInitialSx = randomizer.nextDouble()
        calibrator.magnetometerInitialSx = magnetometerInitialSx
    }

    @Test
    fun magnetometerInitialSy_whenNotRunning_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertEquals(0.0, calibrator.magnetometerInitialSy, 0.0)
        assertFalse(calibrator.running)

        // set new value
        val randomizer = UniformRandomizer()
        val magnetometerInitialSy = randomizer.nextDouble()
        calibrator.magnetometerInitialSy = magnetometerInitialSy

        // check
        assertEquals(magnetometerInitialSy, calibrator.magnetometerInitialSy, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun magnetometerInitialSy_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            accelerometerAndGyroscopeCalibrator,
            "running",
            true
        )
        setPrivateProperty(
            SingleSensorStaticIntervalCalibrator::class,
            magnetometerCalibrator,
            "running",
            true
        )
        assertTrue(calibrator.running)

        // set new value
        val randomizer = UniformRandomizer()
        val magnetometerInitialSy = randomizer.nextDouble()
        calibrator.magnetometerInitialSy = magnetometerInitialSy
    }

    @Test
    fun magnetometerInitialSz_whenNotRunning_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertEquals(0.0, calibrator.magnetometerInitialSz, 0.0)
        assertFalse(calibrator.running)

        // set new value
        val randomizer = UniformRandomizer()
        val magnetometerInitialSz = randomizer.nextDouble()
        calibrator.magnetometerInitialSz = magnetometerInitialSz

        // check
        assertEquals(magnetometerInitialSz, calibrator.magnetometerInitialSz, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun magnetometerInitialSz_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            accelerometerAndGyroscopeCalibrator,
            "running",
            true
        )
        setPrivateProperty(
            SingleSensorStaticIntervalCalibrator::class,
            magnetometerCalibrator,
            "running",
            true
        )
        assertTrue(calibrator.running)

        // set new value
        val randomizer = UniformRandomizer()
        val magnetometerInitialSz = randomizer.nextDouble()
        calibrator.magnetometerInitialSz = magnetometerInitialSz
    }

    @Test
    fun magnetometerInitialMxy_whenNotRunning_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertEquals(0.0, calibrator.magnetometerInitialMxy, 0.0)
        assertFalse(calibrator.running)

        // set new value
        val randomizer = UniformRandomizer()
        val magnetometerInitialMxy = randomizer.nextDouble()
        calibrator.magnetometerInitialMxy = magnetometerInitialMxy

        // check
        assertEquals(magnetometerInitialMxy, calibrator.magnetometerInitialMxy, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun magnetometerInitialMxy_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            accelerometerAndGyroscopeCalibrator,
            "running",
            true
        )
        setPrivateProperty(
            SingleSensorStaticIntervalCalibrator::class,
            magnetometerCalibrator,
            "running",
            true
        )
        assertTrue(calibrator.running)

        // set new value
        val randomizer = UniformRandomizer()
        val magnetometerInitialMxy = randomizer.nextDouble()
        calibrator.magnetometerInitialMxy = magnetometerInitialMxy
    }

    @Test
    fun magnetometerInitialMxz_whenNotRunning_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertEquals(0.0, calibrator.magnetometerInitialMxz, 0.0)
        assertFalse(calibrator.running)

        // set new value
        val randomizer = UniformRandomizer()
        val magnetometerInitialMxz = randomizer.nextDouble()
        calibrator.magnetometerInitialMxz = magnetometerInitialMxz

        // check
        assertEquals(magnetometerInitialMxz, calibrator.magnetometerInitialMxz, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun magnetometerInitialMxz_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            accelerometerAndGyroscopeCalibrator,
            "running",
            true
        )
        setPrivateProperty(
            SingleSensorStaticIntervalCalibrator::class,
            magnetometerCalibrator,
            "running",
            true
        )
        assertTrue(calibrator.running)

        // set new value
        val randomizer = UniformRandomizer()
        val magnetometerInitialMxz = randomizer.nextDouble()
        calibrator.magnetometerInitialMxz = magnetometerInitialMxz
    }

    @Test
    fun magnetometerInitialMyx_whenNotRunning_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertEquals(0.0, calibrator.magnetometerInitialMxz, 0.0)
        assertFalse(calibrator.running)

        // set new value
        val randomizer = UniformRandomizer()
        val magnetometerInitialMyx = randomizer.nextDouble()
        calibrator.magnetometerInitialMyx = magnetometerInitialMyx

        // check
        assertEquals(magnetometerInitialMyx, calibrator.magnetometerInitialMyx, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun magnetometerInitialMyx_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            accelerometerAndGyroscopeCalibrator,
            "running",
            true
        )
        setPrivateProperty(
            SingleSensorStaticIntervalCalibrator::class,
            magnetometerCalibrator,
            "running",
            true
        )
        assertTrue(calibrator.running)

        // set new value
        val randomizer = UniformRandomizer()
        val magnetometerInitialMyx = randomizer.nextDouble()
        calibrator.magnetometerInitialMyx = magnetometerInitialMyx
    }

    @Test
    fun magnetometerInitialMyz_whenNotRunning_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertEquals(0.0, calibrator.magnetometerInitialMyz, 0.0)
        assertFalse(calibrator.running)

        // set new value
        val randomizer = UniformRandomizer()
        val magnetometerInitialMyz = randomizer.nextDouble()
        calibrator.magnetometerInitialMyz = magnetometerInitialMyz

        // check
        assertEquals(magnetometerInitialMyz, calibrator.magnetometerInitialMyz, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun magnetometerInitialMyz_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            accelerometerAndGyroscopeCalibrator,
            "running",
            true
        )
        setPrivateProperty(
            SingleSensorStaticIntervalCalibrator::class,
            magnetometerCalibrator,
            "running",
            true
        )
        assertTrue(calibrator.running)

        // set new value
        val randomizer = UniformRandomizer()
        val magnetometerInitialMyz = randomizer.nextDouble()
        calibrator.magnetometerInitialMyz = magnetometerInitialMyz
    }

    @Test
    fun magnetometerInitialMzx_whenNotRunning_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertEquals(0.0, calibrator.magnetometerInitialMzx, 0.0)
        assertFalse(calibrator.running)

        // set new value
        val randomizer = UniformRandomizer()
        val magnetometerInitialMzx = randomizer.nextDouble()
        calibrator.magnetometerInitialMzx = magnetometerInitialMzx

        // check
        assertEquals(magnetometerInitialMzx, calibrator.magnetometerInitialMzx, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun magnetometerInitialMzx_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            accelerometerAndGyroscopeCalibrator,
            "running",
            true
        )
        setPrivateProperty(
            SingleSensorStaticIntervalCalibrator::class,
            magnetometerCalibrator,
            "running",
            true
        )
        assertTrue(calibrator.running)

        // set new value
        val randomizer = UniformRandomizer()
        val magnetometerInitialMzx = randomizer.nextDouble()
        calibrator.magnetometerInitialMzx = magnetometerInitialMzx
    }

    @Test
    fun magnetometerInitialMzy_whenNotRunning_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertEquals(0.0, calibrator.magnetometerInitialMzy, 0.0)
        assertFalse(calibrator.running)

        // set new value
        val randomizer = UniformRandomizer()
        val magnetometerInitialMzy = randomizer.nextDouble()
        calibrator.magnetometerInitialMzy = magnetometerInitialMzy

        // check
        assertEquals(magnetometerInitialMzy, calibrator.magnetometerInitialMzy, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun magnetometerInitialMzy_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            accelerometerAndGyroscopeCalibrator,
            "running",
            true
        )
        setPrivateProperty(
            SingleSensorStaticIntervalCalibrator::class,
            magnetometerCalibrator,
            "running",
            true
        )
        assertTrue(calibrator.running)

        // set new value
        val randomizer = UniformRandomizer()
        val magnetometerInitialMzy = randomizer.nextDouble()
        calibrator.magnetometerInitialMzy = magnetometerInitialMzy
    }

    @Test
    fun setMagnetometerInitialScalingFactors_whenNotRunning_setsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default values
        assertEquals(0.0, calibrator.magnetometerInitialSx, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialSy, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialSz, 0.0)
        assertFalse(calibrator.running)

        // set new values
        val randomizer = UniformRandomizer()
        val magnetometerInitialSx = randomizer.nextDouble()
        val magnetometerInitialSy = randomizer.nextDouble()
        val magnetometerInitialSz = randomizer.nextDouble()
        calibrator.setMagnetometerInitialScalingFactors(
            magnetometerInitialSx,
            magnetometerInitialSy,
            magnetometerInitialSz
        )

        // check
        assertEquals(magnetometerInitialSx, calibrator.magnetometerInitialSx, 0.0)
        assertEquals(magnetometerInitialSy, calibrator.magnetometerInitialSy, 0.0)
        assertEquals(magnetometerInitialSz, calibrator.magnetometerInitialSz, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun setMagnetometerInitialScalingFactors_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            accelerometerAndGyroscopeCalibrator,
            "running",
            true
        )
        setPrivateProperty(
            SingleSensorStaticIntervalCalibrator::class,
            magnetometerCalibrator,
            "running",
            true
        )
        assertTrue(calibrator.running)

        // set new values
        val randomizer = UniformRandomizer()
        val magnetometerInitialSx = randomizer.nextDouble()
        val magnetometerInitialSy = randomizer.nextDouble()
        val magnetometerInitialSz = randomizer.nextDouble()
        calibrator.setMagnetometerInitialScalingFactors(
            magnetometerInitialSx,
            magnetometerInitialSy,
            magnetometerInitialSz
        )
    }

    @Test
    fun setMagnetometerInitialCrossCouplingErrors_whenNotRunning_setsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default values
        assertEquals(0.0, calibrator.magnetometerInitialMxy, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMxz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMyx, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMyz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMzx, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMzy, 0.0)
        assertFalse(calibrator.running)

        // set new values
        val randomizer = UniformRandomizer()
        val magnetometerInitialMxy = randomizer.nextDouble()
        val magnetometerInitialMxz = randomizer.nextDouble()
        val magnetometerInitialMyx = randomizer.nextDouble()
        val magnetometerInitialMyz = randomizer.nextDouble()
        val magnetometerInitialMzx = randomizer.nextDouble()
        val magnetometerInitialMzy = randomizer.nextDouble()
        calibrator.setMagnetometerInitialCrossCouplingErrors(
            magnetometerInitialMxy,
            magnetometerInitialMxz,
            magnetometerInitialMyx,
            magnetometerInitialMyz,
            magnetometerInitialMzx,
            magnetometerInitialMzy
        )

        // check
        assertEquals(magnetometerInitialMxy, calibrator.magnetometerInitialMxy, 0.0)
        assertEquals(magnetometerInitialMxz, calibrator.magnetometerInitialMxz, 0.0)
        assertEquals(magnetometerInitialMyx, calibrator.magnetometerInitialMyx, 0.0)
        assertEquals(magnetometerInitialMyz, calibrator.magnetometerInitialMyz, 0.0)
        assertEquals(magnetometerInitialMzx, calibrator.magnetometerInitialMzx, 0.0)
        assertEquals(magnetometerInitialMzy, calibrator.magnetometerInitialMzy, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun setMagnetometerInitialCrossCouplingErrors_whenRunning_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            accelerometerAndGyroscopeCalibrator,
            "running",
            true
        )
        setPrivateProperty(
            SingleSensorStaticIntervalCalibrator::class,
            magnetometerCalibrator,
            "running",
            true
        )
        assertTrue(calibrator.running)

        // set new values
        val randomizer = UniformRandomizer()
        val magnetometerInitialMxy = randomizer.nextDouble()
        val magnetometerInitialMxz = randomizer.nextDouble()
        val magnetometerInitialMyx = randomizer.nextDouble()
        val magnetometerInitialMyz = randomizer.nextDouble()
        val magnetometerInitialMzx = randomizer.nextDouble()
        val magnetometerInitialMzy = randomizer.nextDouble()
        calibrator.setMagnetometerInitialCrossCouplingErrors(
            magnetometerInitialMxy,
            magnetometerInitialMxz,
            magnetometerInitialMyx,
            magnetometerInitialMyz,
            magnetometerInitialMzx,
            magnetometerInitialMzy
        )
    }

    @Test
    fun setMagnetometerInitialScalingFactorsAndCrossCouplingErrors_whenNotRunning_setsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default values
        assertEquals(0.0, calibrator.magnetometerInitialSx, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialSy, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialSz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMxy, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMxz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMyx, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMyz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMzx, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMzy, 0.0)
        assertFalse(calibrator.running)

        // set new values
        val randomizer = UniformRandomizer()
        val magnetometerInitialSx = randomizer.nextDouble()
        val magnetometerInitialSy = randomizer.nextDouble()
        val magnetometerInitialSz = randomizer.nextDouble()
        val magnetometerInitialMxy = randomizer.nextDouble()
        val magnetometerInitialMxz = randomizer.nextDouble()
        val magnetometerInitialMyx = randomizer.nextDouble()
        val magnetometerInitialMyz = randomizer.nextDouble()
        val magnetometerInitialMzx = randomizer.nextDouble()
        val magnetometerInitialMzy = randomizer.nextDouble()
        calibrator.setMagnetometerInitialScalingFactorsAndCrossCouplingErrors(
            magnetometerInitialSx,
            magnetometerInitialSy,
            magnetometerInitialSz,
            magnetometerInitialMxy,
            magnetometerInitialMxz,
            magnetometerInitialMyx,
            magnetometerInitialMyz,
            magnetometerInitialMzx,
            magnetometerInitialMzy
        )

        // check
        assertEquals(magnetometerInitialSx, calibrator.magnetometerInitialSx, 0.0)
        assertEquals(magnetometerInitialSy, calibrator.magnetometerInitialSy, 0.0)
        assertEquals(magnetometerInitialSz, calibrator.magnetometerInitialSz, 0.0)
        assertEquals(magnetometerInitialMxy, calibrator.magnetometerInitialMxy, 0.0)
        assertEquals(magnetometerInitialMxz, calibrator.magnetometerInitialMxz, 0.0)
        assertEquals(magnetometerInitialMyx, calibrator.magnetometerInitialMyx, 0.0)
        assertEquals(magnetometerInitialMyz, calibrator.magnetometerInitialMyz, 0.0)
        assertEquals(magnetometerInitialMzx, calibrator.magnetometerInitialMzx, 0.0)
        assertEquals(magnetometerInitialMzy, calibrator.magnetometerInitialMzy, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun setMagnetometerInitialScalingFactorsAndCrossCouplingErrors_whenRunning_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            accelerometerAndGyroscopeCalibrator,
            "running",
            true
        )
        setPrivateProperty(
            SingleSensorStaticIntervalCalibrator::class,
            magnetometerCalibrator,
            "running",
            true
        )
        assertTrue(calibrator.running)

        // set new values
        val randomizer = UniformRandomizer()
        val magnetometerInitialSx = randomizer.nextDouble()
        val magnetometerInitialSy = randomizer.nextDouble()
        val magnetometerInitialSz = randomizer.nextDouble()
        val magnetometerInitialMxy = randomizer.nextDouble()
        val magnetometerInitialMxz = randomizer.nextDouble()
        val magnetometerInitialMyx = randomizer.nextDouble()
        val magnetometerInitialMyz = randomizer.nextDouble()
        val magnetometerInitialMzx = randomizer.nextDouble()
        val magnetometerInitialMzy = randomizer.nextDouble()
        calibrator.setMagnetometerInitialScalingFactorsAndCrossCouplingErrors(
            magnetometerInitialSx,
            magnetometerInitialSy,
            magnetometerInitialSz,
            magnetometerInitialMxy,
            magnetometerInitialMxz,
            magnetometerInitialMyx,
            magnetometerInitialMyz,
            magnetometerInitialMzx,
            magnetometerInitialMzy
        )
    }

    @Test
    fun isAccelerometerCommonAxisUsed_whenNotRunning_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        assertFalse(calibrator.isAccelerometerCommonAxisUsed)

        // set new value
        calibrator.isAccelerometerCommonAxisUsed = true

        // check
        assertTrue(calibrator.isAccelerometerCommonAxisUsed)
    }

    @Test(expected = IllegalStateException::class)
    fun isAccelerometerCommonAxisUsed_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            accelerometerAndGyroscopeCalibrator,
            "running",
            true
        )
        setPrivateProperty(
            SingleSensorStaticIntervalCalibrator::class,
            magnetometerCalibrator,
            "running",
            true
        )
        assertTrue(calibrator.running)

        calibrator.isAccelerometerCommonAxisUsed = true
    }

    @Test
    fun isGyroscopeCommonAxisUsed_whenNotRunning_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default values
        assertFalse(calibrator.isGyroscopeCommonAxisUsed)
        assertFalse(calibrator.running)

        // set new value
        calibrator.isGyroscopeCommonAxisUsed = true

        // check
        assertTrue(calibrator.isGyroscopeCommonAxisUsed)
    }

    @Test(expected = IllegalStateException::class)
    fun isGyroscopeCommonAxisUsed_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            accelerometerAndGyroscopeCalibrator,
            "running",
            true
        )
        setPrivateProperty(
            SingleSensorStaticIntervalCalibrator::class,
            magnetometerCalibrator,
            "running",
            true
        )
        assertTrue(calibrator.running)

        // set new value
        calibrator.isGyroscopeCommonAxisUsed = true
    }

    @Test
    fun isMagnetometerCommonAxisUsed_whenNotRunning_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default values
        assertFalse(calibrator.isMagnetometerCommonAxisUsed)
        assertFalse(calibrator.running)

        // set new value
        calibrator.isMagnetometerCommonAxisUsed = true

        // check
        assertTrue(calibrator.isMagnetometerCommonAxisUsed)
    }

    @Test(expected = IllegalStateException::class)
    fun isMagnetometerCommonAxisUsed_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            accelerometerAndGyroscopeCalibrator,
            "running",
            true
        )
        setPrivateProperty(
            SingleSensorStaticIntervalCalibrator::class,
            magnetometerCalibrator,
            "running",
            true
        )
        assertTrue(calibrator.running)

        // set new value
        calibrator.isMagnetometerCommonAxisUsed = true
    }

    @Test
    fun isGDependentCrossBiasesEstimated_whenNotRunning_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default values
        assertFalse(calibrator.isGDependentCrossBiasesEstimated)
        assertFalse(calibrator.running)

        // set new value
        calibrator.isGDependentCrossBiasesEstimated = true

        // check
        assertTrue(calibrator.isGDependentCrossBiasesEstimated)
    }

    @Test(expected = IllegalStateException::class)
    fun isGDependentCrossBiasesEstimated_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            accelerometerAndGyroscopeCalibrator,
            "running",
            true
        )
        setPrivateProperty(
            SingleSensorStaticIntervalCalibrator::class,
            magnetometerCalibrator,
            "running",
            true
        )
        assertTrue(calibrator.running)

        // set new value
        calibrator.isGDependentCrossBiasesEstimated = true
    }

    @Test
    fun minimumRequiredAccelerometerMeasurements_whenCommonAxisAndKnownBias_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        calibrator.isAccelerometerCommonAxisUsed = true
        calibrator.isAccelerometerGroundTruthInitialBias = true

        // check
        assertTrue(calibrator.isAccelerometerCommonAxisUsed)
        assertTrue(calibrator.isAccelerometerGroundTruthInitialBias)
        assertEquals(7, calibrator.minimumRequiredAccelerometerMeasurements)
    }

    @Test
    fun minimumRequiredAccelerometerMeasurements_whenCommonAxisAndUnknownBias_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        calibrator.isAccelerometerCommonAxisUsed = true
        calibrator.isAccelerometerGroundTruthInitialBias = false

        // check
        assertTrue(calibrator.isAccelerometerCommonAxisUsed)
        assertFalse(calibrator.isAccelerometerGroundTruthInitialBias)
        assertEquals(10, calibrator.minimumRequiredAccelerometerMeasurements)
    }

    @Test
    fun minimumRequiredAccelerometerMeasurements_whenNotCommonAxisAndKnownBias_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        calibrator.isAccelerometerCommonAxisUsed = false
        calibrator.isAccelerometerGroundTruthInitialBias = true

        // check
        assertFalse(calibrator.isAccelerometerCommonAxisUsed)
        assertTrue(calibrator.isAccelerometerGroundTruthInitialBias)
        assertEquals(10, calibrator.minimumRequiredAccelerometerMeasurements)
    }

    @Test
    fun minimumRequiredAccelerometerMeasurements_whenNotCommonAxisAndUnknownBias_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        calibrator.isAccelerometerCommonAxisUsed = false
        calibrator.isAccelerometerGroundTruthInitialBias = false

        // check
        assertFalse(calibrator.isAccelerometerCommonAxisUsed)
        assertFalse(calibrator.isAccelerometerGroundTruthInitialBias)
        assertEquals(13, calibrator.minimumRequiredAccelerometerMeasurements)
    }

    @Test
    fun minimumRequiredGyroscopeMeasurements_whenGroundTruthInitialBiasCommonAxisAndCrossBiases_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        calibrator.isGyroscopeGroundTruthInitialBias = true
        calibrator.isGyroscopeCommonAxisUsed = true
        calibrator.isGDependentCrossBiasesEstimated = true

        // check
        assertTrue(calibrator.isGyroscopeGroundTruthInitialBias)
        assertTrue(calibrator.isGyroscopeCommonAxisUsed)
        assertTrue(calibrator.isGDependentCrossBiasesEstimated)
        assertEquals(16, calibrator.minimumRequiredGyroscopeMeasurements)
    }

    @Test
    fun minimumRequiredGyroscopeMeasurements_whenGroundTruthInitialBiasCommonAxisAndNoCrossBiases_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        calibrator.isGyroscopeGroundTruthInitialBias = true
        calibrator.isGyroscopeCommonAxisUsed = true
        calibrator.isGDependentCrossBiasesEstimated = false

        // check
        assertTrue(calibrator.isGyroscopeGroundTruthInitialBias)
        assertTrue(calibrator.isGyroscopeCommonAxisUsed)
        assertFalse(calibrator.isGDependentCrossBiasesEstimated)
        assertEquals(7, calibrator.minimumRequiredGyroscopeMeasurements)
    }

    @Test
    fun minimumRequiredGyroscopeMeasurements_whenGroundTruthInitialBiasNoCommonAxisAndCrossBiases_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        calibrator.isGyroscopeGroundTruthInitialBias = true
        calibrator.isGyroscopeCommonAxisUsed = false
        calibrator.isGDependentCrossBiasesEstimated = true

        // check
        assertTrue(calibrator.isGyroscopeGroundTruthInitialBias)
        assertFalse(calibrator.isGyroscopeCommonAxisUsed)
        assertTrue(calibrator.isGDependentCrossBiasesEstimated)
        assertEquals(19, calibrator.minimumRequiredGyroscopeMeasurements)
    }

    @Test
    fun minimumRequiredGyroscopeMeasurements_whenGroundTruthInitialBiasNoCommonAxisAndNoCrossBiases_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        calibrator.isGyroscopeGroundTruthInitialBias = true
        calibrator.isGyroscopeCommonAxisUsed = false
        calibrator.isGDependentCrossBiasesEstimated = false

        // check
        assertTrue(calibrator.isGyroscopeGroundTruthInitialBias)
        assertFalse(calibrator.isGyroscopeCommonAxisUsed)
        assertFalse(calibrator.isGDependentCrossBiasesEstimated)
        assertEquals(10, calibrator.minimumRequiredGyroscopeMeasurements)
    }

    @Test
    fun minimumRequiredGyroscopeMeasurements_whenNoGroundTruthInitialBiasCommonAxisAndCrossBiases_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        calibrator.isGyroscopeGroundTruthInitialBias = false
        calibrator.isGyroscopeCommonAxisUsed = true
        calibrator.isGDependentCrossBiasesEstimated = true

        // check
        assertFalse(calibrator.isGyroscopeGroundTruthInitialBias)
        assertTrue(calibrator.isGyroscopeCommonAxisUsed)
        assertTrue(calibrator.isGDependentCrossBiasesEstimated)
        assertEquals(19, calibrator.minimumRequiredGyroscopeMeasurements)
    }

    @Test
    fun minimumRequiredGyroscopeMeasurements_whenNoGroundTruthInitialBiasCommonAxisAndNoCrossBiases_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        calibrator.isGyroscopeGroundTruthInitialBias = false
        calibrator.isGyroscopeCommonAxisUsed = true
        calibrator.isGDependentCrossBiasesEstimated = false

        // check
        assertFalse(calibrator.isGyroscopeGroundTruthInitialBias)
        assertTrue(calibrator.isGyroscopeCommonAxisUsed)
        assertFalse(calibrator.isGDependentCrossBiasesEstimated)
        assertEquals(10, calibrator.minimumRequiredGyroscopeMeasurements)
    }

    @Test
    fun minimumRequiredGyroscopeMeasurements_whenNoGroundTruthInitialBiasNoCommonAxisAndCrossBiases_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        calibrator.isGyroscopeGroundTruthInitialBias = false
        calibrator.isGyroscopeCommonAxisUsed = false
        calibrator.isGDependentCrossBiasesEstimated = true

        // check
        assertFalse(calibrator.isGyroscopeGroundTruthInitialBias)
        assertFalse(calibrator.isGyroscopeCommonAxisUsed)
        assertTrue(calibrator.isGDependentCrossBiasesEstimated)
        assertEquals(22, calibrator.minimumRequiredGyroscopeMeasurements)
    }

    @Test
    fun minimumRequiredGyroscopeMeasurements_whenNoGroundTruthInitialBiasNoCommonAxisAndNoCrossBiases_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        calibrator.isGyroscopeGroundTruthInitialBias = false
        calibrator.isGyroscopeCommonAxisUsed = false
        calibrator.isGDependentCrossBiasesEstimated = false

        // check
        assertFalse(calibrator.isGyroscopeGroundTruthInitialBias)
        assertFalse(calibrator.isGyroscopeCommonAxisUsed)
        assertFalse(calibrator.isGDependentCrossBiasesEstimated)
        assertEquals(13, calibrator.minimumRequiredGyroscopeMeasurements)
    }

    @Test
    fun minimumRequiredMagnetometerMeasurements_whenCommonAxisAndKnownHardIron_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        calibrator.isMagnetometerCommonAxisUsed = true
        calibrator.isMagnetometerGroundTruthInitialHardIron = true

        // check
        assertTrue(calibrator.isMagnetometerCommonAxisUsed)
        assertTrue(calibrator.isMagnetometerGroundTruthInitialHardIron)
        assertEquals(7, calibrator.minimumRequiredMagnetometerMeasurements)
    }

    @Test
    fun minimumRequiredMagnetometerMeasurements_whenCommonAxisAndUnknownHardIron_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        calibrator.isMagnetometerCommonAxisUsed = true
        calibrator.isMagnetometerGroundTruthInitialHardIron = false

        // check
        assertTrue(calibrator.isMagnetometerCommonAxisUsed)
        assertFalse(calibrator.isMagnetometerGroundTruthInitialHardIron)
        assertEquals(10, calibrator.minimumRequiredMagnetometerMeasurements)
    }

    @Test
    fun minimumRequiredMagnetometerMeasurements_whenNotCommonAxisAndKnownHardIron_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        calibrator.isMagnetometerCommonAxisUsed = false
        calibrator.isMagnetometerGroundTruthInitialHardIron = true

        // check
        assertFalse(calibrator.isMagnetometerCommonAxisUsed)
        assertTrue(calibrator.isMagnetometerGroundTruthInitialHardIron)
        assertEquals(10, calibrator.minimumRequiredMagnetometerMeasurements)
    }

    @Test
    fun minimumRequiredMagnetometerMeasurements_whenNotCommonAxisAndUnknownHardIron_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        calibrator.isMagnetometerCommonAxisUsed = false
        calibrator.isMagnetometerGroundTruthInitialHardIron = false

        // check
        assertFalse(calibrator.isMagnetometerCommonAxisUsed)
        assertFalse(calibrator.isMagnetometerGroundTruthInitialHardIron)
        assertEquals(13, calibrator.minimumRequiredMagnetometerMeasurements)
    }

    @Test
    fun averageGravityNorm_returnsAccelerometerAndGyroscopeAverageGravityNorm() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val accelerometerAndGyroscopeCalibratorSpy = spy(accelerometerAndGyroscopeCalibrator)
//        val accelerometerAndGyroscopeCalibratorSpy = spyk(accelerometerAndGyroscopeCalibrator)
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibratorSpy
        )
        val randomizer = UniformRandomizer()
        val averageGravityNorm = randomizer.nextDouble()
        whenever(accelerometerAndGyroscopeCalibratorSpy.averageGravityNorm).thenReturn(averageGravityNorm)
/*        every { accelerometerAndGyroscopeCalibratorSpy.averageGravityNorm }
            .returns(averageGravityNorm)*/

        assertEquals(averageGravityNorm, calibrator.averageGravityNorm)
    }

    @Test
    fun averageGravityNormAsMeasurement_returnsAccelerometerAndGyroscopeAverageGravityNormAsMeasurement() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val accelerometerAndGyroscopeCalibratorSpy = spy(accelerometerAndGyroscopeCalibrator)
//        val accelerometerAndGyroscopeCalibratorSpy = spyk(accelerometerAndGyroscopeCalibrator)
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibratorSpy
        )
        val randomizer = UniformRandomizer()
        val averageGravityNorm = randomizer.nextDouble()
        val averageGravityNormAsMeasurement = Acceleration(
            averageGravityNorm,
            AccelerationUnit.METERS_PER_SQUARED_SECOND
        )
        whenever(accelerometerAndGyroscopeCalibratorSpy.averageGravityNormAsMeasurement).thenReturn(averageGravityNormAsMeasurement)
/*        every { accelerometerAndGyroscopeCalibratorSpy.averageGravityNormAsMeasurement }
            .returns(averageGravityNormAsMeasurement)*/

        assertSame(averageGravityNormAsMeasurement, calibrator.averageGravityNormAsMeasurement)
    }

    @Test
    fun getAverageGravityNormAsMeasurement_returnsAccelerometerAndGyroscopeAverageGravityNormAsMeasurement() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val accelerometerAndGyroscopeCalibratorSpy = spy(accelerometerAndGyroscopeCalibrator)
//        val accelerometerAndGyroscopeCalibratorSpy = spyk(accelerometerAndGyroscopeCalibrator)
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibratorSpy
        )
        val randomizer = UniformRandomizer()
        val averageGravityNorm = randomizer.nextDouble()
        doAnswer { invocation ->
            val result = invocation.getArgument<Acceleration>(0)
            result.value = averageGravityNorm
            result.unit = AccelerationUnit.METERS_PER_SQUARED_SECOND
            return@doAnswer true
        }.whenever(accelerometerAndGyroscopeCalibratorSpy).getAverageGravityNormAsMeasurement(any())
/*        every { accelerometerAndGyroscopeCalibratorSpy.getAverageGravityNormAsMeasurement(any()) }.answers { answer ->
            val result = answer.invocation.args[0] as Acceleration
            result.value = averageGravityNorm
            result.unit = AccelerationUnit.METERS_PER_SQUARED_SECOND
            return@answers true
        }*/

        val averageGravityNormAsMeasurement = Acceleration(
            0.0,
            AccelerationUnit.METERS_PER_SQUARED_SECOND
        )
        assertTrue(calibrator.getAverageGravityNormAsMeasurement(averageGravityNormAsMeasurement))
        assertEquals(averageGravityNorm, averageGravityNormAsMeasurement.value)
        assertEquals(
            AccelerationUnit.METERS_PER_SQUARED_SECOND,
            averageGravityNormAsMeasurement.unit
        )
    }

    @Test
    fun gravityNormVariance_returnsAccelerometerAndGyroscopeGravityNormVariance() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val accelerometerAndGyroscopeCalibratorSpy = spy(accelerometerAndGyroscopeCalibrator)
//        val accelerometerAndGyroscopeCalibratorSpy = spyk(accelerometerAndGyroscopeCalibrator)
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibratorSpy
        )
        val randomizer = UniformRandomizer()
        val gravityNormVariance = randomizer.nextDouble()
        whenever(accelerometerAndGyroscopeCalibratorSpy.gravityNormVariance).thenReturn(gravityNormVariance)
/*        every { accelerometerAndGyroscopeCalibratorSpy.gravityNormVariance }
            .returns(gravityNormVariance)*/

        assertEquals(gravityNormVariance, calibrator.gravityNormVariance)
    }

    @Test
    fun gravityNormStandardDeviation_returnsAccelerometerAndGyroscopeGravityNormStandardDeviation() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val accelerometerAndGyroscopeCalibratorSpy = spy(accelerometerAndGyroscopeCalibrator)
//        val accelerometerAndGyroscopeCalibratorSpy = spyk(accelerometerAndGyroscopeCalibrator)
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibratorSpy
        )
        val randomizer = UniformRandomizer()
        val gravityNormStandardDeviation = randomizer.nextDouble()
        whenever(accelerometerAndGyroscopeCalibratorSpy.gravityNormStandardDeviation).thenReturn(gravityNormStandardDeviation)
/*        every { accelerometerAndGyroscopeCalibratorSpy.gravityNormStandardDeviation }
            .returns(gravityNormStandardDeviation)*/

        assertEquals(gravityNormStandardDeviation, calibrator.gravityNormStandardDeviation)
    }

    @Test
    fun gravityNormStandardDeviationAsMeasurement_returnsAccelerometerAndGyroscopeGravityNormStandardDeviationAsMeasurement() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val accelerometerAndGyroscopeCalibratorSpy = spy(accelerometerAndGyroscopeCalibrator)
//        val accelerometerAndGyroscopeCalibratorSpy = spyk(accelerometerAndGyroscopeCalibrator)
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibratorSpy
        )
        val randomizer = UniformRandomizer()
        val gravityNormStandardDeviation = randomizer.nextDouble()
        val gravityNormStandardDeviationMeasurement = Acceleration(
            gravityNormStandardDeviation,
            AccelerationUnit.METERS_PER_SQUARED_SECOND
        )
        whenever(accelerometerAndGyroscopeCalibratorSpy.gravityNormStandardDeviationAsMeasurement).thenReturn(gravityNormStandardDeviationMeasurement)
/*        every { accelerometerAndGyroscopeCalibratorSpy.gravityNormStandardDeviationAsMeasurement }
            .returns(gravityNormStandardDeviationMeasurement)*/

        assertSame(
            gravityNormStandardDeviationMeasurement,
            calibrator.gravityNormStandardDeviationAsMeasurement
        )
    }

    @Test
    fun getGravityNormStandardDeviationAsMeasurement_returnsAccelerometerAndGyroscopeGravityNormStandardDeviationAsMeasurement() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val accelerometerAndGyroscopeCalibratorSpy = spy(accelerometerAndGyroscopeCalibrator)
//        val accelerometerAndGyroscopeCalibratorSpy = spyk(accelerometerAndGyroscopeCalibrator)
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibratorSpy
        )
        val randomizer = UniformRandomizer()
        val gravityNormStandardDeviation = randomizer.nextDouble()
        doAnswer { invocation ->
            val acceleration = invocation.getArgument<Acceleration>(0)
            acceleration.value = gravityNormStandardDeviation
            acceleration.unit = AccelerationUnit.METERS_PER_SQUARED_SECOND
            return@doAnswer true
        }.whenever(accelerometerAndGyroscopeCalibratorSpy).getGravityNormStandardDeviationAsMeasurement(
            any()
        )
/*        every {
            accelerometerAndGyroscopeCalibratorSpy.getGravityNormStandardDeviationAsMeasurement(
                any()
            )
        }.answers { answer ->
            val acceleration = answer.invocation.args[0] as Acceleration
            acceleration.value = gravityNormStandardDeviation
            acceleration.unit = AccelerationUnit.METERS_PER_SQUARED_SECOND
            return@answers true
        }*/

        val gravityNormStandardDeviationMeasurement = Acceleration(
            0.0,
            AccelerationUnit.METERS_PER_SQUARED_SECOND
        )
        assertTrue(
            calibrator.getGravityNormStandardDeviationAsMeasurement(
                gravityNormStandardDeviationMeasurement
            )
        )
        assertEquals(gravityNormStandardDeviation, gravityNormStandardDeviationMeasurement.value)
        assertEquals(
            AccelerationUnit.METERS_PER_SQUARED_SECOND,
            gravityNormStandardDeviationMeasurement.unit
        )
    }

    @Test
    fun gravityPsd_returnsAccelerometerAndGyroscopeGravityGravityPsd() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val accelerometerAndGyroscopeCalibratorSpy = spy(accelerometerAndGyroscopeCalibrator)
//        val accelerometerAndGyroscopeCalibratorSpy = spyk(accelerometerAndGyroscopeCalibrator)
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibratorSpy
        )
        val randomizer = UniformRandomizer()
        val gravityPsd = randomizer.nextDouble()
        whenever(accelerometerAndGyroscopeCalibratorSpy.gravityPsd).thenReturn(gravityPsd)
/*        every { accelerometerAndGyroscopeCalibratorSpy.gravityPsd }
            .returns(gravityPsd)*/

        assertEquals(
            gravityPsd,
            calibrator.gravityPsd
        )
    }

    @Test
    fun gravityRootPsd_returnsAccelerometerAndGyroscopeGravityGravityRootPsd() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val accelerometerAndGyroscopeCalibratorSpy = spy(accelerometerAndGyroscopeCalibrator)
//        val accelerometerAndGyroscopeCalibratorSpy = spyk(accelerometerAndGyroscopeCalibrator)
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibratorSpy
        )
        val randomizer = UniformRandomizer()
        val gravityRootPsd = randomizer.nextDouble()
        whenever(accelerometerAndGyroscopeCalibratorSpy.gravityRootPsd).thenReturn(gravityRootPsd)
/*        every { accelerometerAndGyroscopeCalibratorSpy.gravityRootPsd }
            .returns(gravityRootPsd)*/

        assertEquals(
            gravityRootPsd,
            calibrator.gravityRootPsd
        )
    }

    @Test
    fun accelerometerRobustMethod_whenNotRunning_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.accelerometerRobustMethod)

        // set new value
        calibrator.accelerometerRobustMethod = RobustEstimatorMethod.RANSAC

        // check
        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.accelerometerRobustMethod)
    }

    @Test(expected = IllegalStateException::class)
    fun accelerometerRobustMethod_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            accelerometerAndGyroscopeCalibrator,
            "running",
            true
        )
        setPrivateProperty(
            SingleSensorStaticIntervalCalibrator::class,
            magnetometerCalibrator,
            "running",
            true
        )
        assertTrue(calibrator.running)

        calibrator.accelerometerRobustMethod = RobustEstimatorMethod.RANSAC
    }

    @Test
    fun accelerometerRobustConfidence_whenValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertEquals(
            StaticIntervalAccelerometerCalibrator.ROBUST_DEFAULT_CONFIDENCE,
            calibrator.accelerometerRobustConfidence,
            0.0
        )

        // set new value
        calibrator.accelerometerRobustConfidence = ROBUST_CONFIDENCE

        // check
        assertEquals(ROBUST_CONFIDENCE, calibrator.accelerometerRobustConfidence, 0.0)
    }

    @Test(expected = IllegalArgumentException::class)
    fun accelerometerRobustConfidence_whenInvalidLowerBound_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        calibrator.accelerometerRobustConfidence = -1.0
    }

    @Test(expected = IllegalArgumentException::class)
    fun accelerometerRobustConfidence_whenInvalidUpperBound_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        calibrator.accelerometerRobustConfidence = 2.0
    }

    @Test(expected = IllegalStateException::class)
    fun accelerometerRobustConfidence_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            accelerometerAndGyroscopeCalibrator,
            "running",
            true
        )
        setPrivateProperty(
            SingleSensorStaticIntervalCalibrator::class,
            magnetometerCalibrator,
            "running",
            true
        )
        assertTrue(calibrator.running)

        calibrator.accelerometerRobustConfidence = ROBUST_CONFIDENCE
    }

    @Test
    fun accelerometerRobustMaxIterations_whenValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertEquals(
            StaticIntervalAccelerometerCalibrator.ROBUST_DEFAULT_MAX_ITERATIONS,
            calibrator.accelerometerRobustMaxIterations
        )

        // set new value
        calibrator.accelerometerRobustMaxIterations = ROBUST_MAX_ITERATIONS

        // check
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.accelerometerRobustMaxIterations)
    }

    @Test(expected = IllegalArgumentException::class)
    fun accelerometerRobustMaxIterations_whenInvalid_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        calibrator.accelerometerRobustMaxIterations = 0
    }

    @Test(expected = IllegalStateException::class)
    fun accelerometerRobustMaxIterations_whenRunning_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            accelerometerAndGyroscopeCalibrator,
            "running",
            true
        )
        setPrivateProperty(
            SingleSensorStaticIntervalCalibrator::class,
            magnetometerCalibrator,
            "running",
            true
        )
        assertTrue(calibrator.running)

        calibrator.accelerometerRobustMaxIterations = ROBUST_MAX_ITERATIONS
    }

    @Test
    fun accelerometerRobustPreliminarySubsetSize_whenValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertEquals(
            StaticIntervalAccelerometerCalibrator.ACCELEROMETER_UNKNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.accelerometerRobustPreliminarySubsetSize
        )

        // set new value
        calibrator.accelerometerRobustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE

        // check
        assertEquals(
            ROBUST_PRELIMINARY_SUBSET_SIZE,
            calibrator.accelerometerRobustPreliminarySubsetSize
        )
    }

    @Test(expected = IllegalArgumentException::class)
    fun accelerometerRobustPreliminarySubsetSize_whenInvalid_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        calibrator.accelerometerRobustPreliminarySubsetSize = 12
    }

    @Test(expected = IllegalStateException::class)
    fun accelerometerRobustPreliminarySubsetSize_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            accelerometerAndGyroscopeCalibrator,
            "running",
            true
        )
        setPrivateProperty(
            SingleSensorStaticIntervalCalibrator::class,
            magnetometerCalibrator,
            "running",
            true
        )
        assertTrue(calibrator.running)

        calibrator.accelerometerRobustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
    }

    @Test
    fun accelerometerRobustThreshold_whenValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.accelerometerRobustThreshold)

        // set new value
        calibrator.accelerometerRobustThreshold = ROBUST_THRESHOLD

        // check
        val robustThreshold = calibrator.accelerometerRobustThreshold
        requireNotNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD, robustThreshold, 0.0)

        // set new value
        calibrator.accelerometerRobustThreshold = null

        // check
        assertNull(calibrator.accelerometerRobustThreshold)
    }

    @Test(expected = IllegalArgumentException::class)
    fun accelerometerRobustThreshold_whenInvalid_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        calibrator.accelerometerRobustThreshold = 0.0
    }

    @Test(expected = IllegalStateException::class)
    fun accelerometerRobustThreshold_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            accelerometerAndGyroscopeCalibrator,
            "running",
            true
        )
        setPrivateProperty(
            SingleSensorStaticIntervalCalibrator::class,
            magnetometerCalibrator,
            "running",
            true
        )
        assertTrue(calibrator.running)

        calibrator.accelerometerRobustThreshold = ROBUST_THRESHOLD
    }

    @Test
    fun accelerometerRobustThresholdFactor_whenValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertEquals(
            StaticIntervalAccelerometerCalibrator.DEFAULT_ROBUST_THRESHOLD_FACTOR,
            calibrator.accelerometerRobustThresholdFactor,
            0.0
        )

        // set new value
        calibrator.accelerometerRobustThresholdFactor = ROBUST_THRESHOLD_FACTOR

        // check
        assertEquals(ROBUST_THRESHOLD_FACTOR, calibrator.accelerometerRobustThresholdFactor, 0.0)
    }

    @Test(expected = IllegalArgumentException::class)
    fun accelerometerRobustThresholdFactor_whenInvalid_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        calibrator.accelerometerRobustThresholdFactor = 0.0
    }

    @Test(expected = IllegalStateException::class)
    fun accelerometerRobustThresholdFactor_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            accelerometerAndGyroscopeCalibrator,
            "running",
            true
        )
        setPrivateProperty(
            SingleSensorStaticIntervalCalibrator::class,
            magnetometerCalibrator,
            "running",
            true
        )
        assertTrue(calibrator.running)

        calibrator.accelerometerRobustThresholdFactor = ROBUST_THRESHOLD_FACTOR
    }

    @Test
    fun accelerometerRobustStopThresholdFactor_whenValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context)

        assertEquals(
            StaticIntervalAccelerometerCalibrator.DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR,
            calibrator.accelerometerRobustStopThresholdFactor,
            0.0
        )

        // set new value
        calibrator.accelerometerRobustStopThresholdFactor = ROBUST_STOP_THRESHOLD_FACTOR

        // check
        assertEquals(
            ROBUST_STOP_THRESHOLD_FACTOR,
            calibrator.accelerometerRobustStopThresholdFactor,
            0.0
        )
    }

    @Test(expected = IllegalArgumentException::class)
    fun accelerometerRobustStopThresholdFactor_whenInvalid_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        calibrator.accelerometerRobustStopThresholdFactor = 0.0
    }

    @Test(expected = IllegalStateException::class)
    fun accelerometerRobustStopThresholdFactor_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            accelerometerAndGyroscopeCalibrator,
            "running",
            true
        )
        setPrivateProperty(
            SingleSensorStaticIntervalCalibrator::class,
            magnetometerCalibrator,
            "running",
            true
        )
        assertTrue(calibrator.running)

        calibrator.accelerometerRobustStopThresholdFactor = ROBUST_STOP_THRESHOLD_FACTOR
    }

    @Test
    fun gyroscopeRobustMethod_whenNotRunning_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.gyroscopeRobustMethod)

        // set new value
        calibrator.gyroscopeRobustMethod = RobustEstimatorMethod.RANSAC

        // check
        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.gyroscopeRobustMethod)
    }

    @Test(expected = IllegalStateException::class)
    fun gyroscopeRobustMethod_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            accelerometerAndGyroscopeCalibrator,
            "running",
            true
        )
        setPrivateProperty(
            SingleSensorStaticIntervalCalibrator::class,
            magnetometerCalibrator,
            "running",
            true
        )
        assertTrue(calibrator.running)

        calibrator.gyroscopeRobustMethod = RobustEstimatorMethod.RANSAC
    }

    @Test
    fun gyroscopeRobustConfidence_whenValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertEquals(
            StaticIntervalGyroscopeCalibrator.ROBUST_DEFAULT_CONFIDENCE,
            calibrator.gyroscopeRobustConfidence,
            0.0
        )

        // set new value
        calibrator.gyroscopeRobustConfidence = ROBUST_CONFIDENCE

        // check
        assertEquals(ROBUST_CONFIDENCE, calibrator.gyroscopeRobustConfidence, 0.0)
    }

    @Test(expected = IllegalArgumentException::class)
    fun gyroscopeRobustConfidence_whenInvalidLowerBound_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        calibrator.gyroscopeRobustConfidence = -1.0
    }

    @Test(expected = IllegalArgumentException::class)
    fun gyroscopeRobustConfidence_whenInvalidUpperBound_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        calibrator.gyroscopeRobustConfidence = 2.0
    }

    @Test(expected = IllegalStateException::class)
    fun gyroscopeRobustConfidence_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            accelerometerAndGyroscopeCalibrator,
            "running",
            true
        )
        setPrivateProperty(
            SingleSensorStaticIntervalCalibrator::class,
            magnetometerCalibrator,
            "running",
            true
        )
        assertTrue(calibrator.running)

        calibrator.gyroscopeRobustConfidence = ROBUST_CONFIDENCE
    }

    @Test
    fun gyroscopeRobustMaxIterations_whenValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertEquals(
            StaticIntervalGyroscopeCalibrator.ROBUST_DEFAULT_MAX_ITERATIONS,
            calibrator.gyroscopeRobustMaxIterations
        )

        // set new value
        calibrator.gyroscopeRobustMaxIterations = ROBUST_MAX_ITERATIONS

        // check
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.gyroscopeRobustMaxIterations)
    }

    @Test(expected = IllegalArgumentException::class)
    fun gyroscopeRobustMaxIterations_whenInvalid_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        calibrator.gyroscopeRobustMaxIterations = 0
    }

    @Test(expected = IllegalStateException::class)
    fun gyroscopeRobustMaxIterations_whenRunning_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            accelerometerAndGyroscopeCalibrator,
            "running",
            true
        )
        setPrivateProperty(
            SingleSensorStaticIntervalCalibrator::class,
            magnetometerCalibrator,
            "running",
            true
        )
        assertTrue(calibrator.running)

        calibrator.gyroscopeRobustMaxIterations = ROBUST_MAX_ITERATIONS
    }

    @Test
    fun gyroscopeRobustPreliminarySubsetSize_whenValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertEquals(
            StaticIntervalGyroscopeCalibrator.GYROSCOPE_UNKNOWN_BIAS_MINIMUM_SEQUENCES_GENERAL,
            calibrator.gyroscopeRobustPreliminarySubsetSize
        )

        // set new value
        calibrator.gyroscopeRobustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE

        // check
        assertEquals(
            ROBUST_PRELIMINARY_SUBSET_SIZE,
            calibrator.gyroscopeRobustPreliminarySubsetSize
        )
    }

    @Test(expected = IllegalArgumentException::class)
    fun gyroscopeRobustPreliminarySubsetSize_whenInvalid_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        calibrator.gyroscopeRobustPreliminarySubsetSize = 12
    }

    @Test(expected = IllegalStateException::class)
    fun gyroscopeRobustPreliminarySubsetSize_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            accelerometerAndGyroscopeCalibrator,
            "running",
            true
        )
        setPrivateProperty(
            SingleSensorStaticIntervalCalibrator::class,
            magnetometerCalibrator,
            "running",
            true
        )
        assertTrue(calibrator.running)

        calibrator.gyroscopeRobustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
    }

    @Test
    fun gyroscopeRobustThreshold_whenValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.gyroscopeRobustThreshold)

        // set new value
        calibrator.gyroscopeRobustThreshold = ROBUST_THRESHOLD

        // check
        val robustThreshold = calibrator.gyroscopeRobustThreshold
        requireNotNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD, robustThreshold, 0.0)

        // set new value
        calibrator.gyroscopeRobustThreshold = null

        // check
        assertNull(calibrator.gyroscopeRobustThreshold)
    }

    @Test(expected = IllegalArgumentException::class)
    fun gyroscopeRobustThreshold_whenInvalid_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        calibrator.gyroscopeRobustThreshold = 0.0
    }

    @Test(expected = IllegalStateException::class)
    fun gyroscopeRobustThreshold_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            accelerometerAndGyroscopeCalibrator,
            "running",
            true
        )
        setPrivateProperty(
            SingleSensorStaticIntervalCalibrator::class,
            magnetometerCalibrator,
            "running",
            true
        )
        assertTrue(calibrator.running)

        calibrator.gyroscopeRobustThreshold = ROBUST_THRESHOLD
    }

    @Test
    fun gyroscopeRobustThresholdFactor_whenValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertEquals(
            StaticIntervalAccelerometerCalibrator.DEFAULT_ROBUST_THRESHOLD_FACTOR,
            calibrator.gyroscopeRobustThresholdFactor,
            0.0
        )

        // set new value
        calibrator.gyroscopeRobustThresholdFactor = ROBUST_THRESHOLD_FACTOR

        // check
        assertEquals(ROBUST_THRESHOLD_FACTOR, calibrator.gyroscopeRobustThresholdFactor, 0.0)
    }

    @Test(expected = IllegalArgumentException::class)
    fun gyroscopeRobustThresholdFactor_whenInvalid_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        calibrator.gyroscopeRobustThresholdFactor = 0.0
    }

    @Test(expected = IllegalStateException::class)
    fun gyroscopeRobustThresholdFactor_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            accelerometerAndGyroscopeCalibrator,
            "running",
            true
        )
        setPrivateProperty(
            SingleSensorStaticIntervalCalibrator::class,
            magnetometerCalibrator,
            "running",
            true
        )
        assertTrue(calibrator.running)

        calibrator.gyroscopeRobustThresholdFactor = ROBUST_THRESHOLD_FACTOR
    }

    @Test
    fun gyroscopeRobustStopThresholdFactor_whenValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        assertEquals(
            StaticIntervalAccelerometerCalibrator.DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR,
            calibrator.gyroscopeRobustStopThresholdFactor,
            0.0
        )

        // set new value
        calibrator.gyroscopeRobustStopThresholdFactor = ROBUST_STOP_THRESHOLD_FACTOR

        // check
        assertEquals(
            ROBUST_STOP_THRESHOLD_FACTOR,
            calibrator.gyroscopeRobustStopThresholdFactor,
            0.0
        )
    }

    @Test(expected = IllegalArgumentException::class)
    fun gyroscopeRobustStopThresholdFactor_whenInvalid_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        calibrator.gyroscopeRobustStopThresholdFactor = 0.0
    }

    @Test(expected = IllegalStateException::class)
    fun gyroscopeRobustStopThresholdFactor_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            accelerometerAndGyroscopeCalibrator,
            "running",
            true
        )
        setPrivateProperty(
            SingleSensorStaticIntervalCalibrator::class,
            magnetometerCalibrator,
            "running",
            true
        )
        assertTrue(calibrator.running)

        calibrator.gyroscopeRobustStopThresholdFactor = ROBUST_STOP_THRESHOLD_FACTOR
    }

    @Test
    fun magnetometerRobustMethod_whenNotRunning_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.magnetometerRobustMethod)

        // set new value
        calibrator.magnetometerRobustMethod = RobustEstimatorMethod.RANSAC

        // check
        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.magnetometerRobustMethod)
    }

    @Test(expected = IllegalStateException::class)
    fun magnetometerRobustMethod_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            accelerometerAndGyroscopeCalibrator,
            "running",
            true
        )
        setPrivateProperty(
            SingleSensorStaticIntervalCalibrator::class,
            magnetometerCalibrator,
            "running",
            true
        )
        assertTrue(calibrator.running)

        calibrator.magnetometerRobustMethod = RobustEstimatorMethod.RANSAC
    }

    @Test
    fun magnetometerRobustConfidence_whenValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertEquals(
            StaticIntervalAccelerometerCalibrator.ROBUST_DEFAULT_CONFIDENCE,
            calibrator.magnetometerRobustConfidence,
            0.0
        )

        // set new value
        calibrator.magnetometerRobustConfidence = ROBUST_CONFIDENCE

        // check
        assertEquals(ROBUST_CONFIDENCE, calibrator.magnetometerRobustConfidence, 0.0)
    }

    @Test(expected = IllegalArgumentException::class)
    fun magnetometerRobustConfidence_whenInvalidLowerBound_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        calibrator.magnetometerRobustConfidence = -1.0
    }

    @Test(expected = IllegalArgumentException::class)
    fun magnetometerRobustConfidence_whenInvalidUpperBound_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        calibrator.magnetometerRobustConfidence = 2.0
    }

    @Test(expected = IllegalStateException::class)
    fun magnetometerRobustConfidence_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            accelerometerAndGyroscopeCalibrator,
            "running",
            true
        )
        setPrivateProperty(
            SingleSensorStaticIntervalCalibrator::class,
            magnetometerCalibrator,
            "running",
            true
        )
        assertTrue(calibrator.running)

        calibrator.magnetometerRobustConfidence = ROBUST_CONFIDENCE
    }

    @Test
    fun magnetometerRobustMaxIterations_whenValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertEquals(
            StaticIntervalAccelerometerCalibrator.ROBUST_DEFAULT_MAX_ITERATIONS,
            calibrator.magnetometerRobustMaxIterations
        )

        // set new value
        calibrator.magnetometerRobustMaxIterations = ROBUST_MAX_ITERATIONS

        // check
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.magnetometerRobustMaxIterations)
    }

    @Test(expected = IllegalArgumentException::class)
    fun magnetometerRobustMaxIterations_whenInvalid_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        calibrator.magnetometerRobustMaxIterations = 0
    }

    @Test(expected = IllegalStateException::class)
    fun magnetometerRobustMaxIterations_whenRunning_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            accelerometerAndGyroscopeCalibrator,
            "running",
            true
        )
        setPrivateProperty(
            SingleSensorStaticIntervalCalibrator::class,
            magnetometerCalibrator,
            "running",
            true
        )
        assertTrue(calibrator.running)

        calibrator.magnetometerRobustMaxIterations = ROBUST_MAX_ITERATIONS
    }

    @Test
    fun magnetometerRobustPreliminarySubsetSize_whenValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertEquals(
            StaticIntervalAccelerometerCalibrator.ACCELEROMETER_UNKNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.magnetometerRobustPreliminarySubsetSize
        )

        // set new value
        calibrator.magnetometerRobustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE

        // check
        assertEquals(
            ROBUST_PRELIMINARY_SUBSET_SIZE,
            calibrator.magnetometerRobustPreliminarySubsetSize
        )
    }

    @Test(expected = IllegalArgumentException::class)
    fun magnetometerRobustPreliminarySubsetSize_whenInvalid_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        calibrator.magnetometerRobustPreliminarySubsetSize = 12
    }

    @Test(expected = IllegalStateException::class)
    fun magnetometerRobustPreliminarySubsetSize_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            accelerometerAndGyroscopeCalibrator,
            "running",
            true
        )
        setPrivateProperty(
            SingleSensorStaticIntervalCalibrator::class,
            magnetometerCalibrator,
            "running",
            true
        )
        assertTrue(calibrator.running)

        calibrator.magnetometerRobustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
    }

    @Test
    fun magnetometerRobustThreshold_whenValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.magnetometerRobustThreshold)

        // set new value
        calibrator.magnetometerRobustThreshold = ROBUST_THRESHOLD

        // check
        val robustThreshold = calibrator.magnetometerRobustThreshold
        requireNotNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD, robustThreshold, 0.0)

        // set new value
        calibrator.magnetometerRobustThreshold = null

        // check
        assertNull(calibrator.magnetometerRobustThreshold)
    }

    @Test(expected = IllegalArgumentException::class)
    fun magnetometerRobustThreshold_whenInvalid_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        calibrator.magnetometerRobustThreshold = 0.0
    }

    @Test(expected = IllegalStateException::class)
    fun magnetometerRobustThreshold_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            accelerometerAndGyroscopeCalibrator,
            "running",
            true
        )
        setPrivateProperty(
            SingleSensorStaticIntervalCalibrator::class,
            magnetometerCalibrator,
            "running",
            true
        )
        assertTrue(calibrator.running)

        calibrator.magnetometerRobustThreshold = ROBUST_THRESHOLD
    }

    @Test
    fun magnetometerRobustThresholdFactor_whenValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertEquals(
            StaticIntervalAccelerometerCalibrator.DEFAULT_ROBUST_THRESHOLD_FACTOR,
            calibrator.magnetometerRobustThresholdFactor,
            0.0
        )

        // set new value
        calibrator.magnetometerRobustThresholdFactor = ROBUST_THRESHOLD_FACTOR

        // check
        assertEquals(ROBUST_THRESHOLD_FACTOR, calibrator.magnetometerRobustThresholdFactor, 0.0)
    }

    @Test(expected = IllegalArgumentException::class)
    fun magnetometerRobustThresholdFactor_whenInvalid_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        calibrator.magnetometerRobustThresholdFactor = 0.0
    }

    @Test(expected = IllegalStateException::class)
    fun magnetometerRobustThresholdFactor_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            accelerometerAndGyroscopeCalibrator,
            "running",
            true
        )
        setPrivateProperty(
            SingleSensorStaticIntervalCalibrator::class,
            magnetometerCalibrator,
            "running",
            true
        )
        assertTrue(calibrator.running)

        calibrator.magnetometerRobustThresholdFactor = ROBUST_THRESHOLD_FACTOR
    }

    @Test
    fun magnetometerRobustStopThresholdFactor_whenValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        assertEquals(
            StaticIntervalAccelerometerCalibrator.DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR,
            calibrator.magnetometerRobustStopThresholdFactor,
            0.0
        )

        // set new value
        calibrator.magnetometerRobustStopThresholdFactor = ROBUST_STOP_THRESHOLD_FACTOR

        // check
        assertEquals(
            ROBUST_STOP_THRESHOLD_FACTOR,
            calibrator.magnetometerRobustStopThresholdFactor,
            0.0
        )
    }

    @Test(expected = IllegalArgumentException::class)
    fun magnetometerRobustStopThresholdFactor_whenInvalid_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        calibrator.magnetometerRobustStopThresholdFactor = 0.0
    }

    @Test(expected = IllegalStateException::class)
    fun magnetometerRobustStopThresholdFactor_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            accelerometerAndGyroscopeCalibrator,
            "running",
            true
        )
        setPrivateProperty(
            SingleSensorStaticIntervalCalibrator::class,
            magnetometerCalibrator,
            "running",
            true
        )
        assertTrue(calibrator.running)

        calibrator.magnetometerRobustStopThresholdFactor = ROBUST_STOP_THRESHOLD_FACTOR
    }

    @Test
    fun windowSize_whenValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        assertEquals(TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE, calibrator.windowSize)

        // set new value
        calibrator.windowSize = WINDOW_SIZE

        // check
        assertEquals(WINDOW_SIZE, calibrator.windowSize)
    }

    @Test(expected = IllegalArgumentException::class)
    fun windowSize_whenInvalid_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        calibrator.windowSize = 0
    }

    @Test(expected = IllegalStateException::class)
    fun windowSize_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            accelerometerAndGyroscopeCalibrator,
            "running",
            true
        )
        setPrivateProperty(
            SingleSensorStaticIntervalCalibrator::class,
            magnetometerCalibrator,
            "running",
            true
        )
        assertTrue(calibrator.running)

        calibrator.windowSize = WINDOW_SIZE
    }

    @Test
    fun initialStaticSamples_whenValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES,
            calibrator.initialStaticSamples
        )

        // set new value
        calibrator.initialStaticSamples = INITIAL_STATIC_SAMPLES

        // check
        assertEquals(INITIAL_STATIC_SAMPLES, calibrator.initialStaticSamples)
    }

    @Test(expected = IllegalArgumentException::class)
    fun initialStaticSamples_whenInvalid_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        calibrator.initialStaticSamples = 0
    }

    @Test(expected = IllegalStateException::class)
    fun initialStaticSamples_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            accelerometerAndGyroscopeCalibrator,
            "running",
            true
        )
        setPrivateProperty(
            SingleSensorStaticIntervalCalibrator::class,
            magnetometerCalibrator,
            "running",
            true
        )
        assertTrue(calibrator.running)

        calibrator.initialStaticSamples = INITIAL_STATIC_SAMPLES
    }

    @Test
    fun thresholdFactor_whenValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_THRESHOLD_FACTOR,
            calibrator.thresholdFactor,
            0.0
        )

        // set new value
        calibrator.thresholdFactor = THRESHOLD_FACTOR

        // check
        assertEquals(THRESHOLD_FACTOR, calibrator.thresholdFactor, 0.0)
    }

    @Test(expected = IllegalArgumentException::class)
    fun thresholdFactor_whenInvalid_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        calibrator.thresholdFactor = 0.0
    }

    @Test(expected = IllegalStateException::class)
    fun thresholdFactor_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            accelerometerAndGyroscopeCalibrator,
            "running",
            true
        )
        setPrivateProperty(
            SingleSensorStaticIntervalCalibrator::class,
            magnetometerCalibrator,
            "running",
            true
        )
        assertTrue(calibrator.running)

        calibrator.thresholdFactor = THRESHOLD_FACTOR
    }

    @Test
    fun instantaneousNoiseLevelFactor_whenValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR,
            calibrator.instantaneousNoiseLevelFactor,
            0.0
        )

        // set new value
        calibrator.instantaneousNoiseLevelFactor = INSTANTANEOUS_NOISE_LEVEL_FACTOR

        // check
        assertEquals(
            INSTANTANEOUS_NOISE_LEVEL_FACTOR,
            calibrator.instantaneousNoiseLevelFactor,
            0.0
        )
    }

    @Test(expected = IllegalArgumentException::class)
    fun instantaneousNoiseLevelFactor_whenInvalid_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        calibrator.instantaneousNoiseLevelFactor = 0.0
    }

    @Test(expected = IllegalStateException::class)
    fun instantaneousNoiseLevelFactor_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            accelerometerAndGyroscopeCalibrator,
            "running",
            true
        )
        setPrivateProperty(
            SingleSensorStaticIntervalCalibrator::class,
            magnetometerCalibrator,
            "running",
            true
        )
        assertTrue(calibrator.running)

        calibrator.instantaneousNoiseLevelFactor = INSTANTANEOUS_NOISE_LEVEL_FACTOR
    }

    @Test
    fun baseNoiseLevelAbsoluteThreshold_whenValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
            calibrator.baseNoiseLevelAbsoluteThreshold,
            0.0
        )

        // set new value
        calibrator.baseNoiseLevelAbsoluteThreshold = BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD

        // check
        assertEquals(
            BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
            calibrator.baseNoiseLevelAbsoluteThreshold,
            0.0
        )
    }

    @Test(expected = IllegalArgumentException::class)
    fun baseNoiseLevelAbsoluteThreshold_whenInvalid_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        calibrator.baseNoiseLevelAbsoluteThreshold = 0.0
    }

    @Test(expected = IllegalStateException::class)
    fun baseNoiseLevelAbsoluteThreshold_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            accelerometerAndGyroscopeCalibrator,
            "running",
            true
        )
        setPrivateProperty(
            SingleSensorStaticIntervalCalibrator::class,
            magnetometerCalibrator,
            "running",
            true
        )
        assertTrue(calibrator.running)

        calibrator.baseNoiseLevelAbsoluteThreshold = BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD
    }

    @Test
    fun baseNoiseLevelAbsoluteThresholdAsMeasurement_whenValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        val value1 = calibrator.baseNoiseLevelAbsoluteThresholdAsMeasurement
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
            value1.value.toDouble(),
            0.0
        )
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, value1.unit)

        // set new value
        val value2 = Acceleration(
            BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
            AccelerationUnit.METERS_PER_SQUARED_SECOND
        )
        calibrator.baseNoiseLevelAbsoluteThresholdAsMeasurement = value2

        // check
        val value3 = calibrator.baseNoiseLevelAbsoluteThresholdAsMeasurement
        assertEquals(value2, value3)
    }

    @Test(expected = IllegalArgumentException::class)
    fun baseNoiseLevelAbsoluteThresholdAsMeasurement_whenInvalid_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        val value = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        calibrator.baseNoiseLevelAbsoluteThresholdAsMeasurement = value
    }

    @Test(expected = IllegalStateException::class)
    fun baseNoiseLevelAbsoluteThresholdAsMeasurement_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            accelerometerAndGyroscopeCalibrator,
            "running",
            true
        )
        setPrivateProperty(
            SingleSensorStaticIntervalCalibrator::class,
            magnetometerCalibrator,
            "running",
            true
        )
        assertTrue(calibrator.running)

        val value = Acceleration(
            BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
            AccelerationUnit.METERS_PER_SQUARED_SECOND
        )
        calibrator.baseNoiseLevelAbsoluteThresholdAsMeasurement = value
    }

    @Test
    fun getBaseNoiseLevelAbsoluteThresholdAsMeasurement_getsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        val value1 = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        calibrator.getBaseNoiseLevelAbsoluteThresholdAsMeasurement(value1)

        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
            value1.value.toDouble(),
            0.0
        )
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, value1.unit)

        // set new value
        val value2 = Acceleration(
            BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
            AccelerationUnit.METERS_PER_SQUARED_SECOND
        )
        calibrator.baseNoiseLevelAbsoluteThresholdAsMeasurement = value2

        // check
        val value3 = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        calibrator.getBaseNoiseLevelAbsoluteThresholdAsMeasurement(value3)
        assertEquals(value2, value3)
    }

    @Test
    fun requiredMeasurements_whenValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertEquals(
            StaticIntervalGyroscopeCalibrator.GYROSCOPE_UNKNOWN_BIAS_MINIMUM_SEQUENCES_GENERAL,
            calibrator.requiredMeasurements
        )

        // set new value
        calibrator.requiredMeasurements = REQUIRED_MEASUREMENTS

        // check
        assertEquals(REQUIRED_MEASUREMENTS, calibrator.requiredMeasurements)
    }

    @Test(expected = IllegalArgumentException::class)
    fun requiredMeasurements_whenInvalid_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        calibrator.requiredMeasurements = 0
    }

    @Test(expected = IllegalStateException::class)
    fun requiredMeasurements_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            accelerometerAndGyroscopeCalibrator,
            "running",
            true
        )
        setPrivateProperty(
            SingleSensorStaticIntervalCalibrator::class,
            magnetometerCalibrator,
            "running",
            true
        )
        assertTrue(calibrator.running)

        calibrator.requiredMeasurements = REQUIRED_MEASUREMENTS
    }

    @Test
    fun onInitializationStarted_whenAccelerometerAndGyroscopeAndMagnetometerNotInitializedAndNoListener_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        val accelerometerAndGyroscopeInitializationStarted1: Boolean? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeInitializationStarted")
        requireNotNull(accelerometerAndGyroscopeInitializationStarted1)
        assertFalse(accelerometerAndGyroscopeInitializationStarted1)
        val magnetometerInitializationStarted: Boolean? =
            calibrator.getPrivateProperty("magnetometerInitializationStarted")
        requireNotNull(magnetometerInitializationStarted)
        assertFalse(magnetometerInitializationStarted)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val listener = accelerometerAndGyroscopeCalibrator.initializationStartedListener
        requireNotNull(listener)

        listener.onInitializationStarted(accelerometerAndGyroscopeCalibrator)

        // check
        val accelerometerAndGyroscopeInitializationStarted2: Boolean? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeInitializationStarted")
        requireNotNull(accelerometerAndGyroscopeInitializationStarted2)
        assertTrue(accelerometerAndGyroscopeInitializationStarted2)
    }

    @Test
    fun onInitializationStarted_whenAccelerometerAndGyroscopeAndMagnetometerAlreadyInitialized_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(
            context,
            initializationStartedListener = initializationStartedListener
        )

        calibrator.setPrivateProperty("magnetometerInitializationStarted", true)

        val accelerometerAndGyroscopeInitializationStarted1: Boolean? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeInitializationStarted")
        requireNotNull(accelerometerAndGyroscopeInitializationStarted1)
        assertFalse(accelerometerAndGyroscopeInitializationStarted1)
        val magnetometerInitializationStarted: Boolean? =
            calibrator.getPrivateProperty("magnetometerInitializationStarted")
        requireNotNull(magnetometerInitializationStarted)
        assertTrue(magnetometerInitializationStarted)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val listener = accelerometerAndGyroscopeCalibrator.initializationStartedListener
        requireNotNull(listener)

        listener.onInitializationStarted(accelerometerAndGyroscopeCalibrator)

        // check
        val accelerometerAndGyroscopeInitializationStarted2: Boolean? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeInitializationStarted")
        requireNotNull(accelerometerAndGyroscopeInitializationStarted2)
        assertTrue(accelerometerAndGyroscopeInitializationStarted2)

        verifyNoInteractions(initializationStartedListener)
//        verify { initializationStartedListener wasNot Called }
    }

    @Test
    fun onInitializationStarted_whenAccelerometerAndGyroscopeAndMagnetometerNotInitialized_notifies() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(
            context,
            initializationStartedListener = initializationStartedListener
        )

        val accelerometerAndGyroscopeInitializationStarted1: Boolean? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeInitializationStarted")
        requireNotNull(accelerometerAndGyroscopeInitializationStarted1)
        assertFalse(accelerometerAndGyroscopeInitializationStarted1)
        val magnetometerInitializationStarted: Boolean? =
            calibrator.getPrivateProperty("magnetometerInitializationStarted")
        requireNotNull(magnetometerInitializationStarted)
        assertFalse(magnetometerInitializationStarted)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val listener = accelerometerAndGyroscopeCalibrator.initializationStartedListener
        requireNotNull(listener)

        listener.onInitializationStarted(accelerometerAndGyroscopeCalibrator)

        // check
        val accelerometerAndGyroscopeInitializationStarted2: Boolean? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeInitializationStarted")
        requireNotNull(accelerometerAndGyroscopeInitializationStarted2)
        assertTrue(accelerometerAndGyroscopeInitializationStarted2)

        verify(initializationStartedListener, only()).onInitializationStarted(calibrator)
//        verify(exactly = 1) { initializationStartedListener.onInitializationStarted(calibrator) }
    }

    @Test
    fun onInitializationStarted_whenMagnetometerAndAccelerometerAndGyroscopeNotInitializedAndNoListener_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        val accelerometerAndGyroscopeInitializationStarted1: Boolean? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeInitializationStarted")
        requireNotNull(accelerometerAndGyroscopeInitializationStarted1)
        assertFalse(accelerometerAndGyroscopeInitializationStarted1)
        val magnetometerInitializationStarted: Boolean? =
            calibrator.getPrivateProperty("magnetometerInitializationStarted")
        requireNotNull(magnetometerInitializationStarted)
        assertFalse(magnetometerInitializationStarted)

        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        val listener = magnetometerCalibrator.initializationStartedListener
        requireNotNull(listener)

        listener.onInitializationStarted(magnetometerCalibrator)

        // check
        val magnetometerInitializationStarted2: Boolean? =
            calibrator.getPrivateProperty("magnetometerInitializationStarted")
        requireNotNull(magnetometerInitializationStarted2)
        assertTrue(magnetometerInitializationStarted2)
    }

    @Test
    fun onInitializationStarted_whenMagnetometerAndAccelerometerAndGyroscopeAlreadyInitialized_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(
            context,
            initializationStartedListener = initializationStartedListener
        )

        calibrator.setPrivateProperty("accelerometerAndGyroscopeInitializationStarted", true)

        val accelerometerAndGyroscopeInitializationStarted1: Boolean? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeInitializationStarted")
        requireNotNull(accelerometerAndGyroscopeInitializationStarted1)
        assertTrue(accelerometerAndGyroscopeInitializationStarted1)
        val magnetometerInitializationStarted: Boolean? =
            calibrator.getPrivateProperty("magnetometerInitializationStarted")
        requireNotNull(magnetometerInitializationStarted)
        assertFalse(magnetometerInitializationStarted)

        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        val listener = magnetometerCalibrator.initializationStartedListener
        requireNotNull(listener)

        listener.onInitializationStarted(magnetometerCalibrator)

        // check
        val magnetometerInitializationStarted2: Boolean? =
            calibrator.getPrivateProperty("magnetometerInitializationStarted")
        requireNotNull(magnetometerInitializationStarted2)
        assertTrue(magnetometerInitializationStarted2)

        verifyNoInteractions(initializationStartedListener)
//        verify { initializationStartedListener wasNot Called }
    }

    @Test
    fun onInitializationStarted_whenMagnetometerAndAccelerometerAndGyroscopeNotInitialized_notifies() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(
            context,
            initializationStartedListener = initializationStartedListener
        )

        val accelerometerAndGyroscopeInitializationStarted1: Boolean? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeInitializationStarted")
        requireNotNull(accelerometerAndGyroscopeInitializationStarted1)
        assertFalse(accelerometerAndGyroscopeInitializationStarted1)
        val magnetometerInitializationStarted: Boolean? =
            calibrator.getPrivateProperty("magnetometerInitializationStarted")
        requireNotNull(magnetometerInitializationStarted)
        assertFalse(magnetometerInitializationStarted)

        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        val listener = magnetometerCalibrator.initializationStartedListener
        requireNotNull(listener)

        listener.onInitializationStarted(magnetometerCalibrator)

        // check
        val magnetometerInitializationStarted2: Boolean? =
            calibrator.getPrivateProperty("magnetometerInitializationStarted")
        requireNotNull(magnetometerInitializationStarted2)
        assertTrue(magnetometerInitializationStarted2)

        verify(initializationStartedListener, only()).onInitializationStarted(calibrator)
//        verify(exactly = 1) { initializationStartedListener.onInitializationStarted(calibrator) }
    }

    @Test
    fun onInitializationCompleted_whenAccelerometerAndGyroscopeAndMagnetometerCompletedAndNoListener_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        calibrator.setPrivateProperty("magnetometerInitializationCompleted", true)

        val accelerometerAndGyroscopeInitializationCompleted1: Boolean? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeInitializationCompleted")
        requireNotNull(accelerometerAndGyroscopeInitializationCompleted1)
        assertFalse(accelerometerAndGyroscopeInitializationCompleted1)
        val magnetometerInitializationCompleted: Boolean? =
            calibrator.getPrivateProperty("magnetometerInitializationCompleted")
        requireNotNull(magnetometerInitializationCompleted)
        assertTrue(magnetometerInitializationCompleted)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val listener = accelerometerAndGyroscopeCalibrator.initializationCompletedListener
        requireNotNull(listener)

        listener.onInitializationCompleted(accelerometerAndGyroscopeCalibrator)

        val accelerometerAndGyroscopeInitializationCompleted2: Boolean? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeInitializationCompleted")
        requireNotNull(accelerometerAndGyroscopeInitializationCompleted2)
        assertTrue(accelerometerAndGyroscopeInitializationCompleted2)
    }

    @Test
    fun onInitializationCompleted_whenAccelerometerAndGyroscopeAndMagnetometerNotCompleted_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(
            context,
            initializationCompletedListener = initializationCompletedListener
        )

        val accelerometerAndGyroscopeInitializationCompleted1: Boolean? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeInitializationCompleted")
        requireNotNull(accelerometerAndGyroscopeInitializationCompleted1)
        assertFalse(accelerometerAndGyroscopeInitializationCompleted1)
        val magnetometerInitializationCompleted: Boolean? =
            calibrator.getPrivateProperty("magnetometerInitializationCompleted")
        requireNotNull(magnetometerInitializationCompleted)
        assertFalse(magnetometerInitializationCompleted)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val listener = accelerometerAndGyroscopeCalibrator.initializationCompletedListener
        requireNotNull(listener)

        listener.onInitializationCompleted(accelerometerAndGyroscopeCalibrator)

        val accelerometerAndGyroscopeInitializationCompleted2: Boolean? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeInitializationCompleted")
        requireNotNull(accelerometerAndGyroscopeInitializationCompleted2)
        assertTrue(accelerometerAndGyroscopeInitializationCompleted2)

        verifyNoInteractions(initializationCompletedListener)
//        verify { initializationCompletedListener wasNot Called }
    }

    @Test
    fun onInitializationCompleted_whenAccelerometerAndGyroscopeAndMagnetometerAlreadyCompleted_notifies() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(
            context,
            initializationCompletedListener = initializationCompletedListener
        )

        calibrator.setPrivateProperty("magnetometerInitializationCompleted", true)

        val accelerometerAndGyroscopeInitializationCompleted1: Boolean? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeInitializationCompleted")
        requireNotNull(accelerometerAndGyroscopeInitializationCompleted1)
        assertFalse(accelerometerAndGyroscopeInitializationCompleted1)
        val magnetometerInitializationCompleted: Boolean? =
            calibrator.getPrivateProperty("magnetometerInitializationCompleted")
        requireNotNull(magnetometerInitializationCompleted)
        assertTrue(magnetometerInitializationCompleted)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val listener = accelerometerAndGyroscopeCalibrator.initializationCompletedListener
        requireNotNull(listener)

        listener.onInitializationCompleted(accelerometerAndGyroscopeCalibrator)

        val accelerometerAndGyroscopeInitializationCompleted2: Boolean? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeInitializationCompleted")
        requireNotNull(accelerometerAndGyroscopeInitializationCompleted2)
        assertTrue(accelerometerAndGyroscopeInitializationCompleted2)

        verify(initializationCompletedListener, only()).onInitializationCompleted(calibrator)
//        verify(exactly = 1) { initializationCompletedListener.onInitializationCompleted(calibrator) }
    }

    @Test
    fun onInitializationCompleted_whenMagnetometerAndAccelerometerAndGyroscopeCompletedAndNoListener_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        calibrator.setPrivateProperty("accelerometerAndGyroscopeInitializationCompleted", true)

        val accelerometerAndGyroscopeInitializationCompleted: Boolean? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeInitializationCompleted")
        requireNotNull(accelerometerAndGyroscopeInitializationCompleted)
        assertTrue(accelerometerAndGyroscopeInitializationCompleted)
        val magnetometerInitializationCompleted1: Boolean? =
            calibrator.getPrivateProperty("magnetometerInitializationCompleted")
        requireNotNull(magnetometerInitializationCompleted1)
        assertFalse(magnetometerInitializationCompleted1)

        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        val listener = magnetometerCalibrator.initializationCompletedListener
        requireNotNull(listener)

        listener.onInitializationCompleted(magnetometerCalibrator)

        val magnetometerInitializationCompleted2: Boolean? =
            calibrator.getPrivateProperty("magnetometerInitializationCompleted")
        requireNotNull(magnetometerInitializationCompleted2)
        assertTrue(magnetometerInitializationCompleted2)
    }

    @Test
    fun onInitializationCompleted_whenMagnetometerAndAccelerometerAndGyroscopeNotCompleted_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(
            context,
            initializationCompletedListener = initializationCompletedListener
        )

        val accelerometerAndGyroscopeInitializationCompleted: Boolean? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeInitializationCompleted")
        requireNotNull(accelerometerAndGyroscopeInitializationCompleted)
        assertFalse(accelerometerAndGyroscopeInitializationCompleted)
        val magnetometerInitializationCompleted1: Boolean? =
            calibrator.getPrivateProperty("magnetometerInitializationCompleted")
        requireNotNull(magnetometerInitializationCompleted1)
        assertFalse(magnetometerInitializationCompleted1)

        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        val listener = magnetometerCalibrator.initializationCompletedListener
        requireNotNull(listener)

        listener.onInitializationCompleted(magnetometerCalibrator)

        val magnetometerInitializationCompleted2: Boolean? =
            calibrator.getPrivateProperty("magnetometerInitializationCompleted")
        requireNotNull(magnetometerInitializationCompleted2)
        assertTrue(magnetometerInitializationCompleted2)

        verifyNoInteractions(initializationCompletedListener)
//        verify { initializationCompletedListener wasNot Called }
    }

    @Test
    fun onInitializationCompleted_whenMagnetometerAndAccelerometerAndGyroscopeAlreadyCompleted_notifies() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(
            context,
            initializationCompletedListener = initializationCompletedListener
        )

        calibrator.setPrivateProperty("accelerometerAndGyroscopeInitializationCompleted", true)

        val accelerometerAndGyroscopeInitializationCompleted: Boolean? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeInitializationCompleted")
        requireNotNull(accelerometerAndGyroscopeInitializationCompleted)
        assertTrue(accelerometerAndGyroscopeInitializationCompleted)
        val magnetometerInitializationCompleted1: Boolean? =
            calibrator.getPrivateProperty("magnetometerInitializationCompleted")
        requireNotNull(magnetometerInitializationCompleted1)
        assertFalse(magnetometerInitializationCompleted1)

        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        val listener = magnetometerCalibrator.initializationCompletedListener
        requireNotNull(listener)

        listener.onInitializationCompleted(magnetometerCalibrator)

        val magnetometerInitializationCompleted2: Boolean? =
            calibrator.getPrivateProperty("magnetometerInitializationCompleted")
        requireNotNull(magnetometerInitializationCompleted2)
        assertTrue(magnetometerInitializationCompleted2)

        verify(initializationCompletedListener, only()).onInitializationCompleted(calibrator)
//        verify(exactly = 1) { initializationCompletedListener.onInitializationCompleted(calibrator) }
    }

    @Test
    fun onError_whenAccelerometerAndGyroscopeAndNoListener_stops() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val accelerometerAndGyroscopeCalibratorSpy = spy(accelerometerAndGyroscopeCalibrator)
//        val accelerometerAndGyroscopeCalibratorSpy = spyk(accelerometerAndGyroscopeCalibrator)
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibratorSpy
        )
        val listener = accelerometerAndGyroscopeCalibrator.errorListener
        requireNotNull(listener)
        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        val magnetometerCalibratorSpy = spy(magnetometerCalibrator)
//        val magnetometerCalibratorSpy = spyk(magnetometerCalibrator)
        calibrator.setPrivateProperty("magnetometerCalibrator", magnetometerCalibratorSpy)

        listener.onError(
            accelerometerAndGyroscopeCalibrator,
            CalibratorErrorReason.OVERALL_EXCESSIVE_MOVEMENT_DETECTED_DURING_INITIALIZATION
        )

        verify(accelerometerAndGyroscopeCalibratorSpy, times(1)).stop()
        verify(magnetometerCalibratorSpy, times(1)).stop()
//        verify(exactly = 1) { accelerometerAndGyroscopeCalibratorSpy.stop() }
//        verify(exactly = 1) { magnetometerCalibratorSpy.stop() }
    }

    @Test
    fun onError_whenAccelerometerAndGyroscope_stopsAndNotifies() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(
            context,
            errorListener = errorListener
        )

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val accelerometerAndGyroscopeCalibratorSpy = spy(accelerometerAndGyroscopeCalibrator)
//        val accelerometerAndGyroscopeCalibratorSpy = spyk(accelerometerAndGyroscopeCalibrator)
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibratorSpy
        )
        val listener = accelerometerAndGyroscopeCalibrator.errorListener
        requireNotNull(listener)
        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        val magnetometerCalibratorSpy = spy(magnetometerCalibrator)
//        val magnetometerCalibratorSpy = spyk(magnetometerCalibrator)
        calibrator.setPrivateProperty("magnetometerCalibrator", magnetometerCalibratorSpy)

        listener.onError(
            accelerometerAndGyroscopeCalibrator,
            CalibratorErrorReason.OVERALL_EXCESSIVE_MOVEMENT_DETECTED_DURING_INITIALIZATION
        )

        verify(accelerometerAndGyroscopeCalibratorSpy, times(1)).stop()
        verify(magnetometerCalibratorSpy, times(1)).stop()
//        verify(exactly = 1) { accelerometerAndGyroscopeCalibratorSpy.stop() }
//        verify(exactly = 1) { magnetometerCalibratorSpy.stop() }

        verify(errorListener, only()).onError(
            calibrator,
            CalibratorErrorReason.OVERALL_EXCESSIVE_MOVEMENT_DETECTED_DURING_INITIALIZATION
        )
/*        verify(exactly = 1) {
            errorListener.onError(
                calibrator,
                CalibratorErrorReason.OVERALL_EXCESSIVE_MOVEMENT_DETECTED_DURING_INITIALIZATION
            )
        }*/
    }

    @Test
    fun onError_whenMagnetometerAndNoListener_stops() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val accelerometerAndGyroscopeCalibratorSpy = spy(accelerometerAndGyroscopeCalibrator)
//        val accelerometerAndGyroscopeCalibratorSpy = spyk(accelerometerAndGyroscopeCalibrator)
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibratorSpy
        )
        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        val magnetometerCalibratorSpy = spy(magnetometerCalibrator)
//        val magnetometerCalibratorSpy = spyk(magnetometerCalibrator)
        calibrator.setPrivateProperty("magnetometerCalibrator", magnetometerCalibratorSpy)
        val listener = magnetometerCalibrator.errorListener
        requireNotNull(listener)

        listener.onError(
            magnetometerCalibrator,
            CalibratorErrorReason.OVERALL_EXCESSIVE_MOVEMENT_DETECTED_DURING_INITIALIZATION
        )

        verify(accelerometerAndGyroscopeCalibratorSpy, times(1)).stop()
        verify(magnetometerCalibratorSpy, times(1)).stop()
//        verify(exactly = 1) { accelerometerAndGyroscopeCalibratorSpy.stop() }
//        verify(exactly = 1) { magnetometerCalibratorSpy.stop() }
    }

//    @Test
    fun onError_whenMagnetometer_stopsAndNotifies() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(
            context,
            errorListener = errorListener
        )

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val accelerometerAndGyroscopeCalibratorSpy = spy(accelerometerAndGyroscopeCalibrator)
//        val accelerometerAndGyroscopeCalibratorSpy = spyk(accelerometerAndGyroscopeCalibrator)
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibratorSpy
        )
        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        val magnetometerCalibratorSpy = spy(magnetometerCalibrator)
//        val magnetometerCalibratorSpy = spyk(magnetometerCalibrator)
        calibrator.setPrivateProperty("magnetometerCalibrator", magnetometerCalibratorSpy)
        val listener = magnetometerCalibrator.errorListener
        requireNotNull(listener)

        listener.onError(
            magnetometerCalibrator,
            CalibratorErrorReason.OVERALL_EXCESSIVE_MOVEMENT_DETECTED_DURING_INITIALIZATION
        )

        verify(accelerometerAndGyroscopeCalibratorSpy, only()).stop()
        verify(magnetometerCalibratorSpy, only()).stop()
//        verify(exactly = 1) { accelerometerAndGyroscopeCalibratorSpy.stop() }
//        verify(exactly = 1) { magnetometerCalibratorSpy.stop() }

        verify(errorListener, only()).onError(
            calibrator,
            CalibratorErrorReason.OVERALL_EXCESSIVE_MOVEMENT_DETECTED_DURING_INITIALIZATION
        )
/*        verify(exactly = 1) {
            errorListener.onError(
                calibrator,
                CalibratorErrorReason.OVERALL_EXCESSIVE_MOVEMENT_DETECTED_DURING_INITIALIZATION
            )
        }*/
    }

    @Test
    fun onStaticIntervalDetected_whenNoListener_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val listener = accelerometerAndGyroscopeCalibrator.staticIntervalDetectedListener
        requireNotNull(listener)

        listener.onStaticIntervalDetected(accelerometerAndGyroscopeCalibrator)
    }

    @Test
    fun onStaticIntervalDetected_whenListener_notifies() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(
            context,
            staticIntervalDetectedListener = staticIntervalDetectedListener
        )

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val listener = accelerometerAndGyroscopeCalibrator.staticIntervalDetectedListener
        requireNotNull(listener)

        listener.onStaticIntervalDetected(accelerometerAndGyroscopeCalibrator)

        verify(staticIntervalDetectedListener, only()).onStaticIntervalDetected(calibrator)
//        verify(exactly = 1) { staticIntervalDetectedListener.onStaticIntervalDetected(calibrator) }
    }

    @Test
    fun onDynamicIntervalDetected_whenNoListener_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val listener = accelerometerAndGyroscopeCalibrator.dynamicIntervalDetectedListener
        requireNotNull(listener)

        listener.onDynamicIntervalDetected(accelerometerAndGyroscopeCalibrator)
    }

    @Test
    fun onDynamicIntervalDetected_whenListener_notifies() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(
            context,
            dynamicIntervalDetectedListener = dynamicIntervalDetectedListener
        )

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val listener = accelerometerAndGyroscopeCalibrator.dynamicIntervalDetectedListener
        requireNotNull(listener)

        listener.onDynamicIntervalDetected(accelerometerAndGyroscopeCalibrator)

        verify(dynamicIntervalDetectedListener, only()).onDynamicIntervalDetected(calibrator)
//        verify(exactly = 1) { dynamicIntervalDetectedListener.onDynamicIntervalDetected(calibrator) }
    }

    @Test
    fun onStaticIntervalSkipped_whenNoListener_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val listener = accelerometerAndGyroscopeCalibrator.staticIntervalSkippedListener
        requireNotNull(listener)

        listener.onStaticIntervalSkipped(accelerometerAndGyroscopeCalibrator)
    }

    @Test
    fun onStaticIntervalSkipped_whenListener_notifies() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(
            context,
            staticIntervalSkippedListener = staticIntervalSkippedListener
        )

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val listener = accelerometerAndGyroscopeCalibrator.staticIntervalSkippedListener
        requireNotNull(listener)

        listener.onStaticIntervalSkipped(accelerometerAndGyroscopeCalibrator)

        verify(staticIntervalSkippedListener, only()).onStaticIntervalSkipped(calibrator)
//        verify(exactly = 1) { staticIntervalSkippedListener.onStaticIntervalSkipped(calibrator) }
    }

    @Test
    fun onDynamicIntervalSkipped_whenNoListener_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val listener = accelerometerAndGyroscopeCalibrator.dynamicIntervalSkippedListener
        requireNotNull(listener)

        listener.onDynamicIntervalSkipped(accelerometerAndGyroscopeCalibrator)
    }

    @Test
    fun onDynamicIntervalSkipped_whenListener_notifies() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(
            context,
            dynamicIntervalSkippedListener = dynamicIntervalSkippedListener
        )

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val listener = accelerometerAndGyroscopeCalibrator.dynamicIntervalSkippedListener
        requireNotNull(listener)

        listener.onDynamicIntervalSkipped(accelerometerAndGyroscopeCalibrator)

        verify(dynamicIntervalSkippedListener, only()).onDynamicIntervalSkipped(calibrator)
//        verify(exactly = 1) { dynamicIntervalSkippedListener.onDynamicIntervalSkipped(calibrator) }
    }

    @Test
    fun onGeneratedAccelerometerMeasurement_whenNoListener_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val listener = accelerometerAndGyroscopeCalibrator.generatedAccelerometerMeasurementListener
        requireNotNull(listener)

        val measurement = StandardDeviationBodyKinematics()
        listener.onGeneratedAccelerometerMeasurement(
            accelerometerAndGyroscopeCalibrator,
            measurement,
            1,
            20
        )
    }

    @Test
    fun onGeneratedAccelerometerMeasurement_whenListener_notifies() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(
            context,
            generatedAccelerometerMeasurementListener = generatedAccelerometerMeasurementListener
        )

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val listener = accelerometerAndGyroscopeCalibrator.generatedAccelerometerMeasurementListener
        requireNotNull(listener)

        val measurement = StandardDeviationBodyKinematics()
        listener.onGeneratedAccelerometerMeasurement(
            accelerometerAndGyroscopeCalibrator,
            measurement,
            1,
            20
        )

        verify(generatedAccelerometerMeasurementListener, only()).onGeneratedAccelerometerMeasurement(
            calibrator,
            measurement,
            1,
            20
        )
/*        verify(exactly = 1) {
            generatedAccelerometerMeasurementListener.onGeneratedAccelerometerMeasurement(
                calibrator,
                measurement,
                1,
                20
            )
        }*/
    }

    @Test
    fun onGeneratedGyroscopeMeasurementListener_whenNoListener_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val listener = accelerometerAndGyroscopeCalibrator.generatedGyroscopeMeasurementListener
        requireNotNull(listener)

        val measurement = BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>()
        listener.onGeneratedGyroscopeMeasurement(
            accelerometerAndGyroscopeCalibrator,
            measurement,
            2,
            30
        )
    }

    @Test
    fun onGeneratedGyroscopeMeasurementListener_whenListener_notifies() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(
            context,
            generatedGyroscopeMeasurementListener = generatedGyroscopeMeasurementListener
        )

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val listener = accelerometerAndGyroscopeCalibrator.generatedGyroscopeMeasurementListener
        requireNotNull(listener)

        val measurement = BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>()
        listener.onGeneratedGyroscopeMeasurement(
            accelerometerAndGyroscopeCalibrator,
            measurement,
            2,
            30
        )

        verify(generatedGyroscopeMeasurementListener, only()).onGeneratedGyroscopeMeasurement(
            calibrator,
            measurement,
            2,
            30
        )
/*        verify(exactly = 1) {
            generatedGyroscopeMeasurementListener.onGeneratedGyroscopeMeasurement(
                calibrator,
                measurement,
                2,
                30
            )
        }*/
    }

    @Test
    fun onReadyToSolveCalibration_whenAccelerometerAndGyroscopeAndMagnetometerReadyToSolveCalibrationAndNoListener_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        calibrator.setPrivateProperty("magnetometerReadyToSolveCalibration", true)

        val accelerometerAndGyroscopeReadyToSolveCalibration1: Boolean? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeReadyToSolveCalibration")
        requireNotNull(accelerometerAndGyroscopeReadyToSolveCalibration1)
        assertFalse(accelerometerAndGyroscopeReadyToSolveCalibration1)
        val magnetometerReadyToSolveCalibration: Boolean? =
            calibrator.getPrivateProperty("magnetometerReadyToSolveCalibration")
        requireNotNull(magnetometerReadyToSolveCalibration)
        assertTrue(magnetometerReadyToSolveCalibration)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val listener = accelerometerAndGyroscopeCalibrator.readyToSolveCalibrationListener
        requireNotNull(listener)

        listener.onReadyToSolveCalibration(accelerometerAndGyroscopeCalibrator)

        val accelerometerAndGyroscopeReadyToSolveCalibration2: Boolean? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeReadyToSolveCalibration")
        requireNotNull(accelerometerAndGyroscopeReadyToSolveCalibration2)
        assertTrue(accelerometerAndGyroscopeReadyToSolveCalibration2)
    }

    @Test
    fun onReadyToSolveCalibration_whenAccelerometerAndGyroscopeAndMagnetometerNotReadyToSolveCalibration_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(
            context,
            readyToSolveCalibrationListener = readyToSolveCalibrationListener
        )

        val accelerometerAndGyroscopeReadyToSolveCalibration1: Boolean? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeReadyToSolveCalibration")
        requireNotNull(accelerometerAndGyroscopeReadyToSolveCalibration1)
        assertFalse(accelerometerAndGyroscopeReadyToSolveCalibration1)
        val magnetometerReadyToSolveCalibration: Boolean? =
            calibrator.getPrivateProperty("magnetometerReadyToSolveCalibration")
        requireNotNull(magnetometerReadyToSolveCalibration)
        assertFalse(magnetometerReadyToSolveCalibration)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val listener = accelerometerAndGyroscopeCalibrator.readyToSolveCalibrationListener
        requireNotNull(listener)

        listener.onReadyToSolveCalibration(accelerometerAndGyroscopeCalibrator)

        val accelerometerAndGyroscopeReadyToSolveCalibration2: Boolean? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeReadyToSolveCalibration")
        requireNotNull(accelerometerAndGyroscopeReadyToSolveCalibration2)
        assertTrue(accelerometerAndGyroscopeReadyToSolveCalibration2)

        verifyNoInteractions(readyToSolveCalibrationListener)
//        verify { readyToSolveCalibrationListener wasNot Called }
    }

    @Test
    fun onReadyToSolveCalibration_whenAccelerometerAndGyroscopeAndMagnetometerReadyToSolveCalibration_notifies() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(
            context,
            readyToSolveCalibrationListener = readyToSolveCalibrationListener
        )

        calibrator.setPrivateProperty("magnetometerReadyToSolveCalibration", true)

        val accelerometerAndGyroscopeReadyToSolveCalibration1: Boolean? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeReadyToSolveCalibration")
        requireNotNull(accelerometerAndGyroscopeReadyToSolveCalibration1)
        assertFalse(accelerometerAndGyroscopeReadyToSolveCalibration1)
        val magnetometerReadyToSolveCalibration: Boolean? =
            calibrator.getPrivateProperty("magnetometerReadyToSolveCalibration")
        requireNotNull(magnetometerReadyToSolveCalibration)
        assertTrue(magnetometerReadyToSolveCalibration)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val listener = accelerometerAndGyroscopeCalibrator.readyToSolveCalibrationListener
        requireNotNull(listener)

        listener.onReadyToSolveCalibration(accelerometerAndGyroscopeCalibrator)

        val accelerometerAndGyroscopeReadyToSolveCalibration2: Boolean? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeReadyToSolveCalibration")
        requireNotNull(accelerometerAndGyroscopeReadyToSolveCalibration2)
        assertTrue(accelerometerAndGyroscopeReadyToSolveCalibration2)

        verify(readyToSolveCalibrationListener, only()).onReadyToSolveCalibration(calibrator)
//        verify(exactly = 1) { readyToSolveCalibrationListener.onReadyToSolveCalibration(calibrator) }
    }

    @Test
    fun onReadyToSolveCalibration_whenMagnetometerAndAccelerometerAndGyroscopeReadyToSolveCalibrationAndNoListener_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        calibrator.setPrivateProperty("accelerometerAndGyroscopeReadyToSolveCalibration", true)

        val accelerometerAndGyroscopeReadyToSolveCalibration: Boolean? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeReadyToSolveCalibration")
        requireNotNull(accelerometerAndGyroscopeReadyToSolveCalibration)
        assertTrue(accelerometerAndGyroscopeReadyToSolveCalibration)
        val magnetometerReadyToSolveCalibration1: Boolean? =
            calibrator.getPrivateProperty("magnetometerReadyToSolveCalibration")
        requireNotNull(magnetometerReadyToSolveCalibration1)
        assertFalse(magnetometerReadyToSolveCalibration1)

        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        val listener = magnetometerCalibrator.readyToSolveCalibrationListener
        requireNotNull(listener)

        listener.onReadyToSolveCalibration(magnetometerCalibrator)

        val magnetometerReadyToSolveCalibration2: Boolean? =
            calibrator.getPrivateProperty("magnetometerReadyToSolveCalibration")
        requireNotNull(magnetometerReadyToSolveCalibration2)
        assertTrue(magnetometerReadyToSolveCalibration2)
    }

    @Test
    fun onReadyToSolveCalibration_whenMagnetometerAndAccelerometerAndGyroscopeNotReadyToSolveCalibration_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(
            context,
            readyToSolveCalibrationListener = readyToSolveCalibrationListener
        )

        val accelerometerAndGyroscopeReadyToSolveCalibration: Boolean? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeReadyToSolveCalibration")
        requireNotNull(accelerometerAndGyroscopeReadyToSolveCalibration)
        assertFalse(accelerometerAndGyroscopeReadyToSolveCalibration)
        val magnetometerReadyToSolveCalibration1: Boolean? =
            calibrator.getPrivateProperty("magnetometerReadyToSolveCalibration")
        requireNotNull(magnetometerReadyToSolveCalibration1)
        assertFalse(magnetometerReadyToSolveCalibration1)

        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        val listener = magnetometerCalibrator.readyToSolveCalibrationListener
        requireNotNull(listener)

        listener.onReadyToSolveCalibration(magnetometerCalibrator)

        val magnetometerReadyToSolveCalibration2: Boolean? =
            calibrator.getPrivateProperty("magnetometerReadyToSolveCalibration")
        requireNotNull(magnetometerReadyToSolveCalibration2)
        assertTrue(magnetometerReadyToSolveCalibration2)

        verifyNoInteractions(readyToSolveCalibrationListener)
//        verify { readyToSolveCalibrationListener wasNot Called }
    }

    @Test
    fun onReadyToSolveCalibration_whenMagnetometerAndAccelerometerAndGyroscopeReadyToSolveCalibration_notifies() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(
            context,
            readyToSolveCalibrationListener = readyToSolveCalibrationListener
        )

        calibrator.setPrivateProperty("accelerometerAndGyroscopeReadyToSolveCalibration", true)

        val accelerometerAndGyroscopeReadyToSolveCalibration: Boolean? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeReadyToSolveCalibration")
        requireNotNull(accelerometerAndGyroscopeReadyToSolveCalibration)
        assertTrue(accelerometerAndGyroscopeReadyToSolveCalibration)
        val magnetometerReadyToSolveCalibration1: Boolean? =
            calibrator.getPrivateProperty("magnetometerReadyToSolveCalibration")
        requireNotNull(magnetometerReadyToSolveCalibration1)
        assertFalse(magnetometerReadyToSolveCalibration1)

        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        val listener = magnetometerCalibrator.readyToSolveCalibrationListener
        requireNotNull(listener)

        listener.onReadyToSolveCalibration(magnetometerCalibrator)

        val magnetometerReadyToSolveCalibration2: Boolean? =
            calibrator.getPrivateProperty("magnetometerReadyToSolveCalibration")
        requireNotNull(magnetometerReadyToSolveCalibration2)
        assertTrue(magnetometerReadyToSolveCalibration2)

        verify(readyToSolveCalibrationListener, only()).onReadyToSolveCalibration(calibrator)
//        verify(exactly = 1) { readyToSolveCalibrationListener.onReadyToSolveCalibration(calibrator) }
    }

    @Test
    fun onCalibrationSolvingStarted_whenAccelerometerAndGyroscopeAndMagnetometerNotReadyToSolveCalibration_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(
            context,
            calibrationSolvingStartedListener = calibrationSolvingStartedListener
        )

        val accelerometerAndGyroscopeCalibrationSolvingStarted1: Boolean? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrationSolvingStarted")
        requireNotNull(accelerometerAndGyroscopeCalibrationSolvingStarted1)
        assertFalse(accelerometerAndGyroscopeCalibrationSolvingStarted1)
        val magnetometerReadyToSolveCalibration: Boolean? =
            calibrator.getPrivateProperty("magnetometerReadyToSolveCalibration")
        requireNotNull(magnetometerReadyToSolveCalibration)
        assertFalse(magnetometerReadyToSolveCalibration)
        val magnetometerCalibrationSolvingStarted: Boolean? =
            calibrator.getPrivateProperty("magnetometerCalibrationSolvingStarted")
        requireNotNull(magnetometerCalibrationSolvingStarted)
        assertFalse(magnetometerCalibrationSolvingStarted)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val listener = accelerometerAndGyroscopeCalibrator.calibrationSolvingStartedListener
        requireNotNull(listener)

        listener.onCalibrationSolvingStarted(accelerometerAndGyroscopeCalibrator)

        val accelerometerAndGyroscopeCalibrationSolvingStarted2: Boolean? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrationSolvingStarted")
        requireNotNull(accelerometerAndGyroscopeCalibrationSolvingStarted2)
        assertTrue(accelerometerAndGyroscopeCalibrationSolvingStarted2)

        verifyNoInteractions(calibrationSolvingStartedListener)
//        verify { calibrationSolvingStartedListener wasNot Called }
    }

    @Test
    fun onCalibrationSolvingStarted_whenAccelerometerAndGyroscopeMagnetometerNotReadyToSolveCalibrationAndMagnetometerCalibrationSolvingStarted_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(
            context,
            calibrationSolvingStartedListener = calibrationSolvingStartedListener
        )

        calibrator.setPrivateProperty("magnetometerCalibrationSolvingStarted", true)

        val accelerometerAndGyroscopeCalibrationSolvingStarted1: Boolean? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrationSolvingStarted")
        requireNotNull(accelerometerAndGyroscopeCalibrationSolvingStarted1)
        assertFalse(accelerometerAndGyroscopeCalibrationSolvingStarted1)
        val magnetometerReadyToSolveCalibration: Boolean? =
            calibrator.getPrivateProperty("magnetometerReadyToSolveCalibration")
        requireNotNull(magnetometerReadyToSolveCalibration)
        assertFalse(magnetometerReadyToSolveCalibration)
        val magnetometerCalibrationSolvingStarted: Boolean? =
            calibrator.getPrivateProperty("magnetometerCalibrationSolvingStarted")
        requireNotNull(magnetometerCalibrationSolvingStarted)
        assertTrue(magnetometerCalibrationSolvingStarted)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val listener = accelerometerAndGyroscopeCalibrator.calibrationSolvingStartedListener
        requireNotNull(listener)

        listener.onCalibrationSolvingStarted(accelerometerAndGyroscopeCalibrator)

        val accelerometerAndGyroscopeCalibrationSolvingStarted2: Boolean? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrationSolvingStarted")
        requireNotNull(accelerometerAndGyroscopeCalibrationSolvingStarted2)
        assertTrue(accelerometerAndGyroscopeCalibrationSolvingStarted2)

        verifyNoInteractions(calibrationSolvingStartedListener)
//        verify { calibrationSolvingStartedListener wasNot Called }
    }

    @Test
    fun onCalibrationSolvingStarted_whenAccelerometerAndGyroscopeMagnetometerReadyToSolveCalibrationAndMagnetometerCalibrationSolvingNotStartedAndNoListener_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        calibrator.setPrivateProperty("magnetometerReadyToSolveCalibration", true)

        val accelerometerAndGyroscopeCalibrationSolvingStarted1: Boolean? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrationSolvingStarted")
        requireNotNull(accelerometerAndGyroscopeCalibrationSolvingStarted1)
        assertFalse(accelerometerAndGyroscopeCalibrationSolvingStarted1)
        val magnetometerReadyToSolveCalibration: Boolean? =
            calibrator.getPrivateProperty("magnetometerReadyToSolveCalibration")
        requireNotNull(magnetometerReadyToSolveCalibration)
        assertTrue(magnetometerReadyToSolveCalibration)
        val magnetometerCalibrationSolvingStarted: Boolean? =
            calibrator.getPrivateProperty("magnetometerCalibrationSolvingStarted")
        requireNotNull(magnetometerCalibrationSolvingStarted)
        assertFalse(magnetometerCalibrationSolvingStarted)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val listener = accelerometerAndGyroscopeCalibrator.calibrationSolvingStartedListener
        requireNotNull(listener)

        listener.onCalibrationSolvingStarted(accelerometerAndGyroscopeCalibrator)

        val accelerometerAndGyroscopeCalibrationSolvingStarted2: Boolean? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrationSolvingStarted")
        requireNotNull(accelerometerAndGyroscopeCalibrationSolvingStarted2)
        assertTrue(accelerometerAndGyroscopeCalibrationSolvingStarted2)
    }

    @Test
    fun onCalibrationSolvingStarted_whenAccelerometerAndGyroscopeMagnetometerReadyToSolveCalibrationAndMagnetometerCalibrationSolvingNotStarted_notifies() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(
            context,
            calibrationSolvingStartedListener = calibrationSolvingStartedListener
        )

        calibrator.setPrivateProperty("magnetometerReadyToSolveCalibration", true)

        val accelerometerAndGyroscopeCalibrationSolvingStarted1: Boolean? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrationSolvingStarted")
        requireNotNull(accelerometerAndGyroscopeCalibrationSolvingStarted1)
        assertFalse(accelerometerAndGyroscopeCalibrationSolvingStarted1)
        val magnetometerReadyToSolveCalibration: Boolean? =
            calibrator.getPrivateProperty("magnetometerReadyToSolveCalibration")
        requireNotNull(magnetometerReadyToSolveCalibration)
        assertTrue(magnetometerReadyToSolveCalibration)
        val magnetometerCalibrationSolvingStarted: Boolean? =
            calibrator.getPrivateProperty("magnetometerCalibrationSolvingStarted")
        requireNotNull(magnetometerCalibrationSolvingStarted)
        assertFalse(magnetometerCalibrationSolvingStarted)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val listener = accelerometerAndGyroscopeCalibrator.calibrationSolvingStartedListener
        requireNotNull(listener)

        listener.onCalibrationSolvingStarted(accelerometerAndGyroscopeCalibrator)

        val accelerometerAndGyroscopeCalibrationSolvingStarted2: Boolean? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrationSolvingStarted")
        requireNotNull(accelerometerAndGyroscopeCalibrationSolvingStarted2)
        assertTrue(accelerometerAndGyroscopeCalibrationSolvingStarted2)

        verify(calibrationSolvingStartedListener, only()).onCalibrationSolvingStarted(
            calibrator
        )
/*        verify(exactly = 1) {
            calibrationSolvingStartedListener.onCalibrationSolvingStarted(
                calibrator
            )
        }*/
    }

    @Test
    fun onCalibrationSolvingStarted_whenAccelerometerAndGyroscopeMagnetometerReadyToSolveCalibrationAndMagnetometerCalibrationSolvingStarted_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(
            context,
            calibrationSolvingStartedListener = calibrationSolvingStartedListener
        )

        calibrator.setPrivateProperty("magnetometerReadyToSolveCalibration", true)
        calibrator.setPrivateProperty("magnetometerCalibrationSolvingStarted", true)

        val accelerometerAndGyroscopeCalibrationSolvingStarted1: Boolean? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrationSolvingStarted")
        requireNotNull(accelerometerAndGyroscopeCalibrationSolvingStarted1)
        assertFalse(accelerometerAndGyroscopeCalibrationSolvingStarted1)
        val magnetometerReadyToSolveCalibration: Boolean? =
            calibrator.getPrivateProperty("magnetometerReadyToSolveCalibration")
        requireNotNull(magnetometerReadyToSolveCalibration)
        assertTrue(magnetometerReadyToSolveCalibration)
        val magnetometerCalibrationSolvingStarted: Boolean? =
            calibrator.getPrivateProperty("magnetometerCalibrationSolvingStarted")
        requireNotNull(magnetometerCalibrationSolvingStarted)
        assertTrue(magnetometerCalibrationSolvingStarted)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val listener = accelerometerAndGyroscopeCalibrator.calibrationSolvingStartedListener
        requireNotNull(listener)

        listener.onCalibrationSolvingStarted(accelerometerAndGyroscopeCalibrator)

        val accelerometerAndGyroscopeCalibrationSolvingStarted2: Boolean? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrationSolvingStarted")
        requireNotNull(accelerometerAndGyroscopeCalibrationSolvingStarted2)
        assertTrue(accelerometerAndGyroscopeCalibrationSolvingStarted2)

        verifyNoInteractions(calibrationSolvingStartedListener)
//        verify { calibrationSolvingStartedListener wasNot Called }
    }

    @Test
    fun onCalibrationSolvingStarted_whenMagnetometerAndAccelerometerAndGyroscopeNotReadyToSolveCalibration_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(
            context,
            calibrationSolvingStartedListener = calibrationSolvingStartedListener
        )

        val magnetometerCalibrationSolvingStarted1: Boolean? =
            calibrator.getPrivateProperty("magnetometerCalibrationSolvingStarted")
        requireNotNull(magnetometerCalibrationSolvingStarted1)
        assertFalse(magnetometerCalibrationSolvingStarted1)
        val accelerometerAndGyroscopeReadyToSolveCalibration: Boolean? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeReadyToSolveCalibration")
        requireNotNull(accelerometerAndGyroscopeReadyToSolveCalibration)
        assertFalse(accelerometerAndGyroscopeReadyToSolveCalibration)
        val accelerometerAndGyroscopeCalibrationSolvingStarted: Boolean? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrationSolvingStarted")
        requireNotNull(accelerometerAndGyroscopeCalibrationSolvingStarted)
        assertFalse(accelerometerAndGyroscopeCalibrationSolvingStarted)

        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        val listener = magnetometerCalibrator.calibrationSolvingStartedListener
        requireNotNull(listener)

        listener.onCalibrationSolvingStarted(magnetometerCalibrator)

        val magnetometerCalibrationSolvingStarted2: Boolean? =
            calibrator.getPrivateProperty("magnetometerCalibrationSolvingStarted")
        requireNotNull(magnetometerCalibrationSolvingStarted2)
        assertTrue(magnetometerCalibrationSolvingStarted2)

        verifyNoInteractions(calibrationSolvingStartedListener)
//        verify { calibrationSolvingStartedListener wasNot Called }
    }

    @Test
    fun onCalibrationSolvingStarted_whenMagnetometerAndAccelerometerAndGyroscopeNotReadyToSolveCalibrationAndAccelerometerAndGyroscopeCalibrationSolvingStarted_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(
            context,
            calibrationSolvingStartedListener = calibrationSolvingStartedListener
        )

        calibrator.setPrivateProperty("accelerometerAndGyroscopeCalibrationSolvingStarted", true)

        val magnetometerCalibrationSolvingStarted1: Boolean? =
            calibrator.getPrivateProperty("magnetometerCalibrationSolvingStarted")
        requireNotNull(magnetometerCalibrationSolvingStarted1)
        assertFalse(magnetometerCalibrationSolvingStarted1)
        val accelerometerAndGyroscopeReadyToSolveCalibration: Boolean? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeReadyToSolveCalibration")
        requireNotNull(accelerometerAndGyroscopeReadyToSolveCalibration)
        assertFalse(accelerometerAndGyroscopeReadyToSolveCalibration)
        val accelerometerAndGyroscopeCalibrationSolvingStarted: Boolean? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrationSolvingStarted")
        requireNotNull(accelerometerAndGyroscopeCalibrationSolvingStarted)
        assertTrue(accelerometerAndGyroscopeCalibrationSolvingStarted)

        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        val listener = magnetometerCalibrator.calibrationSolvingStartedListener
        requireNotNull(listener)

        listener.onCalibrationSolvingStarted(magnetometerCalibrator)

        val magnetometerCalibrationSolvingStarted2: Boolean? =
            calibrator.getPrivateProperty("magnetometerCalibrationSolvingStarted")
        requireNotNull(magnetometerCalibrationSolvingStarted2)
        assertTrue(magnetometerCalibrationSolvingStarted2)

        verifyNoInteractions(calibrationSolvingStartedListener)
//        verify { calibrationSolvingStartedListener wasNot Called }
    }

    @Test
    fun onCalibrationSolvingStarted_whenMagnetometerAndAccelerometerAndGyroscopeReadyToSolveCalibrationAndAccelerometerAndGyroscopeCalibrationSolvingNotStartedAndNoListener_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        calibrator.setPrivateProperty("accelerometerAndGyroscopeReadyToSolveCalibration", true)

        val magnetometerCalibrationSolvingStarted1: Boolean? =
            calibrator.getPrivateProperty("magnetometerCalibrationSolvingStarted")
        requireNotNull(magnetometerCalibrationSolvingStarted1)
        assertFalse(magnetometerCalibrationSolvingStarted1)
        val accelerometerAndGyroscopeReadyToSolveCalibration: Boolean? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeReadyToSolveCalibration")
        requireNotNull(accelerometerAndGyroscopeReadyToSolveCalibration)
        assertTrue(accelerometerAndGyroscopeReadyToSolveCalibration)
        val accelerometerAndGyroscopeCalibrationSolvingStarted: Boolean? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrationSolvingStarted")
        requireNotNull(accelerometerAndGyroscopeCalibrationSolvingStarted)
        assertFalse(accelerometerAndGyroscopeCalibrationSolvingStarted)

        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        val listener = magnetometerCalibrator.calibrationSolvingStartedListener
        requireNotNull(listener)

        listener.onCalibrationSolvingStarted(magnetometerCalibrator)

        val magnetometerCalibrationSolvingStarted2: Boolean? =
            calibrator.getPrivateProperty("magnetometerCalibrationSolvingStarted")
        requireNotNull(magnetometerCalibrationSolvingStarted2)
        assertTrue(magnetometerCalibrationSolvingStarted2)
    }

    @Test
    fun onCalibrationSolvingStarted_whenMagnetometerAndAccelerometerAndGyroscopeReadyToSolveCalibrationAndAccelerometerAndGyroscopeCalibrationSolvingNotStarted_notifies() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(
            context,
            calibrationSolvingStartedListener = calibrationSolvingStartedListener
        )

        calibrator.setPrivateProperty("accelerometerAndGyroscopeReadyToSolveCalibration", true)

        val magnetometerCalibrationSolvingStarted1: Boolean? =
            calibrator.getPrivateProperty("magnetometerCalibrationSolvingStarted")
        requireNotNull(magnetometerCalibrationSolvingStarted1)
        assertFalse(magnetometerCalibrationSolvingStarted1)
        val accelerometerAndGyroscopeReadyToSolveCalibration: Boolean? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeReadyToSolveCalibration")
        requireNotNull(accelerometerAndGyroscopeReadyToSolveCalibration)
        assertTrue(accelerometerAndGyroscopeReadyToSolveCalibration)
        val accelerometerAndGyroscopeCalibrationSolvingStarted: Boolean? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrationSolvingStarted")
        requireNotNull(accelerometerAndGyroscopeCalibrationSolvingStarted)
        assertFalse(accelerometerAndGyroscopeCalibrationSolvingStarted)

        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        val listener = magnetometerCalibrator.calibrationSolvingStartedListener
        requireNotNull(listener)

        listener.onCalibrationSolvingStarted(magnetometerCalibrator)

        val magnetometerCalibrationSolvingStarted2: Boolean? =
            calibrator.getPrivateProperty("magnetometerCalibrationSolvingStarted")
        requireNotNull(magnetometerCalibrationSolvingStarted2)
        assertTrue(magnetometerCalibrationSolvingStarted2)

        verify(calibrationSolvingStartedListener, only()).onCalibrationSolvingStarted(
            calibrator
        )
/*        verify(exactly = 1) {
            calibrationSolvingStartedListener.onCalibrationSolvingStarted(
                calibrator
            )
        }*/
    }

    @Test
    fun onCalibrationSolvingStarted_whenMagnetometerAndAccelerometerAndGyroscopeReadyToSolveCalibrationAndAccelerometerAndGyroscopeCalibrationSolvingStarted_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(
            context,
            calibrationSolvingStartedListener = calibrationSolvingStartedListener
        )

        calibrator.setPrivateProperty("accelerometerAndGyroscopeReadyToSolveCalibration", true)
        calibrator.setPrivateProperty("accelerometerAndGyroscopeCalibrationSolvingStarted", true)

        val magnetometerCalibrationSolvingStarted1: Boolean? =
            calibrator.getPrivateProperty("magnetometerCalibrationSolvingStarted")
        requireNotNull(magnetometerCalibrationSolvingStarted1)
        assertFalse(magnetometerCalibrationSolvingStarted1)
        val accelerometerAndGyroscopeReadyToSolveCalibration: Boolean? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeReadyToSolveCalibration")
        requireNotNull(accelerometerAndGyroscopeReadyToSolveCalibration)
        assertTrue(accelerometerAndGyroscopeReadyToSolveCalibration)
        val accelerometerAndGyroscopeCalibrationSolvingStarted: Boolean? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrationSolvingStarted")
        requireNotNull(accelerometerAndGyroscopeCalibrationSolvingStarted)
        assertTrue(accelerometerAndGyroscopeCalibrationSolvingStarted)

        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        val listener = magnetometerCalibrator.calibrationSolvingStartedListener
        requireNotNull(listener)

        listener.onCalibrationSolvingStarted(magnetometerCalibrator)

        val magnetometerCalibrationSolvingStarted2: Boolean? =
            calibrator.getPrivateProperty("magnetometerCalibrationSolvingStarted")
        requireNotNull(magnetometerCalibrationSolvingStarted2)
        assertTrue(magnetometerCalibrationSolvingStarted2)

        verifyNoInteractions(calibrationSolvingStartedListener)
//        verify { calibrationSolvingStartedListener wasNot Called }
    }

    @Test
    fun onCalibrationCompleted_whenAccelerometerAndGyroscopeAndMagnetometerCalibrationNotCompleted_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(
            context,
            calibrationCompletedListener = calibrationCompletedListener
        )

        val accelerometerAndGyroscopeCalibrationCompleted1: Boolean? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrationCompleted")
        requireNotNull(accelerometerAndGyroscopeCalibrationCompleted1)
        assertFalse(accelerometerAndGyroscopeCalibrationCompleted1)
        val magnetometerCalibrationCompleted: Boolean? =
            calibrator.getPrivateProperty("magnetometerCalibrationCompleted")
        requireNotNull(magnetometerCalibrationCompleted)
        assertFalse(magnetometerCalibrationCompleted)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val listener = accelerometerAndGyroscopeCalibrator.calibrationCompletedListener
        requireNotNull(listener)

        listener.onCalibrationCompleted(accelerometerAndGyroscopeCalibrator)

        val accelerometerAndGyroscopeCalibrationCompleted2: Boolean? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrationCompleted")
        requireNotNull(accelerometerAndGyroscopeCalibrationCompleted2)
        assertTrue(accelerometerAndGyroscopeCalibrationCompleted2)

        verifyNoInteractions(calibrationCompletedListener)
//        verify { calibrationCompletedListener wasNot Called }
    }

    @Test
    fun onCalibrationCompleted_whenAccelerometerAndGyroscopeAndMagnetometerCalibrationCompletedAndNoListener_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        calibrator.setPrivateProperty("magnetometerCalibrationCompleted", true)

        val accelerometerAndGyroscopeCalibrationCompleted1: Boolean? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrationCompleted")
        requireNotNull(accelerometerAndGyroscopeCalibrationCompleted1)
        assertFalse(accelerometerAndGyroscopeCalibrationCompleted1)
        val magnetometerCalibrationCompleted: Boolean? =
            calibrator.getPrivateProperty("magnetometerCalibrationCompleted")
        requireNotNull(magnetometerCalibrationCompleted)
        assertTrue(magnetometerCalibrationCompleted)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val listener = accelerometerAndGyroscopeCalibrator.calibrationCompletedListener
        requireNotNull(listener)

        listener.onCalibrationCompleted(accelerometerAndGyroscopeCalibrator)

        val accelerometerAndGyroscopeCalibrationCompleted2: Boolean? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrationCompleted")
        requireNotNull(accelerometerAndGyroscopeCalibrationCompleted2)
        assertTrue(accelerometerAndGyroscopeCalibrationCompleted2)
    }

    @Test
    fun onCalibrationCompleted_whenAccelerometerAndGyroscopeAndMagnetometerCalibrationCompleted_notifies() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(
            context,
            calibrationCompletedListener = calibrationCompletedListener
        )

        calibrator.setPrivateProperty("magnetometerCalibrationCompleted", true)

        val accelerometerAndGyroscopeCalibrationCompleted1: Boolean? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrationCompleted")
        requireNotNull(accelerometerAndGyroscopeCalibrationCompleted1)
        assertFalse(accelerometerAndGyroscopeCalibrationCompleted1)
        val magnetometerCalibrationCompleted: Boolean? =
            calibrator.getPrivateProperty("magnetometerCalibrationCompleted")
        requireNotNull(magnetometerCalibrationCompleted)
        assertTrue(magnetometerCalibrationCompleted)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val listener = accelerometerAndGyroscopeCalibrator.calibrationCompletedListener
        requireNotNull(listener)

        listener.onCalibrationCompleted(accelerometerAndGyroscopeCalibrator)

        val accelerometerAndGyroscopeCalibrationCompleted2: Boolean? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrationCompleted")
        requireNotNull(accelerometerAndGyroscopeCalibrationCompleted2)
        assertTrue(accelerometerAndGyroscopeCalibrationCompleted2)

        verify(calibrationCompletedListener, only()).onCalibrationCompleted(calibrator)
//        verify(exactly = 1) { calibrationCompletedListener.onCalibrationCompleted(calibrator) }
    }

    @Test
    fun onCalibrationCompleted_whenMagnetometerAndAccelerometerAndGyroscopeCalibrationNotCompleted_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(
            context,
            calibrationCompletedListener = calibrationCompletedListener
        )

        val magnetometerCalibrationCompleted1: Boolean? =
            calibrator.getPrivateProperty("magnetometerCalibrationCompleted")
        requireNotNull(magnetometerCalibrationCompleted1)
        assertFalse(magnetometerCalibrationCompleted1)
        val accelerometerAndGyroscopeCalibrationCompleted: Boolean? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrationCompleted")
        requireNotNull(accelerometerAndGyroscopeCalibrationCompleted)
        assertFalse(accelerometerAndGyroscopeCalibrationCompleted)

        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        val listener = magnetometerCalibrator.calibrationCompletedListener
        requireNotNull(listener)

        listener.onCalibrationCompleted(magnetometerCalibrator)

        val magnetometerCalibrationCompleted2: Boolean? =
            calibrator.getPrivateProperty("magnetometerCalibrationCompleted")
        requireNotNull(magnetometerCalibrationCompleted2)
        assertTrue(magnetometerCalibrationCompleted2)

        verifyNoInteractions(calibrationCompletedListener)
//        verify { calibrationCompletedListener wasNot Called }
    }

    @Test
    fun onCalibrationCompleted_whenMagnetometerAndAccelerometerAndGyroscopeCalibrationCompletedAndNoListener_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        calibrator.setPrivateProperty("accelerometerAndGyroscopeCalibrationCompleted", true)

        val magnetometerCalibrationCompleted1: Boolean? =
            calibrator.getPrivateProperty("magnetometerCalibrationCompleted")
        requireNotNull(magnetometerCalibrationCompleted1)
        assertFalse(magnetometerCalibrationCompleted1)
        val accelerometerAndGyroscopeCalibrationCompleted: Boolean? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrationCompleted")
        requireNotNull(accelerometerAndGyroscopeCalibrationCompleted)
        assertTrue(accelerometerAndGyroscopeCalibrationCompleted)

        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        val listener = magnetometerCalibrator.calibrationCompletedListener
        requireNotNull(listener)

        listener.onCalibrationCompleted(magnetometerCalibrator)

        val magnetometerCalibrationCompleted2: Boolean? =
            calibrator.getPrivateProperty("magnetometerCalibrationCompleted")
        requireNotNull(magnetometerCalibrationCompleted2)
        assertTrue(magnetometerCalibrationCompleted2)
    }

    @Test
    fun onCalibrationCompleted_whenMagnetometerAndAccelerometerAndGyroscopeCalibrationCompleted_notifies() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(
            context,
            calibrationCompletedListener = calibrationCompletedListener
        )

        calibrator.setPrivateProperty("accelerometerAndGyroscopeCalibrationCompleted", true)

        val magnetometerCalibrationCompleted1: Boolean? =
            calibrator.getPrivateProperty("magnetometerCalibrationCompleted")
        requireNotNull(magnetometerCalibrationCompleted1)
        assertFalse(magnetometerCalibrationCompleted1)
        val accelerometerAndGyroscopeCalibrationCompleted: Boolean? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrationCompleted")
        requireNotNull(accelerometerAndGyroscopeCalibrationCompleted)
        assertTrue(accelerometerAndGyroscopeCalibrationCompleted)

        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        val listener = magnetometerCalibrator.calibrationCompletedListener
        requireNotNull(listener)

        listener.onCalibrationCompleted(magnetometerCalibrator)

        val magnetometerCalibrationCompleted2: Boolean? =
            calibrator.getPrivateProperty("magnetometerCalibrationCompleted")
        requireNotNull(magnetometerCalibrationCompleted2)
        assertTrue(magnetometerCalibrationCompleted2)

        verify(calibrationCompletedListener, only()).onCalibrationCompleted(calibrator)
//        verify(exactly = 1) { calibrationCompletedListener.onCalibrationCompleted(calibrator) }
    }

    @Test
    fun onStopped_whenAccelerometerAndGyroscopeAndMagnetometerNotStopped_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(
            context,
            stoppedListener = stoppedListener
        )

        val accelerometerAndGyroscopeStopped1: Boolean? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeStopped")
        requireNotNull(accelerometerAndGyroscopeStopped1)
        assertFalse(accelerometerAndGyroscopeStopped1)
        val magnetometerStopped: Boolean? =
            calibrator.getPrivateProperty("magnetometerStopped")
        requireNotNull(magnetometerStopped)
        assertFalse(magnetometerStopped)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val listener = accelerometerAndGyroscopeCalibrator.stoppedListener
        requireNotNull(listener)

        listener.onStopped(accelerometerAndGyroscopeCalibrator)

        val accelerometerAndGyroscopeStopped2: Boolean? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeStopped")
        requireNotNull(accelerometerAndGyroscopeStopped2)
        assertTrue(accelerometerAndGyroscopeStopped2)

        verifyNoInteractions(stoppedListener)
//        verify { stoppedListener wasNot Called }
    }

    @Test
    fun onStopped_whenAccelerometerAndGyroscopeAndMagnetometerStoppedAndNoListener_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        calibrator.setPrivateProperty("magnetometerStopped", true)

        val accelerometerAndGyroscopeStopped1: Boolean? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeStopped")
        requireNotNull(accelerometerAndGyroscopeStopped1)
        assertFalse(accelerometerAndGyroscopeStopped1)
        val magnetometerStopped: Boolean? =
            calibrator.getPrivateProperty("magnetometerStopped")
        requireNotNull(magnetometerStopped)
        assertTrue(magnetometerStopped)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val listener = accelerometerAndGyroscopeCalibrator.stoppedListener
        requireNotNull(listener)

        listener.onStopped(accelerometerAndGyroscopeCalibrator)

        val accelerometerAndGyroscopeStopped2: Boolean? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeStopped")
        requireNotNull(accelerometerAndGyroscopeStopped2)
        assertTrue(accelerometerAndGyroscopeStopped2)
    }

    @Test
    fun onStopped_whenAccelerometerAndGyroscopeAndMagnetometerStopped_notifies() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(
            context,
            stoppedListener = stoppedListener
        )

        calibrator.setPrivateProperty("magnetometerStopped", true)

        val accelerometerAndGyroscopeStopped1: Boolean? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeStopped")
        requireNotNull(accelerometerAndGyroscopeStopped1)
        assertFalse(accelerometerAndGyroscopeStopped1)
        val magnetometerStopped: Boolean? =
            calibrator.getPrivateProperty("magnetometerStopped")
        requireNotNull(magnetometerStopped)
        assertTrue(magnetometerStopped)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val listener = accelerometerAndGyroscopeCalibrator.stoppedListener
        requireNotNull(listener)

        listener.onStopped(accelerometerAndGyroscopeCalibrator)

        val accelerometerAndGyroscopeStopped2: Boolean? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeStopped")
        requireNotNull(accelerometerAndGyroscopeStopped2)
        assertTrue(accelerometerAndGyroscopeStopped2)

        verify(stoppedListener, only()).onStopped(calibrator)
//        verify(exactly = 1) { stoppedListener.onStopped(calibrator) }
    }

    @Test
    fun onStopped_whenMagnetometerAndAccelerometerAndGyroscopeNotStopped_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(
            context,
            stoppedListener = stoppedListener
        )

        val magnetometerStopped1: Boolean? =
            calibrator.getPrivateProperty("magnetometerStopped")
        requireNotNull(magnetometerStopped1)
        assertFalse(magnetometerStopped1)
        val accelerometerAndGyroscopeStopped: Boolean? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeStopped")
        requireNotNull(accelerometerAndGyroscopeStopped)
        assertFalse(accelerometerAndGyroscopeStopped)

        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        val listener = magnetometerCalibrator.stoppedListener
        requireNotNull(listener)

        listener.onStopped(magnetometerCalibrator)

        val magnetometerStopped2: Boolean? =
            calibrator.getPrivateProperty("magnetometerStopped")
        requireNotNull(magnetometerStopped2)
        assertTrue(magnetometerStopped2)

        verifyNoInteractions(stoppedListener)
//        verify { stoppedListener wasNot Called }
    }

    @Test
    fun onStopped_whenMagnetometerAndAccelerometerAndGyroscopeNotStoppedAndNoListener_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        calibrator.setPrivateProperty("accelerometerAndGyroscopeStopped", true)

        val magnetometerStopped1: Boolean? =
            calibrator.getPrivateProperty("magnetometerStopped")
        requireNotNull(magnetometerStopped1)
        assertFalse(magnetometerStopped1)
        val accelerometerAndGyroscopeStopped: Boolean? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeStopped")
        requireNotNull(accelerometerAndGyroscopeStopped)
        assertTrue(accelerometerAndGyroscopeStopped)

        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        val listener = magnetometerCalibrator.stoppedListener
        requireNotNull(listener)

        listener.onStopped(magnetometerCalibrator)

        val magnetometerStopped2: Boolean? =
            calibrator.getPrivateProperty("magnetometerStopped")
        requireNotNull(magnetometerStopped2)
        assertTrue(magnetometerStopped2)
    }

    @Test
    fun onStopped_whenMagnetometerAndAccelerometerAndGyroscopeNotStopped_notifies() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(
            context,
            stoppedListener = stoppedListener
        )

        calibrator.setPrivateProperty("accelerometerAndGyroscopeStopped", true)

        val magnetometerStopped1: Boolean? =
            calibrator.getPrivateProperty("magnetometerStopped")
        requireNotNull(magnetometerStopped1)
        assertFalse(magnetometerStopped1)
        val accelerometerAndGyroscopeStopped: Boolean? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeStopped")
        requireNotNull(accelerometerAndGyroscopeStopped)
        assertTrue(accelerometerAndGyroscopeStopped)

        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        val listener = magnetometerCalibrator.stoppedListener
        requireNotNull(listener)

        listener.onStopped(magnetometerCalibrator)

        val magnetometerStopped2: Boolean? =
            calibrator.getPrivateProperty("magnetometerStopped")
        requireNotNull(magnetometerStopped2)
        assertTrue(magnetometerStopped2)

        verify(stoppedListener, only()).onStopped(calibrator)
//        verify(exactly = 1) { stoppedListener.onStopped(calibrator) }
    }

    @Test
    fun onUnreliableGravityNormEstimation_whenNoListener_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val listener = accelerometerAndGyroscopeCalibrator.unreliableGravityNormEstimationListener
        requireNotNull(listener)

        listener.onUnreliableGravityEstimation(accelerometerAndGyroscopeCalibrator)
    }

    @Test
    fun onUnreliableGravityNormEstimation_whenListener_notifies() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(
            context,
            unreliableGravityNormEstimationListener = unreliableGravityNormEstimationListener
        )

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val listener = accelerometerAndGyroscopeCalibrator.unreliableGravityNormEstimationListener
        requireNotNull(listener)

        listener.onUnreliableGravityEstimation(accelerometerAndGyroscopeCalibrator)

        verify(unreliableGravityNormEstimationListener, only()).onUnreliableGravityEstimation(
            calibrator
        )
/*        verify(exactly = 1) {
            unreliableGravityNormEstimationListener.onUnreliableGravityEstimation(
                calibrator
            )
        }*/
    }

    @Test
    fun onInitialAccelerometerBiasAvailable_whenNoListener_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val listener = accelerometerAndGyroscopeCalibrator.initialAccelerometerBiasAvailableListener
        requireNotNull(listener)


        listener.onInitialBiasAvailable(accelerometerAndGyroscopeCalibrator, 1.0, 2.0, 3.0)
    }

    @Test
    fun onInitialAccelerometerBiasAvailable_whenListener_notifies() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(
            context,
            initialAccelerometerBiasAvailableListener = initialAccelerometerBiasAvailableListener
        )

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val listener = accelerometerAndGyroscopeCalibrator.initialAccelerometerBiasAvailableListener
        requireNotNull(listener)


        listener.onInitialBiasAvailable(accelerometerAndGyroscopeCalibrator, 1.0, 2.0, 3.0)

        verify(initialAccelerometerBiasAvailableListener, only()).onInitialBiasAvailable(
            calibrator,
            1.0,
            2.0,
            3.0
        )
/*        verify(exactly = 1) {
            initialAccelerometerBiasAvailableListener.onInitialBiasAvailable(
                calibrator,
                1.0,
                2.0,
                3.0
            )
        }*/
    }

    @Test
    fun onInitialGyroscopeBiasAvailable_whenNoListener_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val listener = accelerometerAndGyroscopeCalibrator.initialGyroscopeBiasAvailableListener
        requireNotNull(listener)

        listener.onInitialBiasAvailable(accelerometerAndGyroscopeCalibrator, 1.0, 2.0, 3.0)
    }

    @Test
    fun onInitialGyroscopeBiasAvailable_whenListener_notifies() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(
            context,
            initialGyroscopeBiasAvailableListener = initialGyroscopeBiasAvailableListener
        )

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val listener = accelerometerAndGyroscopeCalibrator.initialGyroscopeBiasAvailableListener
        requireNotNull(listener)

        listener.onInitialBiasAvailable(accelerometerAndGyroscopeCalibrator, 1.0, 2.0, 3.0)

        verify(initialGyroscopeBiasAvailableListener, only()).onInitialBiasAvailable(
            calibrator,
            1.0,
            2.0,
            3.0
        )
/*        verify(exactly = 1) {
            initialGyroscopeBiasAvailableListener.onInitialBiasAvailable(
                calibrator,
                1.0,
                2.0,
                3.0
            )
        }*/
    }

    @Test
    fun onAccuracyChanged_whenNoListener_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val listener = accelerometerAndGyroscopeCalibrator.accuracyChangedListener
        requireNotNull(listener)

        listener.onAccuracyChanged(SensorAccuracy.LOW)
    }

    @Test
    fun onAccuracyChanged_whenListener_notifies() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(
            context,
            accuracyChangedListener = accuracyChangedListener
        )

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val listener = accelerometerAndGyroscopeCalibrator.accuracyChangedListener
        requireNotNull(listener)

        listener.onAccuracyChanged(SensorAccuracy.LOW)

        verify(accuracyChangedListener, only()).onAccuracyChanged(SensorAccuracy.LOW)
//        verify(exactly = 1) { accuracyChangedListener.onAccuracyChanged(SensorAccuracy.LOW) }
    }

    @Test
    fun onInitialHardIronAvailable_whenNoListener_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        val listener = magnetometerCalibrator.initialHardIronAvailableListener
        requireNotNull(listener)

        listener.onInitialHardIronAvailable(magnetometerCalibrator, 1.0, 2.0, 3.0)
    }

    @Test
    fun onInitialHardIronAvailable_whenListener_notifies() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(
            context,
            initialMagnetometerHardIronAvailableListener = initialMagnetometerHardIronAvailableListener
        )

        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        val listener = magnetometerCalibrator.initialHardIronAvailableListener
        requireNotNull(listener)

        listener.onInitialHardIronAvailable(magnetometerCalibrator, 1.0, 2.0, 3.0)

        verify(initialMagnetometerHardIronAvailableListener, only()).onInitialHardIronAvailable(
            calibrator,
            1.0,
            2.0,
            3.0
        )
/*        verify(exactly = 1) {
            initialMagnetometerHardIronAvailableListener.onInitialHardIronAvailable(
                calibrator,
                1.0,
                2.0,
                3.0
            )
        }*/
    }

    @Test
    fun onGeneratedMagnetometerMeasurement_whenNoListener_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        val listener = magnetometerCalibrator.newCalibrationMeasurementAvailableListener
        requireNotNull(listener)

        val measurement = StandardDeviationBodyMagneticFluxDensity()
        listener.onNewCalibrationMeasurementAvailable(magnetometerCalibrator, measurement, 3, 40)
    }

    @Test
    fun onGeneratedMagnetometerMeasurement_whenListener_notifies() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(
            context,
            generatedMagnetometerMeasurementListener = generatedMagnetometerMeasurementListener
        )

        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        val listener = magnetometerCalibrator.newCalibrationMeasurementAvailableListener
        requireNotNull(listener)

        val measurement = StandardDeviationBodyMagneticFluxDensity()
        listener.onNewCalibrationMeasurementAvailable(magnetometerCalibrator, measurement, 3, 40)

        verify(generatedMagnetometerMeasurementListener, only()).onGeneratedMagnetometerMeasurement(
            calibrator,
            measurement,
            3,
            40
        )
/*        verify(exactly = 1) {
            generatedMagnetometerMeasurementListener.onGeneratedMagnetometerMeasurement(
                calibrator,
                measurement,
                3,
                40
            )
        }*/
    }

    @Test
    fun magnetometerBaseNoiseLevel_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.magnetometerBaseNoiseLevel)

        // mock magnetometer calibrator
        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        val magnetometerCalibratorSpy = spy(magnetometerCalibrator)
//        val magnetometerCalibratorSpy = spyk(magnetometerCalibrator)
        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        whenever(magnetometerCalibratorSpy.baseNoiseLevel).thenReturn(value)
//        every { magnetometerCalibratorSpy.baseNoiseLevel }.returns(value)
        calibrator.setPrivateProperty("magnetometerCalibrator", magnetometerCalibratorSpy)

        // check
        assertEquals(value, calibrator.magnetometerBaseNoiseLevel)

        verify(magnetometerCalibratorSpy, times(2)).baseNoiseLevel
//        verify(exactly = 1) { magnetometerCalibratorSpy.baseNoiseLevel }
    }

    @Test
    fun magnetometerBaseNoiseLevelAsMeasurement_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.magnetometerBaseNoiseLevelAsMeasurement)

        // mock magnetometer calibrator
        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
//        val magnetometerCalibratorSpy = spyk(magnetometerCalibrator)
        val magnetometerCalibratorSpy = spy(magnetometerCalibrator)
        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        val measurement = MagneticFluxDensity(value, MagneticFluxDensityUnit.TESLA)
        whenever(magnetometerCalibratorSpy.baseNoiseLevelAsMeasurement).thenReturn(measurement)
//        every { magnetometerCalibratorSpy.baseNoiseLevelAsMeasurement }.returns(measurement)
        calibrator.setPrivateProperty("magnetometerCalibrator", magnetometerCalibratorSpy)

        // check
        assertSame(measurement, calibrator.magnetometerBaseNoiseLevelAsMeasurement)

        verify(magnetometerCalibratorSpy, times(2)).baseNoiseLevelAsMeasurement
//        verify(exactly = 1) { magnetometerCalibratorSpy.baseNoiseLevelAsMeasurement }
    }

    @Test
    fun getMagnetometerBaseNoiseLevelAsMeasurement_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        val result = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(calibrator.getMagnetometerBaseNoiseLevelAsMeasurement(result))

        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        val magnetometerCalibratorSpy = spy(magnetometerCalibrator)
//        val magnetometerCalibratorSpy = spyk(magnetometerCalibrator)
        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        doAnswer { invocation ->
            val b = invocation.getArgument<MagneticFluxDensity>(0)
            b.value = value
            b.unit = MagneticFluxDensityUnit.TESLA
            return@doAnswer true
        }.whenever(magnetometerCalibratorSpy).getBaseNoiseLevelAsMeasurement(any())
/*        every { magnetometerCalibratorSpy.getBaseNoiseLevelAsMeasurement(any()) }.answers { answer ->
            val b = answer.invocation.args[0] as MagneticFluxDensity
            b.value = value
            b.unit = MagneticFluxDensityUnit.TESLA
            return@answers true
        }*/
        calibrator.setPrivateProperty("magnetometerCalibrator", magnetometerCalibratorSpy)

        // check
        assertTrue(calibrator.getMagnetometerBaseNoiseLevelAsMeasurement(result))
        assertEquals(value, result.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, result.unit)

        verify(magnetometerCalibratorSpy, only()).getBaseNoiseLevelAsMeasurement(result)
//        verify(exactly = 1) { magnetometerCalibratorSpy.getBaseNoiseLevelAsMeasurement(result) }
    }

    @Test
    fun gyroscopeBaseNoiseLevel_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.gyroscopeBaseNoiseLevel)

        // mock internal calibrator
        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val accelerometerAndGyroscopeCalibratorSpy = spy(accelerometerAndGyroscopeCalibrator)
//        val accelerometerAndGyroscopeCalibratorSpy = spyk(accelerometerAndGyroscopeCalibrator)
        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        whenever(accelerometerAndGyroscopeCalibratorSpy.gyroscopeBaseNoiseLevel).thenReturn(value)
//        every { accelerometerAndGyroscopeCalibratorSpy.gyroscopeBaseNoiseLevel }.returns(value)
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibratorSpy
        )

        // check
        assertEquals(value, calibrator.gyroscopeBaseNoiseLevel)

        verify(accelerometerAndGyroscopeCalibratorSpy, times(2)).gyroscopeBaseNoiseLevel
//        verify(exactly = 1) { accelerometerAndGyroscopeCalibratorSpy.gyroscopeBaseNoiseLevel }
    }

    @Test
    fun gyroscopeBaseNoiseLevelAsMeasurement_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.gyroscopeBaseNoiseLevelAsMeasurement)

        // mock internal calibrator
        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val accelerometerAndGyroscopeCalibratorSpy = spy(accelerometerAndGyroscopeCalibrator)
//        val accelerometerAndGyroscopeCalibratorSpy = spyk(accelerometerAndGyroscopeCalibrator)
        val measurement = AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND)
        whenever(accelerometerAndGyroscopeCalibratorSpy.gyroscopeBaseNoiseLevelAsMeasurement).thenReturn(measurement)
/*        every { accelerometerAndGyroscopeCalibratorSpy.gyroscopeBaseNoiseLevelAsMeasurement }.returns(
            measurement
        )*/
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibratorSpy
        )

        // check
        assertSame(measurement, calibrator.gyroscopeBaseNoiseLevelAsMeasurement)

        verify(accelerometerAndGyroscopeCalibratorSpy, times(2)).gyroscopeBaseNoiseLevelAsMeasurement
//        verify(exactly = 1) { accelerometerAndGyroscopeCalibratorSpy.gyroscopeBaseNoiseLevelAsMeasurement }
    }

    @Test
    fun getGyroscopeBaseNoiseLevelAsMeasurement_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        val result = AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND)
        assertFalse(calibrator.getGyroscopeBaseNoiseLevelAsMeasurement(result))

        // mock internal calibrator
        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val accelerometerAndGyroscopeCalibratorSpy = spy(accelerometerAndGyroscopeCalibrator)
//        val accelerometerAndGyroscopeCalibratorSpy = spyk(accelerometerAndGyroscopeCalibrator)
        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        doAnswer { invocation ->
            val w = invocation.getArgument<AngularSpeed>(0)
            w.value = value
            w.unit = AngularSpeedUnit.RADIANS_PER_SECOND
            return@doAnswer true
        }.whenever(accelerometerAndGyroscopeCalibratorSpy).getGyroscopeBaseNoiseLevelAsMeasurement(any())
/*        every { accelerometerAndGyroscopeCalibratorSpy.getGyroscopeBaseNoiseLevelAsMeasurement(any()) }.answers { answer ->
            val w = answer.invocation.args[0] as AngularSpeed
            w.value = value
            w.unit = AngularSpeedUnit.RADIANS_PER_SECOND
            return@answers true
        }*/
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibratorSpy
        )

        // check
        assertTrue(calibrator.getGyroscopeBaseNoiseLevelAsMeasurement(result))
        assertEquals(value, result.value.toDouble(), 0.0)
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, result.unit)

        verify(accelerometerAndGyroscopeCalibratorSpy, only()).getGyroscopeBaseNoiseLevelAsMeasurement(
            result
        )
/*        verify(exactly = 1) {
            accelerometerAndGyroscopeCalibratorSpy.getGyroscopeBaseNoiseLevelAsMeasurement(
                result
            )
        }*/
    }

    @Test
    fun accelerometerBaseNoiseLevel_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.accelerometerBaseNoiseLevel)

        // mock internal calibrator
        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val accelerometerAndGyroscopeCalibratorSpy = spy(accelerometerAndGyroscopeCalibrator)
//        val accelerometerAndGyroscopeCalibratorSpy = spyk(accelerometerAndGyroscopeCalibrator)
        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        whenever(accelerometerAndGyroscopeCalibratorSpy.accelerometerBaseNoiseLevel).thenReturn(value)
//        every { accelerometerAndGyroscopeCalibratorSpy.accelerometerBaseNoiseLevel }.returns(value)
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibratorSpy
        )

        // check
        assertEquals(value, calibrator.accelerometerBaseNoiseLevel)

        verify(accelerometerAndGyroscopeCalibratorSpy, times(2)).accelerometerBaseNoiseLevel
//        verify(exactly = 1) { accelerometerAndGyroscopeCalibratorSpy.accelerometerBaseNoiseLevel }
    }

    @Test
    fun accelerometerBaseNoiseLevelAsMeasurement_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.accelerometerBaseNoiseLevelAsMeasurement)

        // mock internal calibrator
        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val accelerometerAndGyroscopeCalibratorSpy = spy(accelerometerAndGyroscopeCalibrator)
//        val accelerometerAndGyroscopeCalibratorSpy = spyk(accelerometerAndGyroscopeCalibrator)
        val measurement = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        whenever(accelerometerAndGyroscopeCalibratorSpy.accelerometerBaseNoiseLevelAsMeasurement).thenReturn(measurement)
/*        every { accelerometerAndGyroscopeCalibratorSpy.accelerometerBaseNoiseLevelAsMeasurement }.returns(
            measurement
        )*/
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibratorSpy
        )

        // check
        assertSame(measurement, calibrator.accelerometerBaseNoiseLevelAsMeasurement)

        verify(accelerometerAndGyroscopeCalibratorSpy, times(2)).accelerometerBaseNoiseLevelAsMeasurement
//        verify(exactly = 1) { accelerometerAndGyroscopeCalibratorSpy.accelerometerBaseNoiseLevelAsMeasurement }
    }

    @Test
    fun getAccelerometerBaseNoiseLevelAsMeasurement_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        val result = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getAccelerometerBaseNoiseLevelAsMeasurement(result))

        // mock internal calibrator
        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val accelerometerAndGyroscopeCalibratorSpy = spy(accelerometerAndGyroscopeCalibrator)
//        val accelerometerAndGyroscopeCalibratorSpy = spyk(accelerometerAndGyroscopeCalibrator)
        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        doAnswer { invocation ->
            val a = invocation.getArgument<Acceleration>(0)
            a.value = value
            a.unit = AccelerationUnit.METERS_PER_SQUARED_SECOND
            return@doAnswer true
        }.whenever(accelerometerAndGyroscopeCalibratorSpy).getAccelerometerBaseNoiseLevelAsMeasurement(
            any()
        )
/*        every {
            accelerometerAndGyroscopeCalibratorSpy.getAccelerometerBaseNoiseLevelAsMeasurement(
                any()
            )
        }.answers { answer ->
            val a = answer.invocation.args[0] as Acceleration
            a.value = value
            a.unit = AccelerationUnit.METERS_PER_SQUARED_SECOND
            return@answers true
        }*/
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibratorSpy
        )

        // check
        assertTrue(calibrator.getAccelerometerBaseNoiseLevelAsMeasurement(result))
        assertEquals(value, result.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, result.unit)

        verify(accelerometerAndGyroscopeCalibratorSpy, only()).getAccelerometerBaseNoiseLevelAsMeasurement(
            result
        )
/*        verify(exactly = 1) {
            accelerometerAndGyroscopeCalibratorSpy.getAccelerometerBaseNoiseLevelAsMeasurement(
                result
            )
        }*/
    }

    @Test
    fun accelerometerBaseNoiseLevelPsd_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.accelerometerBaseNoiseLevelPsd)

        // mock internal calibrator
        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val accelerometerAndGyroscopeCalibratorSpy = spy(accelerometerAndGyroscopeCalibrator)
//        val accelerometerAndGyroscopeCalibratorSpy = spyk(accelerometerAndGyroscopeCalibrator)
        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        whenever(accelerometerAndGyroscopeCalibratorSpy.accelerometerBaseNoiseLevelPsd).thenReturn(value)
/*        every { accelerometerAndGyroscopeCalibratorSpy.accelerometerBaseNoiseLevelPsd }.returns(
            value
        )*/
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibratorSpy
        )

        // check
        assertEquals(value, calibrator.accelerometerBaseNoiseLevelPsd)

        verify(accelerometerAndGyroscopeCalibratorSpy, times(2)).accelerometerBaseNoiseLevelPsd
//        verify(exactly = 1) { accelerometerAndGyroscopeCalibratorSpy.accelerometerBaseNoiseLevelPsd }
    }

    @Test
    fun accelerometerBaseNoiseLevelRootPsd_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.accelerometerBaseNoiseLevelRootPsd)

        // mock internal calibrator
        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val accelerometerAndGyroscopeCalibratorSpy = spy(accelerometerAndGyroscopeCalibrator)
//        val accelerometerAndGyroscopeCalibratorSpy = spyk(accelerometerAndGyroscopeCalibrator)
        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        whenever(accelerometerAndGyroscopeCalibratorSpy.accelerometerBaseNoiseLevelRootPsd).thenReturn(value)
/*        every { accelerometerAndGyroscopeCalibratorSpy.accelerometerBaseNoiseLevelRootPsd }.returns(
            value
        )*/
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibratorSpy
        )

        // check
        assertEquals(value, calibrator.accelerometerBaseNoiseLevelRootPsd)

        verify(accelerometerAndGyroscopeCalibratorSpy, times(2)).accelerometerBaseNoiseLevelRootPsd
//        verify(exactly = 1) { accelerometerAndGyroscopeCalibratorSpy.accelerometerBaseNoiseLevelRootPsd }
    }

    @Test
    fun threshold_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.threshold)

        // mock internal calibrator
        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val accelerometerAndGyroscopeCalibratorSpy = spy(accelerometerAndGyroscopeCalibrator)
//        val accelerometerAndGyroscopeCalibratorSpy = spyk(accelerometerAndGyroscopeCalibrator)
        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        whenever(accelerometerAndGyroscopeCalibratorSpy.threshold).thenReturn(value)
//        every { accelerometerAndGyroscopeCalibratorSpy.threshold }.returns(value)
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibratorSpy
        )

        // check
        assertEquals(value, calibrator.threshold)

        verify(accelerometerAndGyroscopeCalibratorSpy, times(2)).threshold
//        verify(exactly = 1) { accelerometerAndGyroscopeCalibratorSpy.threshold }
    }

    @Test
    fun thresholdAsMeasurement_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.thresholdAsMeasurement)

        // mock internal calibrator
        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val accelerometerAndGyroscopeCalibratorSpy = spy(accelerometerAndGyroscopeCalibrator)
//        val accelerometerAndGyroscopeCalibratorSpy = spyk(accelerometerAndGyroscopeCalibrator)
        val measurement = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        whenever(accelerometerAndGyroscopeCalibratorSpy.thresholdAsMeasurement).thenReturn(measurement)
/*        every { accelerometerAndGyroscopeCalibratorSpy.thresholdAsMeasurement }.returns(
            measurement
        )*/
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibratorSpy
        )

        // check
        assertSame(measurement, calibrator.thresholdAsMeasurement)

        verify(accelerometerAndGyroscopeCalibratorSpy, times(2)).thresholdAsMeasurement
//        verify(exactly = 1) { accelerometerAndGyroscopeCalibratorSpy.thresholdAsMeasurement }
    }

    @Test
    fun getThresholdAsMeasurement_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        val result = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getThresholdAsMeasurement(result))

        // mock internal calibrator
        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val accelerometerAndGyroscopeCalibratorSpy = spy(accelerometerAndGyroscopeCalibrator)
//        val accelerometerAndGyroscopeCalibratorSpy = spyk(accelerometerAndGyroscopeCalibrator)
        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        doAnswer { invocation ->
            val a = invocation.getArgument<Acceleration>(0)
            a.value = value
            a.unit = AccelerationUnit.METERS_PER_SQUARED_SECOND
            return@doAnswer true
        }.whenever(accelerometerAndGyroscopeCalibratorSpy).getThresholdAsMeasurement(any())
/*        every { accelerometerAndGyroscopeCalibratorSpy.getThresholdAsMeasurement(any()) }.answers { answer ->
            val a = answer.invocation.args[0] as Acceleration
            a.value = value
            a.unit = AccelerationUnit.METERS_PER_SQUARED_SECOND
            return@answers true
        }*/
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibratorSpy
        )

        // check
        assertTrue(calibrator.getThresholdAsMeasurement(result))
        assertEquals(value, result.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, result.unit)

        verify(accelerometerAndGyroscopeCalibratorSpy, only()).getThresholdAsMeasurement(
            result
        )
/*        verify(exactly = 1) {
            accelerometerAndGyroscopeCalibratorSpy.getThresholdAsMeasurement(
                result
            )
        }*/
    }

    @Test
    fun processedStaticSamples_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertEquals(0, calibrator.processedStaticSamples)

        // mock internal calibrator
        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val accelerometerAndGyroscopeCalibratorSpy = spy(accelerometerAndGyroscopeCalibrator)
//        val accelerometerAndGyroscopeCalibratorSpy = spyk(accelerometerAndGyroscopeCalibrator)
        val randomizer = UniformRandomizer()
        val value = randomizer.nextInt()
        whenever(accelerometerAndGyroscopeCalibratorSpy.processedStaticSamples).thenReturn(value)
//        every { accelerometerAndGyroscopeCalibratorSpy.processedStaticSamples }.returns(value)
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibratorSpy
        )

        // check
        assertEquals(value, calibrator.processedStaticSamples)

        verify(accelerometerAndGyroscopeCalibratorSpy, times(2)).processedStaticSamples
//        verify(exactly = 1) { accelerometerAndGyroscopeCalibratorSpy.processedStaticSamples }
    }

    @Test
    fun processedDynamicSamples_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertEquals(0, calibrator.processedDynamicSamples)

        // mock internal calibrator
        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val accelerometerAndGyroscopeCalibratorSpy = spy(accelerometerAndGyroscopeCalibrator)
//        val accelerometerAndGyroscopeCalibratorSpy = spyk(accelerometerAndGyroscopeCalibrator)
        val randomizer = UniformRandomizer()
        val value = randomizer.nextInt()
        whenever(accelerometerAndGyroscopeCalibratorSpy.processedDynamicSamples).thenReturn(value)
//        every { accelerometerAndGyroscopeCalibratorSpy.processedDynamicSamples }.returns(value)
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibratorSpy
        )

        // check
        assertEquals(value, calibrator.processedDynamicSamples)

        verify(accelerometerAndGyroscopeCalibratorSpy, times(2)).processedDynamicSamples
//        verify(exactly = 1) { accelerometerAndGyroscopeCalibratorSpy.processedDynamicSamples }
    }

    @Test
    fun isStaticIntervalSkipped_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertFalse(calibrator.isStaticIntervalSkipped)

        // mock internal calibrator
        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val accelerometerAndGyroscopeCalibratorSpy = spy(accelerometerAndGyroscopeCalibrator)
//        val accelerometerAndGyroscopeCalibratorSpy = spyk(accelerometerAndGyroscopeCalibrator)
        val randomizer = UniformRandomizer()
        val value = randomizer.nextBoolean()
        whenever(accelerometerAndGyroscopeCalibratorSpy.isStaticIntervalSkipped).thenReturn(value)
//        every { accelerometerAndGyroscopeCalibratorSpy.isStaticIntervalSkipped }.returns(value)
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibratorSpy
        )

        // check
        assertEquals(value, calibrator.isStaticIntervalSkipped)

        verify(accelerometerAndGyroscopeCalibratorSpy, times(2)).isStaticIntervalSkipped
//        verify(exactly = 1) { accelerometerAndGyroscopeCalibratorSpy.isStaticIntervalSkipped }
    }

    @Test
    fun isDynamicIntervalSkipped_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertFalse(calibrator.isDynamicIntervalSkipped)

        // mock internal calibrator
        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val accelerometerAndGyroscopeCalibratorSpy = spy(accelerometerAndGyroscopeCalibrator)
//        val accelerometerAndGyroscopeCalibratorSpy = spyk(accelerometerAndGyroscopeCalibrator)
        val randomizer = UniformRandomizer()
        val value = randomizer.nextBoolean()
        whenever(accelerometerAndGyroscopeCalibratorSpy.isDynamicIntervalSkipped).thenReturn(value)
//        every { accelerometerAndGyroscopeCalibratorSpy.isDynamicIntervalSkipped }.returns(value)
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibratorSpy
        )

        // check
        assertEquals(value, calibrator.isDynamicIntervalSkipped)

        verify(accelerometerAndGyroscopeCalibratorSpy, times(2)).isDynamicIntervalSkipped
//        verify(exactly = 1) { accelerometerAndGyroscopeCalibratorSpy.isDynamicIntervalSkipped }
    }

    @Test
    fun accelerometerAverageTimeInterval_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.accelerometerAverageTimeInterval)

        // mock internal calibrator
        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val accelerometerAndGyroscopeCalibratorSpy = spy(accelerometerAndGyroscopeCalibrator)
//        val accelerometerAndGyroscopeCalibratorSpy = spyk(accelerometerAndGyroscopeCalibrator)
        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        whenever(accelerometerAndGyroscopeCalibratorSpy.accelerometerAverageTimeInterval).thenReturn(value)
/*        every { accelerometerAndGyroscopeCalibratorSpy.accelerometerAverageTimeInterval }.returns(
            value
        )*/
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibratorSpy
        )

        // check
        assertEquals(value, calibrator.accelerometerAverageTimeInterval)

        verify(accelerometerAndGyroscopeCalibratorSpy, times(2)).accelerometerAverageTimeInterval
//        verify(exactly = 1) { accelerometerAndGyroscopeCalibratorSpy.accelerometerAverageTimeInterval }
    }

    @Test
    fun accelerometerAverageTimeIntervalAsTime_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.accelerometerAverageTimeIntervalAsTime)

        // mock internal calibrator
        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val accelerometerAndGyroscopeCalibratorSpy = spy(accelerometerAndGyroscopeCalibrator)
//        val accelerometerAndGyroscopeCalibratorSpy = spyk(accelerometerAndGyroscopeCalibrator)
        val time = Time(0.0, TimeUnit.SECOND)
        whenever(accelerometerAndGyroscopeCalibratorSpy.accelerometerAverageTimeIntervalAsTime).thenReturn(time)
/*        every { accelerometerAndGyroscopeCalibratorSpy.accelerometerAverageTimeIntervalAsTime }.returns(
            time
        )*/
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibratorSpy
        )

        // check
        assertSame(time, calibrator.accelerometerAverageTimeIntervalAsTime)

        verify(accelerometerAndGyroscopeCalibratorSpy, times(2)).accelerometerAverageTimeIntervalAsTime
//        verify(exactly = 1) { accelerometerAndGyroscopeCalibratorSpy.accelerometerAverageTimeIntervalAsTime }
    }

    @Test
    fun getAccelerometerAverageTimeIntervalAsTime_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        val time = Time(0.0, TimeUnit.SECOND)
        assertFalse(calibrator.getAccelerometerAverageTimeIntervalAsTime(time))

        // mock internal calibrator
        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val accelerometerAndGyroscopeCalibratorSpy = spy(accelerometerAndGyroscopeCalibrator)
//        val accelerometerAndGyroscopeCalibratorSpy = spyk(accelerometerAndGyroscopeCalibrator)
        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        doAnswer { invocation ->
            val t = invocation.getArgument<Time>(0)
            t.value = value
            t.unit = TimeUnit.MILLISECOND
            return@doAnswer true
        }.whenever(accelerometerAndGyroscopeCalibratorSpy).getAccelerometerAverageTimeIntervalAsTime(any())
/*        every { accelerometerAndGyroscopeCalibratorSpy.getAccelerometerAverageTimeIntervalAsTime(any()) }.answers { answer ->
            val t = answer.invocation.args[0] as Time
            t.value = value
            t.unit = TimeUnit.MILLISECOND
            return@answers true
        }*/
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibratorSpy
        )

        // check
        assertTrue(calibrator.getAccelerometerAverageTimeIntervalAsTime(time))
        assertEquals(value, time.value.toDouble(), 0.0)
        assertEquals(TimeUnit.MILLISECOND, time.unit)

        verify(accelerometerAndGyroscopeCalibratorSpy, only()).getAccelerometerAverageTimeIntervalAsTime(
            time
        )
/*        verify(exactly = 1) {
            accelerometerAndGyroscopeCalibratorSpy.getAccelerometerAverageTimeIntervalAsTime(
                time
            )
        }*/
    }

    @Test
    fun accelerometerTimeIntervalVariance_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.accelerometerTimeIntervalVariance)

        // mock internal calibrator
        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val accelerometerAndGyroscopeCalibratorSpy = spy(accelerometerAndGyroscopeCalibrator)
//        val accelerometerAndGyroscopeCalibratorSpy = spyk(accelerometerAndGyroscopeCalibrator)
        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        whenever(accelerometerAndGyroscopeCalibratorSpy.accelerometerTimeIntervalVariance).thenReturn(value)
/*        every { accelerometerAndGyroscopeCalibratorSpy.accelerometerTimeIntervalVariance }.returns(
            value
        )*/
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibratorSpy
        )

        // check
        assertEquals(value, calibrator.accelerometerTimeIntervalVariance)

        verify(accelerometerAndGyroscopeCalibratorSpy, times(2)).accelerometerTimeIntervalVariance
//        verify(exactly = 1) { accelerometerAndGyroscopeCalibratorSpy.accelerometerTimeIntervalVariance }
    }

    @Test
    fun accelerometerTimeIntervalStandardDeviation_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.accelerometerTimeIntervalStandardDeviation)

        // mock internal calibrator
        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val accelerometerAndGyroscopeCalibratorSpy = spy(accelerometerAndGyroscopeCalibrator)
//        val accelerometerAndGyroscopeCalibratorSpy = spyk(accelerometerAndGyroscopeCalibrator)
        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        whenever(accelerometerAndGyroscopeCalibratorSpy.accelerometerTimeIntervalStandardDeviation).thenReturn(value)
/*        every { accelerometerAndGyroscopeCalibratorSpy.accelerometerTimeIntervalStandardDeviation }.returns(
            value
        )*/
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibratorSpy
        )

        // check
        assertEquals(value, calibrator.accelerometerTimeIntervalStandardDeviation)

        verify(accelerometerAndGyroscopeCalibratorSpy, times(2)).accelerometerTimeIntervalStandardDeviation
//        verify(exactly = 1) { accelerometerAndGyroscopeCalibratorSpy.accelerometerTimeIntervalStandardDeviation }
    }

    @Test
    fun accelerometerTimeIntervalStandardDeviationAsTime_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.accelerometerTimeIntervalStandardDeviationAsTime)

        // mock internal calibrator
        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val accelerometerAndGyroscopeCalibratorSpy = spy(accelerometerAndGyroscopeCalibrator)
//        val accelerometerAndGyroscopeCalibratorSpy = spyk(accelerometerAndGyroscopeCalibrator)
        val time = Time(0.0, TimeUnit.SECOND)
        whenever(accelerometerAndGyroscopeCalibratorSpy.accelerometerTimeIntervalStandardDeviationAsTime).thenReturn(time)
/*        every { accelerometerAndGyroscopeCalibratorSpy.accelerometerTimeIntervalStandardDeviationAsTime }.returns(
            time
        )*/
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibratorSpy
        )

        // check
        assertSame(time, calibrator.accelerometerTimeIntervalStandardDeviationAsTime)

        verify(accelerometerAndGyroscopeCalibratorSpy, times(2)).accelerometerTimeIntervalStandardDeviationAsTime
//        verify(exactly = 1) { accelerometerAndGyroscopeCalibratorSpy.accelerometerTimeIntervalStandardDeviationAsTime }
    }

    @Test
    fun getAccelerometerTimeIntervalStandardDeviationAsTime_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        val time = Time(0.0, TimeUnit.SECOND)
        assertFalse(calibrator.getAccelerometerTimeIntervalStandardDeviationAsTime(time))

        // mock internal calibrator
        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val accelerometerAndGyroscopeCalibratorSpy = spy(accelerometerAndGyroscopeCalibrator)
//        val accelerometerAndGyroscopeCalibratorSpy = spyk(accelerometerAndGyroscopeCalibrator)
        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        doAnswer { invocation ->
            val t = invocation.getArgument<Time>(0)
            t.value = value
            t.unit = TimeUnit.MILLISECOND
            return@doAnswer true
        }.whenever(accelerometerAndGyroscopeCalibratorSpy).getAccelerometerTimeIntervalStandardDeviationAsTime(
            any()
        )
/*        every {
            accelerometerAndGyroscopeCalibratorSpy.getAccelerometerTimeIntervalStandardDeviationAsTime(
                any()
            )
        }.answers { answer ->
            val t = answer.invocation.args[0] as Time
            t.value = value
            t.unit = TimeUnit.MILLISECOND
            return@answers true
        }*/
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibratorSpy
        )

        // check
        assertTrue(calibrator.getAccelerometerTimeIntervalStandardDeviationAsTime(time))
        assertEquals(value, time.value.toDouble(), 0.0)
        assertEquals(TimeUnit.MILLISECOND, time.unit)

        verify(accelerometerAndGyroscopeCalibratorSpy, only()).getAccelerometerTimeIntervalStandardDeviationAsTime(
            time
        )
/*        verify(exactly = 1) {
            accelerometerAndGyroscopeCalibratorSpy.getAccelerometerTimeIntervalStandardDeviationAsTime(
                time
            )
        }*/
    }

    @Test
    fun numberOfProcessedGyroscopeMeasurements_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertEquals(0, calibrator.numberOfProcessedGyroscopeMeasurements)

        // mock internal calibrator
        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val accelerometerAndGyroscopeCalibratorSpy = spy(accelerometerAndGyroscopeCalibrator)
//        val accelerometerAndGyroscopeCalibratorSpy = spyk(accelerometerAndGyroscopeCalibrator)
        val randomizer = UniformRandomizer()
        val value = randomizer.nextInt()
        whenever(accelerometerAndGyroscopeCalibratorSpy.numberOfProcessedGyroscopeMeasurements).thenReturn(value)
/*        every { accelerometerAndGyroscopeCalibratorSpy.numberOfProcessedGyroscopeMeasurements }.returns(
            value
        )*/
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibratorSpy
        )

        // check
        assertEquals(value, calibrator.numberOfProcessedGyroscopeMeasurements)

        verify(accelerometerAndGyroscopeCalibratorSpy, times(2)).numberOfProcessedGyroscopeMeasurements
//        verify(exactly = 1) { accelerometerAndGyroscopeCalibratorSpy.numberOfProcessedGyroscopeMeasurements }
    }

    @Test
    fun numberOfProcessedAccelerometerMeasurements_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertEquals(0, calibrator.numberOfProcessedAccelerometerMeasurements)

        // mock internal calibrator
        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val accelerometerAndGyroscopeCalibratorSpy = spy(accelerometerAndGyroscopeCalibrator)
//        val accelerometerAndGyroscopeCalibratorSpy = spyk(accelerometerAndGyroscopeCalibrator)
        val randomizer = UniformRandomizer()
        val value = randomizer.nextInt()
        whenever(accelerometerAndGyroscopeCalibratorSpy.numberOfProcessedAccelerometerMeasurements).thenReturn(value)
/*        every { accelerometerAndGyroscopeCalibratorSpy.numberOfProcessedAccelerometerMeasurements }.returns(
            value
        )*/
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibratorSpy
        )

        // check
        assertEquals(value, calibrator.numberOfProcessedAccelerometerMeasurements)

        verify(accelerometerAndGyroscopeCalibratorSpy, times(2)).numberOfProcessedAccelerometerMeasurements
//        verify(exactly = 1) { accelerometerAndGyroscopeCalibratorSpy.numberOfProcessedAccelerometerMeasurements }
    }

    @Test
    fun gyroscopeInitialBiasX_getsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.gyroscopeInitialBiasX)

        // mock internal calibrator
        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val accelerometerAndGyroscopeCalibratorSpy = spy(accelerometerAndGyroscopeCalibrator)
//        val accelerometerAndGyroscopeCalibratorSpy = spyk(accelerometerAndGyroscopeCalibrator)
        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        whenever(accelerometerAndGyroscopeCalibratorSpy.gyroscopeInitialBiasX).thenReturn(value)
//        every { accelerometerAndGyroscopeCalibratorSpy.gyroscopeInitialBiasX }.returns(value)
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibratorSpy
        )

        // check
        assertEquals(value, calibrator.gyroscopeInitialBiasX)

        verify(accelerometerAndGyroscopeCalibratorSpy, only()).gyroscopeInitialBiasX
//        verify(exactly = 1) { accelerometerAndGyroscopeCalibratorSpy.gyroscopeInitialBiasX }
    }

    @Test
    fun gyroscopeInitialBiasY_getsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.gyroscopeInitialBiasY)

        // mock internal calibrator
        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val accelerometerAndGyroscopeCalibratorSpy = spy(accelerometerAndGyroscopeCalibrator)
//        val accelerometerAndGyroscopeCalibratorSpy = spyk(accelerometerAndGyroscopeCalibrator)
        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        whenever(accelerometerAndGyroscopeCalibratorSpy.gyroscopeInitialBiasY).thenReturn(value)
//        every { accelerometerAndGyroscopeCalibratorSpy.gyroscopeInitialBiasY }.returns(value)
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibratorSpy
        )

        // check
        assertEquals(value, calibrator.gyroscopeInitialBiasY)

        verify(accelerometerAndGyroscopeCalibratorSpy, only()).gyroscopeInitialBiasY
//        verify(exactly = 1) { accelerometerAndGyroscopeCalibratorSpy.gyroscopeInitialBiasY }
    }

    @Test
    fun gyroscopeInitialBiasZ_getsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.gyroscopeInitialBiasZ)

        // mock internal calibrator
        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val accelerometerAndGyroscopeCalibratorSpy = spy(accelerometerAndGyroscopeCalibrator)
//        val accelerometerAndGyroscopeCalibratorSpy = spyk(accelerometerAndGyroscopeCalibrator)
        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        whenever(accelerometerAndGyroscopeCalibratorSpy.gyroscopeInitialBiasZ).thenReturn(value)
//        every { accelerometerAndGyroscopeCalibratorSpy.gyroscopeInitialBiasZ }.returns(value)
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibratorSpy
        )

        // check
        assertEquals(value, calibrator.gyroscopeInitialBiasZ)

        verify(accelerometerAndGyroscopeCalibratorSpy, only()).gyroscopeInitialBiasZ
//        verify(exactly = 1) { accelerometerAndGyroscopeCalibratorSpy.gyroscopeInitialBiasZ }
    }

    @Test
    fun gyroscopeInitialBiasXAsMeasurement_getsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.gyroscopeInitialBiasXAsMeasurement)

        // mock internal calibrator
        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val accelerometerAndGyroscopeCalibratorSpy = spy(accelerometerAndGyroscopeCalibrator)
//        val accelerometerAndGyroscopeCalibratorSpy = spyk(accelerometerAndGyroscopeCalibrator)
        val value = AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND)
        whenever(accelerometerAndGyroscopeCalibratorSpy.gyroscopeInitialBiasXAsMeasurement).thenReturn(value)
/*        every { accelerometerAndGyroscopeCalibratorSpy.gyroscopeInitialBiasXAsMeasurement }.returns(
            value
        )*/
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibratorSpy
        )

        // check
        assertSame(value, calibrator.gyroscopeInitialBiasXAsMeasurement)

        verify(accelerometerAndGyroscopeCalibratorSpy, only()).gyroscopeInitialBiasXAsMeasurement
//        verify(exactly = 1) { accelerometerAndGyroscopeCalibratorSpy.gyroscopeInitialBiasXAsMeasurement }
    }

    @Test
    fun getGyroscopeInitialBiasXAsMeasurement_getsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        val result = AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND)
        assertFalse(calibrator.getGyroscopeInitialBiasXAsMeasurement(result))

        // mock internal calibrator
        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val accelerometerAndGyroscopeCalibratorSpy = spy(accelerometerAndGyroscopeCalibrator)
//        val accelerometerAndGyroscopeCalibratorSpy = spyk(accelerometerAndGyroscopeCalibrator)
        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        doAnswer { invocation ->
            val w = invocation.getArgument<AngularSpeed>(0)
            w.value = value
            w.unit = AngularSpeedUnit.RADIANS_PER_SECOND
            return@doAnswer true
        }.whenever(accelerometerAndGyroscopeCalibratorSpy).getGyroscopeInitialBiasXAsMeasurement(any())
/*        every { accelerometerAndGyroscopeCalibratorSpy.getGyroscopeInitialBiasXAsMeasurement(any()) }.answers { answer ->
            val w = answer.invocation.args[0] as AngularSpeed
            w.value = value
            w.unit = AngularSpeedUnit.RADIANS_PER_SECOND
            return@answers true
        }*/
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibratorSpy
        )

        // check
        assertTrue(calibrator.getGyroscopeInitialBiasXAsMeasurement(result))
        assertEquals(value, result.value.toDouble(), 0.0)
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, result.unit)

        verify(accelerometerAndGyroscopeCalibratorSpy, only()).getGyroscopeInitialBiasXAsMeasurement(
            result
        )
/*        verify(exactly = 1) {
            accelerometerAndGyroscopeCalibratorSpy.getGyroscopeInitialBiasXAsMeasurement(
                result
            )
        }*/
    }

    @Test
    fun gyroscopeInitialBiasYAsMeasurement_getsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.gyroscopeInitialBiasYAsMeasurement)

        // mock internal calibrator
        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val accelerometerAndGyroscopeCalibratorSpy = spy(accelerometerAndGyroscopeCalibrator)
//        val accelerometerAndGyroscopeCalibratorSpy = spyk(accelerometerAndGyroscopeCalibrator)
        val value = AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND)
        whenever(accelerometerAndGyroscopeCalibratorSpy.gyroscopeInitialBiasYAsMeasurement).thenReturn(value)
/*        every { accelerometerAndGyroscopeCalibratorSpy.gyroscopeInitialBiasYAsMeasurement }.returns(
            value
        )*/
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibratorSpy
        )

        // check
        assertSame(value, calibrator.gyroscopeInitialBiasYAsMeasurement)

        verify(accelerometerAndGyroscopeCalibratorSpy, only()).gyroscopeInitialBiasYAsMeasurement
//        verify(exactly = 1) { accelerometerAndGyroscopeCalibratorSpy.gyroscopeInitialBiasYAsMeasurement }
    }

    @Test
    fun getGyroscopeInitialBiasYAsMeasurement_getsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        val result = AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND)
        assertFalse(calibrator.getGyroscopeInitialBiasYAsMeasurement(result))

        // mock internal calibrator
        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val accelerometerAndGyroscopeCalibratorSpy = spy(accelerometerAndGyroscopeCalibrator)
//        val accelerometerAndGyroscopeCalibratorSpy = spyk(accelerometerAndGyroscopeCalibrator)
        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        doAnswer { invocation ->
            val w = invocation.getArgument<AngularSpeed>(0)
            w.value = value
            w.unit = AngularSpeedUnit.RADIANS_PER_SECOND
            return@doAnswer true
        }.whenever(accelerometerAndGyroscopeCalibratorSpy).getGyroscopeInitialBiasYAsMeasurement(any())
/*        every { accelerometerAndGyroscopeCalibratorSpy.getGyroscopeInitialBiasYAsMeasurement(any()) }.answers { answer ->
            val w = answer.invocation.args[0] as AngularSpeed
            w.value = value
            w.unit = AngularSpeedUnit.RADIANS_PER_SECOND
            return@answers true
        }*/
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibratorSpy
        )

        // check
        assertTrue(calibrator.getGyroscopeInitialBiasYAsMeasurement(result))
        assertEquals(value, result.value.toDouble(), 0.0)
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, result.unit)

        verify(accelerometerAndGyroscopeCalibratorSpy, only()).getGyroscopeInitialBiasYAsMeasurement(
            result
        )
/*        verify(exactly = 1) {
            accelerometerAndGyroscopeCalibratorSpy.getGyroscopeInitialBiasYAsMeasurement(
                result
            )
        }*/
    }

    @Test
    fun gyroscopeInitialBiasZAsMeasurement_getsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.gyroscopeInitialBiasZAsMeasurement)

        // mock internal calibrator
        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val accelerometerAndGyroscopeCalibratorSpy = spy(accelerometerAndGyroscopeCalibrator)
//        val accelerometerAndGyroscopeCalibratorSpy = spyk(accelerometerAndGyroscopeCalibrator)
        val value = AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND)
        whenever(accelerometerAndGyroscopeCalibratorSpy.gyroscopeInitialBiasZAsMeasurement).thenReturn(value)
/*        every { accelerometerAndGyroscopeCalibratorSpy.gyroscopeInitialBiasZAsMeasurement }.returns(
            value
        )*/
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibratorSpy
        )

        // check
        assertSame(value, calibrator.gyroscopeInitialBiasZAsMeasurement)

        verify(accelerometerAndGyroscopeCalibratorSpy, only()).gyroscopeInitialBiasZAsMeasurement
//        verify(exactly = 1) { accelerometerAndGyroscopeCalibratorSpy.gyroscopeInitialBiasZAsMeasurement }
    }

    @Test
    fun getGyroscopeInitialBiasZAsMeasurement_getsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        val result = AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND)
        assertFalse(calibrator.getGyroscopeInitialBiasZAsMeasurement(result))

        // mock internal calibrator
        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val accelerometerAndGyroscopeCalibratorSpy = spy(accelerometerAndGyroscopeCalibrator)
//        val accelerometerAndGyroscopeCalibratorSpy = spyk(accelerometerAndGyroscopeCalibrator)
        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        doAnswer { invocation ->
            val w = invocation.getArgument<AngularSpeed>(0)
            w.value = value
            w.unit = AngularSpeedUnit.RADIANS_PER_SECOND
            return@doAnswer true
        }.whenever(accelerometerAndGyroscopeCalibratorSpy).getGyroscopeInitialBiasZAsMeasurement(any())
/*        every { accelerometerAndGyroscopeCalibratorSpy.getGyroscopeInitialBiasZAsMeasurement(any()) }.answers { answer ->
            val w = answer.invocation.args[0] as AngularSpeed
            w.value = value
            w.unit = AngularSpeedUnit.RADIANS_PER_SECOND
            return@answers true
        }*/
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibratorSpy
        )

        // check
        assertTrue(calibrator.getGyroscopeInitialBiasZAsMeasurement(result))
        assertEquals(value, result.value.toDouble(), 0.0)
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, result.unit)

        verify(accelerometerAndGyroscopeCalibratorSpy, only()).getGyroscopeInitialBiasZAsMeasurement(
            result
        )
/*        verify(exactly = 1) {
            accelerometerAndGyroscopeCalibratorSpy.getGyroscopeInitialBiasZAsMeasurement(
                result
            )
        }*/
    }

    @Test
    fun gyroscopeInitialBiasAsTriad_getsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.gyroscopeInitialBiasAsTriad)

        // mock internal calibrator
        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val accelerometerAndGyroscopeCalibratorSpy = spy(accelerometerAndGyroscopeCalibrator)
//        val accelerometerAndGyroscopeCalibratorSpy = spyk(accelerometerAndGyroscopeCalibrator)
        val value = AngularSpeedTriad()
        whenever(accelerometerAndGyroscopeCalibratorSpy.gyroscopeInitialBiasAsTriad).thenReturn(value)
/*        every { accelerometerAndGyroscopeCalibratorSpy.gyroscopeInitialBiasAsTriad }.returns(
            value
        )*/
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibratorSpy
        )

        // check
        assertSame(value, calibrator.gyroscopeInitialBiasAsTriad)

        verify(accelerometerAndGyroscopeCalibratorSpy, only()).gyroscopeInitialBiasAsTriad
//        verify(exactly = 1) { accelerometerAndGyroscopeCalibratorSpy.gyroscopeInitialBiasAsTriad }
    }

    @Test
    fun getGyroscopeInitialBiasAsTriad_getsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        val result = AngularSpeedTriad()
        assertFalse(calibrator.getGyroscopeInitialBiasAsTriad(result))

        // mock internal calibrator
        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val accelerometerAndGyroscopeCalibratorSpy = spy(accelerometerAndGyroscopeCalibrator)
//        val accelerometerAndGyroscopeCalibratorSpy = spyk(accelerometerAndGyroscopeCalibrator)
        val randomizer = UniformRandomizer()
        val valueX = randomizer.nextDouble()
        val valueY = randomizer.nextDouble()
        val valueZ = randomizer.nextDouble()
        doAnswer { invocation ->
            val triad = invocation.getArgument<AngularSpeedTriad>(0)
            triad.setValueCoordinatesAndUnit(
                valueX,
                valueY,
                valueZ,
                AngularSpeedUnit.RADIANS_PER_SECOND
            )
            return@doAnswer true
        }.whenever(accelerometerAndGyroscopeCalibratorSpy).getGyroscopeInitialBiasAsTriad(any())
/*        every { accelerometerAndGyroscopeCalibratorSpy.getGyroscopeInitialBiasAsTriad(any()) }.answers { answer ->
            val triad = answer.invocation.args[0] as AngularSpeedTriad
            triad.setValueCoordinatesAndUnit(
                valueX,
                valueY,
                valueZ,
                AngularSpeedUnit.RADIANS_PER_SECOND
            )
            return@answers true
        }*/
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibratorSpy
        )

        // check
        assertTrue(calibrator.getGyroscopeInitialBiasAsTriad(result))
        assertEquals(valueX, result.valueX, 0.0)
        assertEquals(valueY, result.valueY, 0.0)
        assertEquals(valueZ, result.valueZ, 0.0)
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, result.unit)

        verify(accelerometerAndGyroscopeCalibratorSpy, only()).getGyroscopeInitialBiasAsTriad(
            result
        )
/*        verify(exactly = 1) {
            accelerometerAndGyroscopeCalibratorSpy.getGyroscopeInitialBiasAsTriad(
                result
            )
        }*/
    }

    @Test
    fun magnetometerInitialHardIronX_getsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.magnetometerInitialHardIronX)

        // mock internal calibrator
        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        val magnetometerCalibratorSpy = spy(magnetometerCalibrator)
//        val magnetometerCalibratorSpy = spyk(magnetometerCalibrator)
        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        whenever(magnetometerCalibratorSpy.initialHardIronX).thenReturn(value)
//        every { magnetometerCalibratorSpy.initialHardIronX }.returns(value)
        calibrator.setPrivateProperty(
            "magnetometerCalibrator",
            magnetometerCalibratorSpy
        )

        // check
        assertEquals(value, calibrator.magnetometerInitialHardIronX)

        verify(magnetometerCalibratorSpy, only()).initialHardIronX
//        verify(exactly = 1) { magnetometerCalibratorSpy.initialHardIronX }
    }

    @Test
    fun magnetometerInitialHardIronY_getsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.magnetometerInitialHardIronY)

        // mock internal calibrator
        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        val magnetometerCalibratorSpy = spy(magnetometerCalibrator)
//        val magnetometerCalibratorSpy = spyk(magnetometerCalibrator)
        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        whenever(magnetometerCalibratorSpy.initialHardIronY).thenReturn(value)
//        every { magnetometerCalibratorSpy.initialHardIronY }.returns(value)
        calibrator.setPrivateProperty(
            "magnetometerCalibrator",
            magnetometerCalibratorSpy
        )

        // check
        assertEquals(value, calibrator.magnetometerInitialHardIronY)

        verify(magnetometerCalibratorSpy, only()).initialHardIronY
//        verify(exactly = 1) { magnetometerCalibratorSpy.initialHardIronY }
    }

    @Test
    fun magnetometerInitialHardIronZ_getsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.magnetometerInitialHardIronZ)

        // mock internal calibrator
        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        val magnetometerCalibratorSpy = spy(magnetometerCalibrator)
//        val magnetometerCalibratorSpy = spyk(magnetometerCalibrator)
        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        whenever(magnetometerCalibratorSpy.initialHardIronZ).thenReturn(value)
//        every { magnetometerCalibratorSpy.initialHardIronZ }.returns(value)
        calibrator.setPrivateProperty(
            "magnetometerCalibrator",
            magnetometerCalibratorSpy
        )

        // check
        assertEquals(value, calibrator.magnetometerInitialHardIronZ)

        verify(magnetometerCalibratorSpy, only()).initialHardIronZ
//        verify(exactly = 1) { magnetometerCalibratorSpy.initialHardIronZ }
    }

    @Test
    fun magnetometerInitialHardIronXAsMeasurement_getsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.magnetometerInitialHardIronXAsMeasurement)

        // mock internal calibrator
        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        val magnetometerCalibratorSpy = spy(magnetometerCalibrator)
//        val magnetometerCalibratorSpy = spyk(magnetometerCalibrator)
        val value = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        whenever(magnetometerCalibratorSpy.initialHardIronXAsMeasurement).thenReturn(value)
//        every { magnetometerCalibratorSpy.initialHardIronXAsMeasurement }.returns(value)
        calibrator.setPrivateProperty(
            "magnetometerCalibrator",
            magnetometerCalibratorSpy
        )

        // check
        assertSame(value, calibrator.magnetometerInitialHardIronXAsMeasurement)

        verify(magnetometerCalibratorSpy, only()).initialHardIronXAsMeasurement
//        verify(exactly = 1) { magnetometerCalibratorSpy.initialHardIronXAsMeasurement }
    }

    @Test
    fun getMagnetometerInitialHardIronXAsMeasurement_getsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        val result = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(calibrator.getMagnetometerInitialHardIronXAsMeasurement(result))

        // mock internal calibrator
        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        val magnetometerCalibratorSpy = spy(magnetometerCalibrator)
//        val magnetometerCalibratorSpy = spyk(magnetometerCalibrator)
        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        doAnswer { invocation ->
            val b = invocation.getArgument<MagneticFluxDensity>(0)
            b.value = value
            b.unit = MagneticFluxDensityUnit.TESLA
            return@doAnswer true
        }.whenever(magnetometerCalibratorSpy).getInitialHardIronXAsMeasurement(any())
/*        every { magnetometerCalibratorSpy.getInitialHardIronXAsMeasurement(any()) }.answers { answer ->
            val b = answer.invocation.args[0] as MagneticFluxDensity
            b.value = value
            b.unit = MagneticFluxDensityUnit.TESLA
            return@answers true
        }*/
        calibrator.setPrivateProperty(
            "magnetometerCalibrator",
            magnetometerCalibratorSpy
        )

        // check
        assertTrue(calibrator.getMagnetometerInitialHardIronXAsMeasurement(result))
        assertEquals(value, result.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, result.unit)

        verify(magnetometerCalibratorSpy, only()).getInitialHardIronXAsMeasurement(result)
//        verify(exactly = 1) { magnetometerCalibratorSpy.getInitialHardIronXAsMeasurement(result) }
    }

    @Test
    fun magnetometerInitialHardIronYAsMeasurement_getsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.magnetometerInitialHardIronYAsMeasurement)

        // mock internal calibrator
        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        val magnetometerCalibratorSpy = spy(magnetometerCalibrator)
//        val magnetometerCalibratorSpy = spyk(magnetometerCalibrator)
        val value = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        whenever(magnetometerCalibratorSpy.initialHardIronYAsMeasurement).thenReturn(value)
//        every { magnetometerCalibratorSpy.initialHardIronYAsMeasurement }.returns(value)
        calibrator.setPrivateProperty(
            "magnetometerCalibrator",
            magnetometerCalibratorSpy
        )

        // check
        assertSame(value, calibrator.magnetometerInitialHardIronYAsMeasurement)

        verify(magnetometerCalibratorSpy, only()).initialHardIronYAsMeasurement
//        verify(exactly = 1) { magnetometerCalibratorSpy.initialHardIronYAsMeasurement }
    }

    @Test
    fun getMagnetometerInitialHardIronYAsMeasurement_getsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        val result = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(calibrator.getMagnetometerInitialHardIronYAsMeasurement(result))

        // mock internal calibrator
        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        val magnetometerCalibratorSpy = spy(magnetometerCalibrator)
//        val magnetometerCalibratorSpy = spyk(magnetometerCalibrator)
        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        doAnswer { invocation ->
            val b = invocation.getArgument<MagneticFluxDensity>(0)
            b.value = value
            b.unit = MagneticFluxDensityUnit.TESLA
            return@doAnswer true
        }.whenever(magnetometerCalibratorSpy).getInitialHardIronYAsMeasurement(any())
/*        every { magnetometerCalibratorSpy.getInitialHardIronYAsMeasurement(any()) }.answers { answer ->
            val b = answer.invocation.args[0] as MagneticFluxDensity
            b.value = value
            b.unit = MagneticFluxDensityUnit.TESLA
            return@answers true
        }*/
        calibrator.setPrivateProperty(
            "magnetometerCalibrator",
            magnetometerCalibratorSpy
        )

        // check
        assertTrue(calibrator.getMagnetometerInitialHardIronYAsMeasurement(result))
        assertEquals(value, result.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, result.unit)

        verify(magnetometerCalibratorSpy, only()).getInitialHardIronYAsMeasurement(result)
//        verify(exactly = 1) { magnetometerCalibratorSpy.getInitialHardIronYAsMeasurement(result) }
    }

    @Test
    fun magnetometerInitialHardIronZAsMeasurement_getsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.magnetometerInitialHardIronZAsMeasurement)

        // mock internal calibrator
        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        val magnetometerCalibratorSpy = spy(magnetometerCalibrator)
//        val magnetometerCalibratorSpy = spyk(magnetometerCalibrator)
        val value = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        whenever(magnetometerCalibratorSpy.initialHardIronZAsMeasurement).thenReturn(value)
//        every { magnetometerCalibratorSpy.initialHardIronZAsMeasurement }.returns(value)
        calibrator.setPrivateProperty(
            "magnetometerCalibrator",
            magnetometerCalibratorSpy
        )

        // check
        assertSame(value, calibrator.magnetometerInitialHardIronZAsMeasurement)

        verify(magnetometerCalibratorSpy, only()).initialHardIronZAsMeasurement
//        verify(exactly = 1) { magnetometerCalibratorSpy.initialHardIronZAsMeasurement }
    }

    @Test
    fun getMagnetometerInitialHardIronZAsMeasurement_getsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        val result = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(calibrator.getMagnetometerInitialHardIronZAsMeasurement(result))

        // mock internal calibrator
        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        val magnetometerCalibratorSpy = spy(magnetometerCalibrator)
//        val magnetometerCalibratorSpy = spyk(magnetometerCalibrator)
        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        doAnswer { invocation ->
            val b = invocation.getArgument<MagneticFluxDensity>(0)
            b.value = value
            b.unit = MagneticFluxDensityUnit.TESLA
            return@doAnswer true
        }.whenever(magnetometerCalibratorSpy).getInitialHardIronZAsMeasurement(any())
/*        every { magnetometerCalibratorSpy.getInitialHardIronZAsMeasurement(any()) }.answers { answer ->
            val b = answer.invocation.args[0] as MagneticFluxDensity
            b.value = value
            b.unit = MagneticFluxDensityUnit.TESLA
            return@answers true
        }*/
        calibrator.setPrivateProperty(
            "magnetometerCalibrator",
            magnetometerCalibratorSpy
        )

        // check
        assertTrue(calibrator.getMagnetometerInitialHardIronZAsMeasurement(result))
        assertEquals(value, result.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, result.unit)

        verify(magnetometerCalibratorSpy, only()).getInitialHardIronZAsMeasurement(result)
//        verify(exactly = 1) { magnetometerCalibratorSpy.getInitialHardIronZAsMeasurement(result) }
    }

    @Test
    fun magnetometerInitialHardIronAsTriad_getsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.magnetometerInitialHardIronAsTriad)

        // mock internal calibrator
        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        val magnetometerCalibratorSpy = spy(magnetometerCalibrator)
//        val magnetometerCalibratorSpy = spyk(magnetometerCalibrator)
        val value = MagneticFluxDensityTriad()
        whenever(magnetometerCalibratorSpy.initialHardIronAsTriad).thenReturn(value)
//        every { magnetometerCalibratorSpy.initialHardIronAsTriad }.returns(value)
        calibrator.setPrivateProperty(
            "magnetometerCalibrator",
            magnetometerCalibratorSpy
        )

        // check
        assertSame(value, calibrator.magnetometerInitialHardIronAsTriad)

        verify(magnetometerCalibratorSpy, only()).initialHardIronAsTriad
//        verify(exactly = 1) { magnetometerCalibratorSpy.initialHardIronAsTriad }
    }

    @Test
    fun getMagnetometerInitialHardIronAsTriad_getsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        val result = MagneticFluxDensityTriad()
        assertFalse(calibrator.getMagnetometerInitialHardIronAsTriad(result))

        // mock internal calibrator
        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        val magnetometerCalibratorSpy = spy(magnetometerCalibrator)
//        val magnetometerCalibratorSpy = spyk(magnetometerCalibrator)
        val randomizer = UniformRandomizer()
        val valueX = randomizer.nextDouble()
        val valueY = randomizer.nextDouble()
        val valueZ = randomizer.nextDouble()
        doAnswer { invocation ->
            val triad = invocation.getArgument<MagneticFluxDensityTriad>(0)
            triad.setValueCoordinatesAndUnit(valueX, valueY, valueZ, MagneticFluxDensityUnit.TESLA)
            return@doAnswer true
        }.whenever(magnetometerCalibratorSpy).getInitialHardIronAsTriad(any())
/*        every { magnetometerCalibratorSpy.getInitialHardIronAsTriad(any()) }.answers { answer ->
            val triad = answer.invocation.args[0] as MagneticFluxDensityTriad
            triad.setValueCoordinatesAndUnit(valueX, valueY, valueZ, MagneticFluxDensityUnit.TESLA)
            return@answers true
        }*/
        calibrator.setPrivateProperty(
            "magnetometerCalibrator",
            magnetometerCalibratorSpy
        )

        // check
        assertTrue(calibrator.getMagnetometerInitialHardIronAsTriad(result))
        assertEquals(valueX, result.valueX, 0.0)
        assertEquals(valueY, result.valueY, 0.0)
        assertEquals(valueZ, result.valueZ, 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, result.unit)

        verify(magnetometerCalibratorSpy, only()).getInitialHardIronAsTriad(result)
//        verify(exactly = 1) { magnetometerCalibratorSpy.getInitialHardIronAsTriad(result) }
    }

    @Test
    fun accelerometerInitialBiasX_getsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.accelerometerInitialBiasX)

        // mock internal calibrator
        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val accelerometerAndGyroscopeCalibratorSpy = spy(accelerometerAndGyroscopeCalibrator)
//        val accelerometerAndGyroscopeCalibratorSpy = spyk(accelerometerAndGyroscopeCalibrator)
        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        whenever(accelerometerAndGyroscopeCalibratorSpy.accelerometerInitialBiasX).thenReturn(value)
//        every { accelerometerAndGyroscopeCalibratorSpy.accelerometerInitialBiasX }.returns(value)
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibratorSpy
        )

        // check
        assertEquals(value, calibrator.accelerometerInitialBiasX)

        verify(accelerometerAndGyroscopeCalibratorSpy, only()).accelerometerInitialBiasX
//        verify(exactly = 1) { accelerometerAndGyroscopeCalibratorSpy.accelerometerInitialBiasX }
    }

    @Test
    fun accelerometerInitialBiasY_getsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.accelerometerInitialBiasY)

        // mock internal calibrator
        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val accelerometerAndGyroscopeCalibratorSpy = spy(accelerometerAndGyroscopeCalibrator)
//        val accelerometerAndGyroscopeCalibratorSpy = spyk(accelerometerAndGyroscopeCalibrator)
        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        whenever(accelerometerAndGyroscopeCalibratorSpy.accelerometerInitialBiasY).thenReturn(value)
//        every { accelerometerAndGyroscopeCalibratorSpy.accelerometerInitialBiasY }.returns(value)
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibratorSpy
        )

        // check
        assertEquals(value, calibrator.accelerometerInitialBiasY)

        verify(accelerometerAndGyroscopeCalibratorSpy, only()).accelerometerInitialBiasY
//        verify(exactly = 1) { accelerometerAndGyroscopeCalibratorSpy.accelerometerInitialBiasY }
    }

    @Test
    fun accelerometerInitialBiasZ_getsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.accelerometerInitialBiasZ)

        // mock internal calibrator
        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val accelerometerAndGyroscopeCalibratorSpy = spy(accelerometerAndGyroscopeCalibrator)
//        val accelerometerAndGyroscopeCalibratorSpy = spyk(accelerometerAndGyroscopeCalibrator)
        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        whenever(accelerometerAndGyroscopeCalibratorSpy.accelerometerInitialBiasZ).thenReturn(value)
//        every { accelerometerAndGyroscopeCalibratorSpy.accelerometerInitialBiasZ }.returns(value)
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibratorSpy
        )

        // check
        assertEquals(value, calibrator.accelerometerInitialBiasZ)

        verify(accelerometerAndGyroscopeCalibratorSpy, only()).accelerometerInitialBiasZ
//        verify(exactly = 1) { accelerometerAndGyroscopeCalibratorSpy.accelerometerInitialBiasZ }
    }

    @Test
    fun accelerometerInitialBiasXAsMeasurement_getsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.accelerometerInitialBiasXAsMeasurement)

        // mock internal calibrator
        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val accelerometerAndGyroscopeCalibratorSpy = spy(accelerometerAndGyroscopeCalibrator)
//        val accelerometerAndGyroscopeCalibratorSpy = spyk(accelerometerAndGyroscopeCalibrator)
        val value = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        whenever(accelerometerAndGyroscopeCalibratorSpy.accelerometerInitialBiasXAsMeasurement).thenReturn(value)
/*        every { accelerometerAndGyroscopeCalibratorSpy.accelerometerInitialBiasXAsMeasurement }.returns(
            value
        )*/
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibratorSpy
        )

        // check
        assertSame(value, calibrator.accelerometerInitialBiasXAsMeasurement)

        verify(accelerometerAndGyroscopeCalibratorSpy, only()).accelerometerInitialBiasXAsMeasurement
//        verify(exactly = 1) { accelerometerAndGyroscopeCalibratorSpy.accelerometerInitialBiasXAsMeasurement }
    }

    @Test
    fun getAccelerometerInitialBiasXAsMeasurement_getsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        val result = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getAccelerometerInitialBiasXAsMeasurement(result))

        // mock internal calibrator
        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val accelerometerAndGyroscopeCalibratorSpy = spy(accelerometerAndGyroscopeCalibrator)
//        val accelerometerAndGyroscopeCalibratorSpy = spyk(accelerometerAndGyroscopeCalibrator)
        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        doAnswer { invocation ->
            val a = invocation.getArgument<Acceleration>(0)
            a.value = value
            a.unit = AccelerationUnit.METERS_PER_SQUARED_SECOND
            return@doAnswer true
        }.whenever(accelerometerAndGyroscopeCalibratorSpy).getAccelerometerInitialBiasXAsMeasurement(any())
/*        every { accelerometerAndGyroscopeCalibratorSpy.getAccelerometerInitialBiasXAsMeasurement(any()) }.answers { answer ->
            val a = answer.invocation.args[0] as Acceleration
            a.value = value
            a.unit = AccelerationUnit.METERS_PER_SQUARED_SECOND
            return@answers true
        }*/
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibratorSpy
        )

        // check
        assertTrue(calibrator.getAccelerometerInitialBiasXAsMeasurement(result))
        assertEquals(value, result.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, result.unit)

        verify(accelerometerAndGyroscopeCalibratorSpy, only()).getAccelerometerInitialBiasXAsMeasurement(
            result
        )
/*        verify(exactly = 1) {
            accelerometerAndGyroscopeCalibratorSpy.getAccelerometerInitialBiasXAsMeasurement(
                result
            )
        }*/
    }

    @Test
    fun accelerometerInitialBiasYAsMeasurement_getsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.accelerometerInitialBiasYAsMeasurement)

        // mock internal calibrator
        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val accelerometerAndGyroscopeCalibratorSpy = spy(accelerometerAndGyroscopeCalibrator)
//        val accelerometerAndGyroscopeCalibratorSpy = spyk(accelerometerAndGyroscopeCalibrator)
        val value = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        whenever(accelerometerAndGyroscopeCalibratorSpy.accelerometerInitialBiasYAsMeasurement).thenReturn(value)
/*        every { accelerometerAndGyroscopeCalibratorSpy.accelerometerInitialBiasYAsMeasurement }.returns(
            value
        )*/
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibratorSpy
        )

        // check
        assertSame(value, calibrator.accelerometerInitialBiasYAsMeasurement)

        verify(accelerometerAndGyroscopeCalibratorSpy, only()).accelerometerInitialBiasYAsMeasurement
//        verify(exactly = 1) { accelerometerAndGyroscopeCalibratorSpy.accelerometerInitialBiasYAsMeasurement }
    }

    @Test
    fun getAccelerometerInitialBiasYAsMeasurement_getsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        val result = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getAccelerometerInitialBiasYAsMeasurement(result))

        // mock internal calibrator
        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val accelerometerAndGyroscopeCalibratorSpy = spy(accelerometerAndGyroscopeCalibrator)
//        val accelerometerAndGyroscopeCalibratorSpy = spyk(accelerometerAndGyroscopeCalibrator)
        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        doAnswer { invocation ->
            val a = invocation.getArgument<Acceleration>(0)
            a.value = value
            a.unit = AccelerationUnit.METERS_PER_SQUARED_SECOND
            return@doAnswer true
        }.whenever(accelerometerAndGyroscopeCalibratorSpy).getAccelerometerInitialBiasYAsMeasurement(any())
/*        every { accelerometerAndGyroscopeCalibratorSpy.getAccelerometerInitialBiasYAsMeasurement(any()) }.answers { answer ->
            val a = answer.invocation.args[0] as Acceleration
            a.value = value
            a.unit = AccelerationUnit.METERS_PER_SQUARED_SECOND
            return@answers true
        }*/
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibratorSpy
        )

        // check
        assertTrue(calibrator.getAccelerometerInitialBiasYAsMeasurement(result))
        assertEquals(value, result.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, result.unit)

        verify(accelerometerAndGyroscopeCalibratorSpy, only()).getAccelerometerInitialBiasYAsMeasurement(
            result
        )
/*        verify(exactly = 1) {
            accelerometerAndGyroscopeCalibratorSpy.getAccelerometerInitialBiasYAsMeasurement(
                result
            )
        }*/
    }

    @Test
    fun accelerometerInitialBiasZAsMeasurement_getsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.accelerometerInitialBiasZAsMeasurement)

        // mock internal calibrator
        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val accelerometerAndGyroscopeCalibratorSpy = spy(accelerometerAndGyroscopeCalibrator)
//        val accelerometerAndGyroscopeCalibratorSpy = spyk(accelerometerAndGyroscopeCalibrator)
        val value = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        whenever(accelerometerAndGyroscopeCalibratorSpy.accelerometerInitialBiasZAsMeasurement).thenReturn(value)
/*        every { accelerometerAndGyroscopeCalibratorSpy.accelerometerInitialBiasZAsMeasurement }.returns(
            value
        )*/
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibratorSpy
        )

        // check
        assertSame(value, calibrator.accelerometerInitialBiasZAsMeasurement)

        verify(accelerometerAndGyroscopeCalibratorSpy, only()).accelerometerInitialBiasZAsMeasurement
//        verify(exactly = 1) { accelerometerAndGyroscopeCalibratorSpy.accelerometerInitialBiasZAsMeasurement }
    }

    @Test
    fun getAccelerometerInitialBiasZAsMeasurement_getsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        val result = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getAccelerometerInitialBiasZAsMeasurement(result))

        // mock internal calibrator
        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val accelerometerAndGyroscopeCalibratorSpy = spy(accelerometerAndGyroscopeCalibrator)
//        val accelerometerAndGyroscopeCalibratorSpy = spyk(accelerometerAndGyroscopeCalibrator)
        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        doAnswer { invocation ->
            val a = invocation.getArgument<Acceleration>(0)
            a.value = value
            a.unit = AccelerationUnit.METERS_PER_SQUARED_SECOND
            return@doAnswer true
        }.whenever(accelerometerAndGyroscopeCalibratorSpy).getAccelerometerInitialBiasZAsMeasurement(any())
/*        every { accelerometerAndGyroscopeCalibratorSpy.getAccelerometerInitialBiasZAsMeasurement(any()) }.answers { answer ->
            val a = answer.invocation.args[0] as Acceleration
            a.value = value
            a.unit = AccelerationUnit.METERS_PER_SQUARED_SECOND
            return@answers true
        }*/
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibratorSpy
        )

        // check
        assertTrue(calibrator.getAccelerometerInitialBiasZAsMeasurement(result))
        assertEquals(value, result.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, result.unit)

        verify(accelerometerAndGyroscopeCalibratorSpy, only()).getAccelerometerInitialBiasZAsMeasurement(
            result
        )
/*        verify(exactly = 1) {
            accelerometerAndGyroscopeCalibratorSpy.getAccelerometerInitialBiasZAsMeasurement(
                result
            )
        }*/
    }

    @Test
    fun accelerometerInitialBiasAsTriad_getsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.accelerometerInitialBiasAsTriad)

        // mock internal calibrator
        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val accelerometerAndGyroscopeCalibratorSpy = spy(accelerometerAndGyroscopeCalibrator)
//        val accelerometerAndGyroscopeCalibratorSpy = spyk(accelerometerAndGyroscopeCalibrator)
        val value = AccelerationTriad()
        whenever(accelerometerAndGyroscopeCalibratorSpy.accelerometerInitialBiasAsTriad).thenReturn(value)
/*        every { accelerometerAndGyroscopeCalibratorSpy.accelerometerInitialBiasAsTriad }.returns(
            value
        )*/
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibratorSpy
        )

        // check
        assertSame(value, calibrator.accelerometerInitialBiasAsTriad)

        verify(accelerometerAndGyroscopeCalibratorSpy, only()).accelerometerInitialBiasAsTriad
//        verify(exactly = 1) { accelerometerAndGyroscopeCalibratorSpy.accelerometerInitialBiasAsTriad }
    }

    @Test
    fun getAccelerometerInitialBiasAsTriad_getsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        val result = AccelerationTriad()
        assertFalse(calibrator.getAccelerometerInitialBiasAsTriad(result))

        // mock internal calibrator
        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val accelerometerAndGyroscopeCalibratorSpy = spy(accelerometerAndGyroscopeCalibrator)
//        val accelerometerAndGyroscopeCalibratorSpy = spyk(accelerometerAndGyroscopeCalibrator)
        val randomizer = UniformRandomizer()
        val valueX = randomizer.nextDouble()
        val valueY = randomizer.nextDouble()
        val valueZ = randomizer.nextDouble()
        doAnswer { invocation ->
            val triad = invocation.getArgument<AccelerationTriad>(0)
            triad.setValueCoordinatesAndUnit(
                valueX,
                valueY,
                valueZ,
                AccelerationUnit.METERS_PER_SQUARED_SECOND
            )
            return@doAnswer true
        }.whenever(accelerometerAndGyroscopeCalibratorSpy).getAccelerometerInitialBiasAsTriad(any())
/*        every { accelerometerAndGyroscopeCalibratorSpy.getAccelerometerInitialBiasAsTriad(any()) }.answers { answer ->
            val triad = answer.invocation.args[0] as AccelerationTriad
            triad.setValueCoordinatesAndUnit(
                valueX,
                valueY,
                valueZ,
                AccelerationUnit.METERS_PER_SQUARED_SECOND
            )
            return@answers true
        }*/
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibratorSpy
        )

        // check
        assertTrue(calibrator.getAccelerometerInitialBiasAsTriad(result))
        assertEquals(valueX, result.valueX, 0.0)
        assertEquals(valueY, result.valueY, 0.0)
        assertEquals(valueZ, result.valueZ, 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, result.unit)

        verify(accelerometerAndGyroscopeCalibratorSpy, only()).getAccelerometerInitialBiasAsTriad(
            result
        )
/*        verify(exactly = 1) {
            accelerometerAndGyroscopeCalibratorSpy.getAccelerometerInitialBiasAsTriad(
                result
            )
        }*/
    }

    @Test(expected = IllegalStateException::class)
    fun start_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            accelerometerAndGyroscopeCalibrator,
            "running",
            true
        )
        setPrivateProperty(
            SingleSensorStaticIntervalCalibrator::class,
            magnetometerCalibrator,
            "running",
            true
        )
        assertTrue(calibrator.running)

        calibrator.start()
    }

    @Test
    fun start_whenNotRunning_startsInternalCalibratorsAndResets() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibrator
        )
        calibrator.setPrivateProperty("magnetometerCalibrator", magnetometerCalibrator)
        calibrator.setPrivateProperty("accelerometerAndGyroscopeInitializationStarted", true)
        calibrator.setPrivateProperty("magnetometerInitializationStarted", true)
        calibrator.setPrivateProperty("accelerometerAndGyroscopeInitializationCompleted", true)
        calibrator.setPrivateProperty("magnetometerInitializationCompleted", true)
        calibrator.setPrivateProperty("accelerometerAndGyroscopeReadyToSolveCalibration", true)
        calibrator.setPrivateProperty("magnetometerReadyToSolveCalibration", true)
        calibrator.setPrivateProperty("accelerometerAndGyroscopeCalibrationSolvingStarted", true)
        calibrator.setPrivateProperty("magnetometerCalibrationSolvingStarted", true)
        calibrator.setPrivateProperty("accelerometerAndGyroscopeCalibrationCompleted", true)
        calibrator.setPrivateProperty("magnetometerCalibrationCompleted", true)
        calibrator.setPrivateProperty("accelerometerAndGyroscopeStopped", true)
        calibrator.setPrivateProperty("magnetometerStopped", true)

        val accelerometerAndGyroscopeInitializationStarted1: Boolean? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeInitializationStarted")
        requireNotNull(accelerometerAndGyroscopeInitializationStarted1)
        assertTrue(accelerometerAndGyroscopeInitializationStarted1)
        val magnetometerInitializationStarted1: Boolean? =
            calibrator.getPrivateProperty("magnetometerInitializationStarted")
        requireNotNull(magnetometerInitializationStarted1)
        assertTrue(magnetometerInitializationStarted1)
        val accelerometerAndGyroscopeInitializationCompleted1: Boolean? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeInitializationCompleted")
        requireNotNull(accelerometerAndGyroscopeInitializationCompleted1)
        assertTrue(accelerometerAndGyroscopeInitializationCompleted1)
        val magnetometerInitializationCompleted1: Boolean? =
            calibrator.getPrivateProperty("magnetometerInitializationCompleted")
        requireNotNull(magnetometerInitializationCompleted1)
        assertTrue(magnetometerInitializationCompleted1)
        val accelerometerAndGyroscopeReadyToSolveCalibration1: Boolean? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeReadyToSolveCalibration")
        requireNotNull(accelerometerAndGyroscopeReadyToSolveCalibration1)
        assertTrue(accelerometerAndGyroscopeReadyToSolveCalibration1)
        val magnetometerReadyToSolveCalibration1: Boolean? =
            calibrator.getPrivateProperty("magnetometerReadyToSolveCalibration")
        requireNotNull(magnetometerReadyToSolveCalibration1)
        assertTrue(magnetometerReadyToSolveCalibration1)
        val accelerometerAndGyroscopeCalibrationSolvingStarted1: Boolean? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrationSolvingStarted")
        requireNotNull(accelerometerAndGyroscopeCalibrationSolvingStarted1)
        assertTrue(accelerometerAndGyroscopeCalibrationSolvingStarted1)
        val magnetometerCalibrationSolvingStarted1: Boolean? =
            calibrator.getPrivateProperty("magnetometerCalibrationSolvingStarted")
        requireNotNull(magnetometerCalibrationSolvingStarted1)
        assertTrue(magnetometerCalibrationSolvingStarted1)
        val accelerometerAndGyroscopeCalibrationCompleted1: Boolean? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrationCompleted")
        requireNotNull(accelerometerAndGyroscopeCalibrationCompleted1)
        assertTrue(accelerometerAndGyroscopeCalibrationCompleted1)
        val magnetometerCalibrationCompleted1: Boolean? =
            calibrator.getPrivateProperty("magnetometerCalibrationCompleted")
        requireNotNull(magnetometerCalibrationCompleted1)
        assertTrue(magnetometerCalibrationCompleted1)
        val accelerometerAndGyroscopeStopped1: Boolean? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeStopped")
        requireNotNull(accelerometerAndGyroscopeStopped1)
        assertTrue(accelerometerAndGyroscopeStopped1)
        val magnetometerStopped1: Boolean? =
            calibrator.getPrivateProperty("magnetometerStopped")
        requireNotNull(magnetometerStopped1)
        assertTrue(magnetometerStopped1)

        calibrator.start()

        verify(accelerometerAndGyroscopeCalibrator, times(1)).start()
        verify(magnetometerCalibrator, times(1)).start()
//        verify(exactly = 1) { accelerometerAndGyroscopeCalibrator.start() }
//        verify(exactly = 1) { magnetometerCalibrator.start() }

        // check
        val accelerometerAndGyroscopeInitializationStarted2: Boolean? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeInitializationStarted")
        requireNotNull(accelerometerAndGyroscopeInitializationStarted2)
        assertFalse(accelerometerAndGyroscopeInitializationStarted2)
        val magnetometerInitializationStarted2: Boolean? =
            calibrator.getPrivateProperty("magnetometerInitializationStarted")
        requireNotNull(magnetometerInitializationStarted2)
        assertFalse(magnetometerInitializationStarted2)
        val accelerometerAndGyroscopeInitializationCompleted2: Boolean? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeInitializationCompleted")
        requireNotNull(accelerometerAndGyroscopeInitializationCompleted2)
        assertFalse(accelerometerAndGyroscopeInitializationCompleted2)
        val magnetometerInitializationCompleted2: Boolean? =
            calibrator.getPrivateProperty("magnetometerInitializationCompleted")
        requireNotNull(magnetometerInitializationCompleted2)
        assertFalse(magnetometerInitializationCompleted2)
        val accelerometerAndGyroscopeReadyToSolveCalibration2: Boolean? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeReadyToSolveCalibration")
        requireNotNull(accelerometerAndGyroscopeReadyToSolveCalibration2)
        assertFalse(accelerometerAndGyroscopeReadyToSolveCalibration2)
        val magnetometerReadyToSolveCalibration2: Boolean? =
            calibrator.getPrivateProperty("magnetometerReadyToSolveCalibration")
        requireNotNull(magnetometerReadyToSolveCalibration2)
        assertFalse(magnetometerReadyToSolveCalibration2)
        val accelerometerAndGyroscopeCalibrationSolvingStarted2: Boolean? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrationSolvingStarted")
        requireNotNull(accelerometerAndGyroscopeCalibrationSolvingStarted2)
        assertFalse(accelerometerAndGyroscopeCalibrationSolvingStarted2)
        val magnetometerCalibrationSolvingStarted2: Boolean? =
            calibrator.getPrivateProperty("magnetometerCalibrationSolvingStarted")
        requireNotNull(magnetometerCalibrationSolvingStarted2)
        assertFalse(magnetometerCalibrationSolvingStarted2)
        val accelerometerAndGyroscopeCalibrationCompleted2: Boolean? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrationCompleted")
        requireNotNull(accelerometerAndGyroscopeCalibrationCompleted2)
        assertFalse(accelerometerAndGyroscopeCalibrationCompleted2)
        val magnetometerCalibrationCompleted2: Boolean? =
            calibrator.getPrivateProperty("magnetometerCalibrationCompleted")
        requireNotNull(magnetometerCalibrationCompleted2)
        assertFalse(magnetometerCalibrationCompleted2)
        val accelerometerAndGyroscopeStopped2: Boolean? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeStopped")
        requireNotNull(accelerometerAndGyroscopeStopped2)
        assertFalse(accelerometerAndGyroscopeStopped2)
        val magnetometerStopped2: Boolean? =
            calibrator.getPrivateProperty("magnetometerStopped")
        requireNotNull(magnetometerStopped2)
        assertFalse(magnetometerStopped2)
    }

    @Test
    fun stop_stopsInternalCalibrators() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibrator
        )
        calibrator.setPrivateProperty("magnetometerCalibrator", magnetometerCalibrator)

        calibrator.stop()

        verify(accelerometerAndGyroscopeCalibrator, only()).stop()
        verify(magnetometerCalibrator, only()).stop()
//        verify(exactly = 1) { accelerometerAndGyroscopeCalibrator.stop() }
//        verify(exactly = 1) { magnetometerCalibrator.stop() }
    }

    @Test
    fun calibrate_whenAccelerometerAndGyroscopeDoesNotComplete_returnsFalse() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        doReturn(false).whenever(accelerometerAndGyroscopeCalibrator).calibrate()
//        every { accelerometerAndGyroscopeCalibrator.calibrate() }.returns(false)
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibrator
        )
        doReturn(true).whenever(magnetometerCalibrator).calibrate()
//        every { magnetometerCalibrator.calibrate() }.returns(true)
        calibrator.setPrivateProperty("magnetometerCalibrator", magnetometerCalibrator)

        assertFalse(calibrator.calibrate())

        verify(accelerometerAndGyroscopeCalibrator, only()).calibrate()
        verifyNoInteractions(magnetometerCalibrator)
//        verify(exactly = 1) { accelerometerAndGyroscopeCalibrator.calibrate() }
//        verify { magnetometerCalibrator wasNot Called }
    }

    @Test
    fun calibrate_whenAccelerometerAndGyroscopeDoesCompletesAndMagnetometerDoesNotComplete_returnsFalse() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        doReturn(true).whenever(accelerometerAndGyroscopeCalibrator).calibrate()
//        every { accelerometerAndGyroscopeCalibrator.calibrate() }.returns(true)
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibrator
        )
        doReturn(false).whenever(magnetometerCalibrator).calibrate()
//        every { magnetometerCalibrator.calibrate() }.returns(false)
        calibrator.setPrivateProperty("magnetometerCalibrator", magnetometerCalibrator)

        assertFalse(calibrator.calibrate())

        verify(accelerometerAndGyroscopeCalibrator, only()).calibrate()
        verify(magnetometerCalibrator, only()).calibrate()
//        verify(exactly = 1) { accelerometerAndGyroscopeCalibrator.calibrate() }
//        verify(exactly = 1) { magnetometerCalibrator.calibrate() }
    }

    @Test
    fun calibrate_whenAccelerometerAndGyroscopeDoesCompletesAndMagnetometerCompletes_returnsFalse() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        doReturn(true).whenever(accelerometerAndGyroscopeCalibrator).calibrate()
//        every { accelerometerAndGyroscopeCalibrator.calibrate() }.returns(true)
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibrator
        )
        doReturn(true).whenever(magnetometerCalibrator).calibrate()
//        every { magnetometerCalibrator.calibrate() }.returns(true)
        calibrator.setPrivateProperty("magnetometerCalibrator", magnetometerCalibrator)

        assertTrue(calibrator.calibrate())

        verify(accelerometerAndGyroscopeCalibrator, only()).calibrate()
        verify(magnetometerCalibrator, only()).calibrate()
//        verify(exactly = 1) { accelerometerAndGyroscopeCalibrator.calibrate() }
//        verify(exactly = 1) { magnetometerCalibrator.calibrate() }
    }

    @Test
    fun estimatedAccelerometerBiasX_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.estimatedAccelerometerBiasX)

        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        whenever(accelerometerAndGyroscopeCalibrator.estimatedAccelerometerBiasX).thenReturn(value)
//        every { accelerometerAndGyroscopeCalibrator.estimatedAccelerometerBiasX }.returns(value)
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibrator
        )

        assertEquals(value, calibrator.estimatedAccelerometerBiasX)

        verify(accelerometerAndGyroscopeCalibrator, only()).estimatedAccelerometerBiasX
//        verify(exactly = 1) { accelerometerAndGyroscopeCalibrator.estimatedAccelerometerBiasX }
    }

    @Test
    fun estimatedAccelerometerBiasY_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.estimatedAccelerometerBiasY)

        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        whenever(accelerometerAndGyroscopeCalibrator.estimatedAccelerometerBiasY).thenReturn(value)
//        every { accelerometerAndGyroscopeCalibrator.estimatedAccelerometerBiasY }.returns(value)
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibrator
        )

        assertEquals(value, calibrator.estimatedAccelerometerBiasY)

        verify(accelerometerAndGyroscopeCalibrator, only()).estimatedAccelerometerBiasY
//        verify(exactly = 1) { accelerometerAndGyroscopeCalibrator.estimatedAccelerometerBiasY }
    }

    @Test
    fun estimatedAccelerometerBiasZ_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.estimatedAccelerometerBiasZ)

        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        whenever(accelerometerAndGyroscopeCalibrator.estimatedAccelerometerBiasZ).thenReturn(value)
//        every { accelerometerAndGyroscopeCalibrator.estimatedAccelerometerBiasZ }.returns(value)
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibrator
        )

        assertEquals(value, calibrator.estimatedAccelerometerBiasZ)

        verify(accelerometerAndGyroscopeCalibrator, only()).estimatedAccelerometerBiasZ
//        verify(exactly = 1) { accelerometerAndGyroscopeCalibrator.estimatedAccelerometerBiasZ }
    }

    @Test
    fun estimatedAccelerometerBiasXAsMeasurement_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.estimatedAccelerometerBiasXAsMeasurement)

        val value = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        whenever(accelerometerAndGyroscopeCalibrator.estimatedAccelerometerBiasXAsMeasurement).thenReturn(value)
/*        every { accelerometerAndGyroscopeCalibrator.estimatedAccelerometerBiasXAsMeasurement }.returns(
            value
        )*/
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibrator
        )

        assertSame(value, calibrator.estimatedAccelerometerBiasXAsMeasurement)

        verify(accelerometerAndGyroscopeCalibrator, only()).estimatedAccelerometerBiasXAsMeasurement
//        verify(exactly = 1) { accelerometerAndGyroscopeCalibrator.estimatedAccelerometerBiasXAsMeasurement }
    }

    @Test
    fun getEstimatedAccelerometerBiasXAsMeasurement_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        val result = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getEstimatedAccelerometerBiasXAsMeasurement(result))

        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        doAnswer { invocation ->
            val a = invocation.getArgument<Acceleration>(0)
            a.value = value
            a.unit = AccelerationUnit.METERS_PER_SQUARED_SECOND
            return@doAnswer true
        }.whenever(accelerometerAndGyroscopeCalibrator).getEstimatedAccelerometerBiasXAsMeasurement(any())
/*        every { accelerometerAndGyroscopeCalibrator.getEstimatedAccelerometerBiasXAsMeasurement(any()) }.answers { answer ->
            val a = answer.invocation.args[0] as Acceleration
            a.value = value
            a.unit = AccelerationUnit.METERS_PER_SQUARED_SECOND
            return@answers true
        }*/
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibrator
        )

        assertTrue(calibrator.getEstimatedAccelerometerBiasXAsMeasurement(result))
        assertEquals(value, result.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, result.unit)

        verify(accelerometerAndGyroscopeCalibrator, only()).getEstimatedAccelerometerBiasXAsMeasurement(
            result
        )
/*        verify(exactly = 1) {
            accelerometerAndGyroscopeCalibrator.getEstimatedAccelerometerBiasXAsMeasurement(
                result
            )
        }*/
    }

    @Test
    fun estimatedAccelerometerBiasYAsMeasurement_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.estimatedAccelerometerBiasYAsMeasurement)

        val value = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        whenever(accelerometerAndGyroscopeCalibrator.estimatedAccelerometerBiasYAsMeasurement).thenReturn(value)
/*        every { accelerometerAndGyroscopeCalibrator.estimatedAccelerometerBiasYAsMeasurement }.returns(
            value
        )*/
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibrator
        )

        assertSame(value, calibrator.estimatedAccelerometerBiasYAsMeasurement)

        verify(accelerometerAndGyroscopeCalibrator, only()).estimatedAccelerometerBiasYAsMeasurement
//        verify(exactly = 1) { accelerometerAndGyroscopeCalibrator.estimatedAccelerometerBiasYAsMeasurement }
    }

    @Test
    fun getEstimatedAccelerometerBiasYAsMeasurement_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        val result = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getEstimatedAccelerometerBiasYAsMeasurement(result))

        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        doAnswer { invocation ->
            val a = invocation.getArgument<Acceleration>(0)
            a.value = value
            a.unit = AccelerationUnit.METERS_PER_SQUARED_SECOND
            return@doAnswer true
        }.whenever(accelerometerAndGyroscopeCalibrator).getEstimatedAccelerometerBiasYAsMeasurement(any())
/*        every { accelerometerAndGyroscopeCalibrator.getEstimatedAccelerometerBiasYAsMeasurement(any()) }.answers { answer ->
            val a = answer.invocation.args[0] as Acceleration
            a.value = value
            a.unit = AccelerationUnit.METERS_PER_SQUARED_SECOND
            return@answers true
        }*/
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibrator
        )

        assertTrue(calibrator.getEstimatedAccelerometerBiasYAsMeasurement(result))
        assertEquals(value, result.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, result.unit)

        verify(accelerometerAndGyroscopeCalibrator, only()).getEstimatedAccelerometerBiasYAsMeasurement(
            result
        )
/*        verify(exactly = 1) {
            accelerometerAndGyroscopeCalibrator.getEstimatedAccelerometerBiasYAsMeasurement(
                result
            )
        }*/
    }

    @Test
    fun estimatedAccelerometerBiasZAsMeasurement_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.estimatedAccelerometerBiasZAsMeasurement)

        val value = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        whenever(accelerometerAndGyroscopeCalibrator.estimatedAccelerometerBiasZAsMeasurement).thenReturn(value)
/*        every { accelerometerAndGyroscopeCalibrator.estimatedAccelerometerBiasZAsMeasurement }.returns(
            value
        )*/
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibrator
        )

        assertSame(value, calibrator.estimatedAccelerometerBiasZAsMeasurement)

        verify(accelerometerAndGyroscopeCalibrator, only()).estimatedAccelerometerBiasZAsMeasurement
//        verify(exactly = 1) { accelerometerAndGyroscopeCalibrator.estimatedAccelerometerBiasZAsMeasurement }
    }

    @Test
    fun getEstimatedAccelerometerBiasZAsMeasurement_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        val result = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getEstimatedAccelerometerBiasZAsMeasurement(result))

        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        doAnswer { invocation ->
            val a = invocation.getArgument<Acceleration>(0)
            a.value = value
            a.unit = AccelerationUnit.METERS_PER_SQUARED_SECOND
            return@doAnswer true
        }.whenever(accelerometerAndGyroscopeCalibrator).getEstimatedAccelerometerBiasZAsMeasurement(any())
/*        every { accelerometerAndGyroscopeCalibrator.getEstimatedAccelerometerBiasZAsMeasurement(any()) }.answers { answer ->
            val a = answer.invocation.args[0] as Acceleration
            a.value = value
            a.unit = AccelerationUnit.METERS_PER_SQUARED_SECOND
            return@answers true
        }*/
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibrator
        )

        assertTrue(calibrator.getEstimatedAccelerometerBiasZAsMeasurement(result))
        assertEquals(value, result.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, result.unit)

        verify(accelerometerAndGyroscopeCalibrator, only()).getEstimatedAccelerometerBiasZAsMeasurement(
            result
        )
/*        verify(exactly = 1) {
            accelerometerAndGyroscopeCalibrator.getEstimatedAccelerometerBiasZAsMeasurement(
                result
            )
        }*/
    }

    @Test
    fun estimatedAccelerometerBiasAsTriad_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.estimatedAccelerometerBiasAsTriad)

        val value = AccelerationTriad()
        whenever(accelerometerAndGyroscopeCalibrator.estimatedAccelerometerBiasAsTriad).thenReturn(value)
/*        every { accelerometerAndGyroscopeCalibrator.estimatedAccelerometerBiasAsTriad }.returns(
            value
        )*/
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibrator
        )

        assertSame(value, calibrator.estimatedAccelerometerBiasAsTriad)

        verify(accelerometerAndGyroscopeCalibrator, only()).estimatedAccelerometerBiasAsTriad
//        verify(exactly = 1) { accelerometerAndGyroscopeCalibrator.estimatedAccelerometerBiasAsTriad }
    }

    @Test
    fun getEstimatedAccelerometerBiasAsTriad_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        val result = AccelerationTriad()
        assertFalse(calibrator.getEstimatedAccelerometerBiasAsTriad(result))

        val randomizer = UniformRandomizer()
        val valueX = randomizer.nextDouble()
        val valueY = randomizer.nextDouble()
        val valueZ = randomizer.nextDouble()
        doAnswer { invocation ->
            val triad = invocation.getArgument<AccelerationTriad>(0)
            triad.setValueCoordinatesAndUnit(
                valueX,
                valueY,
                valueZ,
                AccelerationUnit.METERS_PER_SQUARED_SECOND
            )
            return@doAnswer true
        }.whenever(accelerometerAndGyroscopeCalibrator).getEstimatedAccelerometerBiasAsTriad(any())
/*        every { accelerometerAndGyroscopeCalibrator.getEstimatedAccelerometerBiasAsTriad(any()) }.answers { answer ->
            val triad = answer.invocation.args[0] as AccelerationTriad
            triad.setValueCoordinatesAndUnit(
                valueX,
                valueY,
                valueZ,
                AccelerationUnit.METERS_PER_SQUARED_SECOND
            )
            return@answers true
        }*/
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibrator
        )

        assertTrue(calibrator.getEstimatedAccelerometerBiasAsTriad(result))
        assertEquals(valueX, result.valueX, 0.0)
        assertEquals(valueY, result.valueY, 0.0)
        assertEquals(valueZ, result.valueZ, 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, result.unit)

        verify(accelerometerAndGyroscopeCalibrator, only()).getEstimatedAccelerometerBiasAsTriad(
            result
        )
/*        verify(exactly = 1) {
            accelerometerAndGyroscopeCalibrator.getEstimatedAccelerometerBiasAsTriad(
                result
            )
        }*/
    }

    @Test
    fun estimatedAccelerometerBiasStandardDeviationNorm_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        whenever(accelerometerAndGyroscopeCalibrator.estimatedAccelerometerBiasStandardDeviationNorm).thenReturn(value)
/*        every { accelerometerAndGyroscopeCalibrator.estimatedAccelerometerBiasStandardDeviationNorm }.returns(
            value
        )*/
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibrator
        )

        assertEquals(value, calibrator.estimatedAccelerometerBiasStandardDeviationNorm)

        verify(accelerometerAndGyroscopeCalibrator, only()).estimatedAccelerometerBiasStandardDeviationNorm
//        verify(exactly = 1) { accelerometerAndGyroscopeCalibrator.estimatedAccelerometerBiasStandardDeviationNorm }
    }

    @Test
    fun estimatedGyroscopeBiasX_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.estimatedGyroscopeBiasX)

        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        whenever(accelerometerAndGyroscopeCalibrator.estimatedGyroscopeBiasX).thenReturn(value)
//        every { accelerometerAndGyroscopeCalibrator.estimatedGyroscopeBiasX }.returns(value)
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibrator
        )

        assertEquals(value, calibrator.estimatedGyroscopeBiasX)

        verify(accelerometerAndGyroscopeCalibrator, only()).estimatedGyroscopeBiasX
//        verify(exactly = 1) { accelerometerAndGyroscopeCalibrator.estimatedGyroscopeBiasX }
    }

    @Test
    fun estimatedGyroscopeBiasY_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.estimatedGyroscopeBiasY)

        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        whenever(accelerometerAndGyroscopeCalibrator.estimatedGyroscopeBiasY).thenReturn(value)
//        every { accelerometerAndGyroscopeCalibrator.estimatedGyroscopeBiasY }.returns(value)
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibrator
        )

        assertEquals(value, calibrator.estimatedGyroscopeBiasY)

        verify(accelerometerAndGyroscopeCalibrator, only()).estimatedGyroscopeBiasY
//        verify(exactly = 1) { accelerometerAndGyroscopeCalibrator.estimatedGyroscopeBiasY }
    }

    @Test
    fun estimatedGyroscopeBiasZ_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.estimatedGyroscopeBiasZ)

        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        whenever(accelerometerAndGyroscopeCalibrator.estimatedGyroscopeBiasZ).thenReturn(value)
//        every { accelerometerAndGyroscopeCalibrator.estimatedGyroscopeBiasZ }.returns(value)
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibrator
        )

        assertEquals(value, calibrator.estimatedGyroscopeBiasZ)

        verify(accelerometerAndGyroscopeCalibrator, only()).estimatedGyroscopeBiasZ
//        verify(exactly = 1) { accelerometerAndGyroscopeCalibrator.estimatedGyroscopeBiasZ }
    }

    @Test
    fun estimatedGyroscopeBiasXAsMeasurement_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.estimatedGyroscopeBiasXAsMeasurement)

        val value = AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND)
        whenever(accelerometerAndGyroscopeCalibrator.estimatedGyroscopeBiasXAsMeasurement).thenReturn(value)
/*        every { accelerometerAndGyroscopeCalibrator.estimatedGyroscopeBiasXAsMeasurement }.returns(
            value
        )*/
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibrator
        )

        assertSame(value, calibrator.estimatedGyroscopeBiasXAsMeasurement)

        verify(accelerometerAndGyroscopeCalibrator, only()).estimatedGyroscopeBiasXAsMeasurement
//        verify(exactly = 1) { accelerometerAndGyroscopeCalibrator.estimatedGyroscopeBiasXAsMeasurement }
    }

    @Test
    fun getEstimatedGyroscopeBiasXAsMeasurement_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        val result = AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND)
        assertFalse(calibrator.getEstimatedGyroscopeBiasXAsMeasurement(result))

        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        doAnswer { invocation ->
            val w = invocation.getArgument<AngularSpeed>(0)
            w.value = value
            w.unit = AngularSpeedUnit.RADIANS_PER_SECOND
            return@doAnswer true
        }.whenever(accelerometerAndGyroscopeCalibrator).getEstimatedGyroscopeBiasXAsMeasurement(any())
/*        every { accelerometerAndGyroscopeCalibrator.getEstimatedGyroscopeBiasXAsMeasurement(any()) }.answers { answer ->
            val w = answer.invocation.args[0] as AngularSpeed
            w.value = value
            w.unit = AngularSpeedUnit.RADIANS_PER_SECOND
            return@answers true
        }*/
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibrator
        )

        assertTrue(calibrator.getEstimatedGyroscopeBiasXAsMeasurement(result))
        assertEquals(value, result.value.toDouble(), 0.0)
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, result.unit)

        verify(accelerometerAndGyroscopeCalibrator, only()).getEstimatedGyroscopeBiasXAsMeasurement(
            result
        )
/*        verify(exactly = 1) {
            accelerometerAndGyroscopeCalibrator.getEstimatedGyroscopeBiasXAsMeasurement(
                result
            )
        }*/
    }

    @Test
    fun estimatedGyroscopeBiasYAsMeasurement_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.estimatedGyroscopeBiasYAsMeasurement)

        val value = AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND)
        whenever(accelerometerAndGyroscopeCalibrator.estimatedGyroscopeBiasYAsMeasurement).thenReturn(value)
/*        every { accelerometerAndGyroscopeCalibrator.estimatedGyroscopeBiasYAsMeasurement }.returns(
            value
        )*/
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibrator
        )

        assertSame(value, calibrator.estimatedGyroscopeBiasYAsMeasurement)

        verify(accelerometerAndGyroscopeCalibrator, only()).estimatedGyroscopeBiasYAsMeasurement
//        verify(exactly = 1) { accelerometerAndGyroscopeCalibrator.estimatedGyroscopeBiasYAsMeasurement }
    }

    @Test
    fun getEstimatedGyroscopeBiasYAsMeasurement_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        val result = AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND)
        assertFalse(calibrator.getEstimatedGyroscopeBiasYAsMeasurement(result))

        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        doAnswer { invocation ->
            val w = invocation.getArgument<AngularSpeed>(0)
            w.value = value
            w.unit = AngularSpeedUnit.RADIANS_PER_SECOND
            return@doAnswer true
        }.whenever(accelerometerAndGyroscopeCalibrator).getEstimatedGyroscopeBiasYAsMeasurement(any())
/*        every { accelerometerAndGyroscopeCalibrator.getEstimatedGyroscopeBiasYAsMeasurement(any()) }.answers { answer ->
            val w = answer.invocation.args[0] as AngularSpeed
            w.value = value
            w.unit = AngularSpeedUnit.RADIANS_PER_SECOND
            return@answers true
        }*/
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibrator
        )

        assertTrue(calibrator.getEstimatedGyroscopeBiasYAsMeasurement(result))
        assertEquals(value, result.value.toDouble(), 0.0)
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, result.unit)

        verify(accelerometerAndGyroscopeCalibrator, only()).getEstimatedGyroscopeBiasYAsMeasurement(
            result
        )
/*        verify(exactly = 1) {
            accelerometerAndGyroscopeCalibrator.getEstimatedGyroscopeBiasYAsMeasurement(
                result
            )
        }*/
    }

    @Test
    fun estimatedGyroscopeBiasZAsMeasurement_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.estimatedGyroscopeBiasZAsMeasurement)

        val value = AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND)
        whenever(accelerometerAndGyroscopeCalibrator.estimatedGyroscopeBiasZAsMeasurement).thenReturn(value)
/*        every { accelerometerAndGyroscopeCalibrator.estimatedGyroscopeBiasZAsMeasurement }.returns(
            value
        )*/
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibrator
        )

        assertSame(value, calibrator.estimatedGyroscopeBiasZAsMeasurement)

        verify(accelerometerAndGyroscopeCalibrator, only()).estimatedGyroscopeBiasZAsMeasurement
//        verify(exactly = 1) { accelerometerAndGyroscopeCalibrator.estimatedGyroscopeBiasZAsMeasurement }
    }

    @Test
    fun getEstimatedGyroscopeBiasZAsMeasurement_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        val result = AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND)
        assertFalse(calibrator.getEstimatedGyroscopeBiasZAsMeasurement(result))

        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        doAnswer { invocation ->
            val w = invocation.getArgument<AngularSpeed>(0)
            w.value = value
            w.unit = AngularSpeedUnit.RADIANS_PER_SECOND
            return@doAnswer true
        }.whenever(accelerometerAndGyroscopeCalibrator).getEstimatedGyroscopeBiasZAsMeasurement(any())
/*        every { accelerometerAndGyroscopeCalibrator.getEstimatedGyroscopeBiasZAsMeasurement(any()) }.answers { answer ->
            val w = answer.invocation.args[0] as AngularSpeed
            w.value = value
            w.unit = AngularSpeedUnit.RADIANS_PER_SECOND
            return@answers true
        }*/
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibrator
        )

        assertTrue(calibrator.getEstimatedGyroscopeBiasZAsMeasurement(result))
        assertEquals(value, result.value.toDouble(), 0.0)
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, result.unit)

        verify(accelerometerAndGyroscopeCalibrator, only()).getEstimatedGyroscopeBiasZAsMeasurement(
            result
        )
/*        verify(exactly = 1) {
            accelerometerAndGyroscopeCalibrator.getEstimatedGyroscopeBiasZAsMeasurement(
                result
            )
        }*/
    }

    @Test
    fun estimatedGyroscopeBiasAsTriad_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.estimatedGyroscopeBiasAsTriad)

        val value = AngularSpeedTriad()
        whenever(accelerometerAndGyroscopeCalibrator.estimatedGyroscopeBiasAsTriad).thenReturn(value)
/*        every { accelerometerAndGyroscopeCalibrator.estimatedGyroscopeBiasAsTriad }.returns(
            value
        )*/
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibrator
        )

        assertSame(value, calibrator.estimatedGyroscopeBiasAsTriad)

        verify(accelerometerAndGyroscopeCalibrator, only()).estimatedGyroscopeBiasAsTriad
//        verify(exactly = 1) { accelerometerAndGyroscopeCalibrator.estimatedGyroscopeBiasAsTriad }
    }

    @Test
    fun getEstimatedGyroscopeBiasAsTriad_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        val result = AngularSpeedTriad()
        assertFalse(calibrator.getEstimatedGyroscopeBiasAsTriad(result))

        val randomizer = UniformRandomizer()
        val valueX = randomizer.nextDouble()
        val valueY = randomizer.nextDouble()
        val valueZ = randomizer.nextDouble()
        doAnswer { invocation ->
            val triad = invocation.getArgument<AngularSpeedTriad>(0)
            triad.setValueCoordinatesAndUnit(
                valueX,
                valueY,
                valueZ,
                AngularSpeedUnit.RADIANS_PER_SECOND
            )
            return@doAnswer true
        }.whenever(accelerometerAndGyroscopeCalibrator).getEstimatedGyroscopeBiasAsTriad(any())
/*        every { accelerometerAndGyroscopeCalibrator.getEstimatedGyroscopeBiasAsTriad(any()) }.answers { answer ->
            val triad = answer.invocation.args[0] as AngularSpeedTriad
            triad.setValueCoordinatesAndUnit(
                valueX,
                valueY,
                valueZ,
                AngularSpeedUnit.RADIANS_PER_SECOND
            )
            return@answers true
        }*/
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibrator
        )

        assertTrue(calibrator.getEstimatedGyroscopeBiasAsTriad(result))
        assertEquals(valueX, result.valueX, 0.0)
        assertEquals(valueY, result.valueY, 0.0)
        assertEquals(valueZ, result.valueZ, 0.0)
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, result.unit)

        verify(accelerometerAndGyroscopeCalibrator, only()).getEstimatedGyroscopeBiasAsTriad(
            result
        )
/*        verify(exactly = 1) {
            accelerometerAndGyroscopeCalibrator.getEstimatedGyroscopeBiasAsTriad(
                result
            )
        }*/
    }

    @Test
    fun estimatedGyroscopeBiasStandardDeviationNorm_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.estimatedGyroscopeBiasStandardDeviationNorm)

        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        whenever(accelerometerAndGyroscopeCalibrator.estimatedGyroscopeBiasStandardDeviationNorm).thenReturn(value)
/*        every { accelerometerAndGyroscopeCalibrator.estimatedGyroscopeBiasStandardDeviationNorm }.returns(
            value
        )*/
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibrator
        )

        assertEquals(value, calibrator.estimatedGyroscopeBiasStandardDeviationNorm)

        verify(accelerometerAndGyroscopeCalibrator, only()).estimatedGyroscopeBiasStandardDeviationNorm
//        verify(exactly = 1) { accelerometerAndGyroscopeCalibrator.estimatedGyroscopeBiasStandardDeviationNorm }
    }

    @Test
    fun estimatedMagnetometerHardIronX_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.estimatedMagnetometerHardIronX)

        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        whenever(magnetometerCalibrator.estimatedHardIronX).thenReturn(value)
/*        every { magnetometerCalibrator.estimatedHardIronX }.returns(
            value
        )*/
        calibrator.setPrivateProperty(
            "magnetometerCalibrator",
            magnetometerCalibrator
        )

        assertEquals(value, calibrator.estimatedMagnetometerHardIronX)

        verify(magnetometerCalibrator, only()).estimatedHardIronX
//        verify(exactly = 1) { magnetometerCalibrator.estimatedHardIronX }
    }

    @Test
    fun estimatedMagnetometerHardIronY_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.estimatedMagnetometerHardIronY)

        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        whenever(magnetometerCalibrator.estimatedHardIronY).thenReturn(value)
/*        every { magnetometerCalibrator.estimatedHardIronY }.returns(
            value
        )*/
        calibrator.setPrivateProperty(
            "magnetometerCalibrator",
            magnetometerCalibrator
        )

        assertEquals(value, calibrator.estimatedMagnetometerHardIronY)

        verify(magnetometerCalibrator, only()).estimatedHardIronY
//        verify(exactly = 1) { magnetometerCalibrator.estimatedHardIronY }
    }

    @Test
    fun estimatedMagnetometerHardIronZ_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.estimatedMagnetometerHardIronZ)

        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        whenever(magnetometerCalibrator.estimatedHardIronZ).thenReturn(value)
/*        every { magnetometerCalibrator.estimatedHardIronZ }.returns(
            value
        )*/
        calibrator.setPrivateProperty(
            "magnetometerCalibrator",
            magnetometerCalibrator
        )

        assertEquals(value, calibrator.estimatedMagnetometerHardIronZ)

        verify(magnetometerCalibrator, only()).estimatedHardIronZ
//        verify(exactly = 1) { magnetometerCalibrator.estimatedHardIronZ }
    }

    @Test
    fun estimatedMagnetometerHardIronXAsMeasurement_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.estimatedMagnetometerHardIronXAsMeasurement)

        val value = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        whenever(magnetometerCalibrator.estimatedHardIronXAsMeasurement).thenReturn(value)
/*        every { magnetometerCalibrator.estimatedHardIronXAsMeasurement }.returns(
            value
        )*/
        calibrator.setPrivateProperty(
            "magnetometerCalibrator",
            magnetometerCalibrator
        )

        assertSame(value, calibrator.estimatedMagnetometerHardIronXAsMeasurement)

        verify(magnetometerCalibrator, only()).estimatedHardIronXAsMeasurement
//        verify(exactly = 1) { magnetometerCalibrator.estimatedHardIronXAsMeasurement }
    }

    @Test
    fun getEstimatedMagnetometerHardIronXAsMeasurement_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        val result = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(calibrator.getEstimatedMagnetometerHardIronXAsMeasurement(result))

        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        doAnswer { invocation ->
            val b = invocation.getArgument<MagneticFluxDensity>(0)
            b.value = value
            b.unit = MagneticFluxDensityUnit.TESLA
            return@doAnswer true
        }.whenever(magnetometerCalibrator).getEstimatedHardIronXAsMeasurement(any())
/*        every { magnetometerCalibrator.getEstimatedHardIronXAsMeasurement(any()) }.answers { answer ->
            val b = answer.invocation.args[0] as MagneticFluxDensity
            b.value = value
            b.unit = MagneticFluxDensityUnit.TESLA
            return@answers true
        }*/
        calibrator.setPrivateProperty(
            "magnetometerCalibrator",
            magnetometerCalibrator
        )

        assertTrue(calibrator.getEstimatedMagnetometerHardIronXAsMeasurement(result))
        assertEquals(value, result.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, result.unit)

        verify(magnetometerCalibrator, only()).getEstimatedHardIronXAsMeasurement(result)
//        verify(exactly = 1) { magnetometerCalibrator.getEstimatedHardIronXAsMeasurement(result) }
    }

    @Test
    fun estimatedMagnetometerHardIronYAsMeasurement_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.estimatedMagnetometerHardIronYAsMeasurement)

        val value = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        whenever(magnetometerCalibrator.estimatedHardIronYAsMeasurement).thenReturn(value)
/*        every { magnetometerCalibrator.estimatedHardIronYAsMeasurement }.returns(
            value
        )*/
        calibrator.setPrivateProperty(
            "magnetometerCalibrator",
            magnetometerCalibrator
        )

        assertSame(value, calibrator.estimatedMagnetometerHardIronYAsMeasurement)

        verify(magnetometerCalibrator, only()).estimatedHardIronYAsMeasurement
//        verify(exactly = 1) { magnetometerCalibrator.estimatedHardIronYAsMeasurement }
    }

    @Test
    fun getEstimatedMagnetometerHardIronYAsMeasurement_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        val result = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(calibrator.getEstimatedMagnetometerHardIronYAsMeasurement(result))

        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        doAnswer { invocation ->
            val b = invocation.getArgument<MagneticFluxDensity>(0)
            b.value = value
            b.unit = MagneticFluxDensityUnit.TESLA
            return@doAnswer true
        }.whenever(magnetometerCalibrator).getEstimatedHardIronYAsMeasurement(any())
/*        every { magnetometerCalibrator.getEstimatedHardIronYAsMeasurement(any()) }.answers { answer ->
            val b = answer.invocation.args[0] as MagneticFluxDensity
            b.value = value
            b.unit = MagneticFluxDensityUnit.TESLA
            return@answers true
        }*/
        calibrator.setPrivateProperty(
            "magnetometerCalibrator",
            magnetometerCalibrator
        )

        assertTrue(calibrator.getEstimatedMagnetometerHardIronYAsMeasurement(result))
        assertEquals(value, result.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, result.unit)

        verify(magnetometerCalibrator, only()).getEstimatedHardIronYAsMeasurement(result)
//        verify(exactly = 1) { magnetometerCalibrator.getEstimatedHardIronYAsMeasurement(result) }
    }

    @Test
    fun estimatedMagnetometerHardIronZAsMeasurement_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.estimatedMagnetometerHardIronZAsMeasurement)

        val value = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        whenever(magnetometerCalibrator.estimatedHardIronZAsMeasurement).thenReturn(value)
/*        every { magnetometerCalibrator.estimatedHardIronZAsMeasurement }.returns(
            value
        )*/
        calibrator.setPrivateProperty(
            "magnetometerCalibrator",
            magnetometerCalibrator
        )

        assertSame(value, calibrator.estimatedMagnetometerHardIronZAsMeasurement)

        verify(magnetometerCalibrator, only()).estimatedHardIronZAsMeasurement
//        verify(exactly = 1) { magnetometerCalibrator.estimatedHardIronZAsMeasurement }
    }

    @Test
    fun getEstimatedMagnetometerHardIronZAsMeasurement_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        val result = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(calibrator.getEstimatedMagnetometerHardIronZAsMeasurement(result))

        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        doAnswer { invocation ->
            val b = invocation.getArgument<MagneticFluxDensity>(0)
            b.value = value
            b.unit = MagneticFluxDensityUnit.TESLA
            return@doAnswer true
        }.whenever(magnetometerCalibrator).getEstimatedHardIronZAsMeasurement(any())
/*        every { magnetometerCalibrator.getEstimatedHardIronZAsMeasurement(any()) }.answers { answer ->
            val b = answer.invocation.args[0] as MagneticFluxDensity
            b.value = value
            b.unit = MagneticFluxDensityUnit.TESLA
            return@answers true
        }*/
        calibrator.setPrivateProperty(
            "magnetometerCalibrator",
            magnetometerCalibrator
        )

        assertTrue(calibrator.getEstimatedMagnetometerHardIronZAsMeasurement(result))
        assertEquals(value, result.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, result.unit)

        verify(magnetometerCalibrator, only()).getEstimatedHardIronZAsMeasurement(result)
//        verify(exactly = 1) { magnetometerCalibrator.getEstimatedHardIronZAsMeasurement(result) }
    }

    @Test
    fun estimatedMagnetometerHardIronAsTriad_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.estimatedMagnetometerHardIronAsTriad)

        val value = MagneticFluxDensityTriad()
        whenever(magnetometerCalibrator.estimatedHardIronAsTriad).thenReturn(value)
/*        every { magnetometerCalibrator.estimatedHardIronAsTriad }.returns(
            value
        )*/
        calibrator.setPrivateProperty(
            "magnetometerCalibrator",
            magnetometerCalibrator
        )

        assertSame(value, calibrator.estimatedMagnetometerHardIronAsTriad)

        verify(magnetometerCalibrator, only()).estimatedHardIronAsTriad
//        verify(exactly = 1) { magnetometerCalibrator.estimatedHardIronAsTriad }
    }

    @Test
    fun getEstimatedMagnetometerHardIronAsTriad_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        val result = MagneticFluxDensityTriad()
        assertFalse(calibrator.getEstimatedMagnetometerHardIronAsTriad(result))

        val randomizer = UniformRandomizer()
        val valueX = randomizer.nextDouble()
        val valueY = randomizer.nextDouble()
        val valueZ = randomizer.nextDouble()
        doAnswer { invocation ->
            val triad = invocation.getArgument<MagneticFluxDensityTriad>(0)
            triad.setValueCoordinatesAndUnit(
                valueX,
                valueY,
                valueZ,
                MagneticFluxDensityUnit.TESLA
            )
            return@doAnswer true
        }.whenever(magnetometerCalibrator).getEstimatedHardIronAsTriad(any())
/*        every { magnetometerCalibrator.getEstimatedHardIronAsTriad(any()) }.answers { answer ->
            val triad = answer.invocation.args[0] as MagneticFluxDensityTriad
            triad.setValueCoordinatesAndUnit(valueX, valueY, valueZ, MagneticFluxDensityUnit.TESLA)
            return@answers true
        }*/
        calibrator.setPrivateProperty(
            "magnetometerCalibrator",
            magnetometerCalibrator
        )

        assertTrue(calibrator.getEstimatedMagnetometerHardIronAsTriad(result))
        assertEquals(valueX, result.valueX, 0.0)
        assertEquals(valueY, result.valueY, 0.0)
        assertEquals(valueZ, result.valueZ, 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, result.unit)

        verify(magnetometerCalibrator, only()).getEstimatedHardIronAsTriad(result)
//        verify(exactly = 1) { magnetometerCalibrator.getEstimatedHardIronAsTriad(result) }
    }

    @Test
    fun running_whenAccelerometerAndGyroscopeCalibratorRunning_returnsTrue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertFalse(calibrator.running)

        // mock internal calibrator
        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val accelerometerAndGyroscopeCalibratorSpy = spy(accelerometerAndGyroscopeCalibrator)
        whenever(accelerometerAndGyroscopeCalibratorSpy.running).thenReturn(true)
//        val accelerometerAndGyroscopeCalibratorSpy = spyk(accelerometerAndGyroscopeCalibrator)
//        every { accelerometerAndGyroscopeCalibratorSpy.running }.returns(true)
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibratorSpy
        )
        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        val magnetometerCalibratorSpy = spy(magnetometerCalibrator)
//        val magnetometerCalibratorSpy = spyk(magnetometerCalibrator)
        calibrator.setPrivateProperty(
            "magnetometerCalibrator",
            magnetometerCalibratorSpy
        )

        // check
        assertTrue(calibrator.running)

        verify(accelerometerAndGyroscopeCalibratorSpy, only()).running
        verifyNoInteractions(magnetometerCalibratorSpy)
//        verify(exactly = 1) { accelerometerAndGyroscopeCalibratorSpy.running }
//        verify { magnetometerCalibratorSpy wasNot Called }
    }

    @Test
    fun running_whenMagnetometerCalibratorRunning_returnsTrue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertFalse(calibrator.running)

        // mock internal calibrator
        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val accelerometerAndGyroscopeCalibratorSpy = spy(accelerometerAndGyroscopeCalibrator)
//        val accelerometerAndGyroscopeCalibratorSpy = spyk(accelerometerAndGyroscopeCalibrator)
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibratorSpy
        )
        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        val magnetometerCalibratorSpy = spy(magnetometerCalibrator)
//        val magnetometerCalibratorSpy = spyk(magnetometerCalibrator)
        whenever(magnetometerCalibratorSpy.running).thenReturn(true)
//        every { magnetometerCalibratorSpy.running }.returns(true)
        calibrator.setPrivateProperty(
            "magnetometerCalibrator",
            magnetometerCalibratorSpy
        )

        // check
        assertTrue(calibrator.running)

        verify(accelerometerAndGyroscopeCalibratorSpy, only()).running
        verify(magnetometerCalibratorSpy, only()).running
//        verify(exactly = 1) { accelerometerAndGyroscopeCalibratorSpy.running }
//        verify(exactly = 1) { magnetometerCalibratorSpy.running }
    }

    @Test
    fun running_whenInternalCalibratorsNotRunning_returnsFalse() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check internal calibrators
        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        assertFalse(accelerometerAndGyroscopeCalibrator.running)
        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        assertFalse(magnetometerCalibrator.running)

        val accelerometerAndGyroscopeCalibratorSpy = spy(accelerometerAndGyroscopeCalibrator)
//        val accelerometerAndGyroscopeCalibratorSpy = spyk(accelerometerAndGyroscopeCalibrator)
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibratorSpy
        )
        val magnetometerCalibratorSpy = spy(magnetometerCalibrator)
//        val magnetometerCalibratorSpy = spyk(magnetometerCalibrator)
        calibrator.setPrivateProperty(
            "magnetometerCalibrator",
            magnetometerCalibratorSpy
        )

        // check
        assertFalse(calibrator.running)

        verify(accelerometerAndGyroscopeCalibratorSpy, only()).running
        verify(magnetometerCalibratorSpy, only()).running
//        verify(exactly = 1) { accelerometerAndGyroscopeCalibratorSpy.running }
//        verify(exactly = 1) { magnetometerCalibratorSpy.running }
    }

    @Test
    fun magnetometerBaseNoiseLevelAbsoluteThreshold_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
            calibrator.magnetometerBaseNoiseLevelAbsoluteThreshold,
            0.0
        )

        // set new value
        calibrator.magnetometerBaseNoiseLevelAbsoluteThreshold = BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD

        // check
        assertEquals(
            BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
            calibrator.magnetometerBaseNoiseLevelAbsoluteThreshold,
            0.0
        )
    }

    @Test
    fun magnetometerBaseNoiseLevelAbsoluteThresholdAsMeasurement_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        val value1 = calibrator.magnetometerBaseNoiseLevelAbsoluteThresholdAsMeasurement
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
            value1.value.toDouble(),
            0.0
        )
        assertEquals(MagneticFluxDensityUnit.TESLA, value1.unit)

        // set new value
        val value2 =
            MagneticFluxDensity(BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD, MagneticFluxDensityUnit.TESLA)
        calibrator.magnetometerBaseNoiseLevelAbsoluteThresholdAsMeasurement = value2

        // check
        val value3 = calibrator.magnetometerBaseNoiseLevelAbsoluteThresholdAsMeasurement
        assertEquals(value2, value3)
    }

    @Test
    fun getMagnetometerBaseNoiseLevelAbsoluteThresholdAsMeasurement_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        val value1 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        calibrator.getMagnetometerBaseNoiseLevelAbsoluteThresholdAsMeasurement(value1)
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
            value1.value.toDouble(),
            0.0
        )
        assertEquals(MagneticFluxDensityUnit.TESLA, value1.unit)

        // set new value
        val value2 =
            MagneticFluxDensity(BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD, MagneticFluxDensityUnit.TESLA)
        calibrator.magnetometerBaseNoiseLevelAbsoluteThresholdAsMeasurement = value2

        // check
        val value3 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        calibrator.getMagnetometerBaseNoiseLevelAbsoluteThresholdAsMeasurement(value3)
        assertEquals(value2, value3)
    }

    @Test
    fun magnetometerBaseNoiseLevelPsd_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        assertNull(calibrator.magnetometerBaseNoiseLevelPsd)

        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        val magnetometerCalibratorSpy = spy(magnetometerCalibrator)
//        val magnetometerCalibratorSpy = spyk(magnetometerCalibrator)
        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        whenever(magnetometerCalibratorSpy.baseNoiseLevelPsd).thenReturn(value)
//        every { magnetometerCalibratorSpy.baseNoiseLevelPsd }.returns(value)
        calibrator.setPrivateProperty(
            "magnetometerCalibrator",
            magnetometerCalibratorSpy
        )

        // check
        assertEquals(value, calibrator.magnetometerBaseNoiseLevelPsd)

        verify(magnetometerCalibratorSpy, times(2)).baseNoiseLevelPsd
//        verify(exactly = 1) { magnetometerCalibratorSpy.baseNoiseLevelPsd }
    }

    @Test
    fun magnetometerBaseNoiseLevelRootPsd_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        assertNull(calibrator.magnetometerBaseNoiseLevelRootPsd)

        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        val magnetometerCalibratorSpy = spy(magnetometerCalibrator)
//        val magnetometerCalibratorSpy = spyk(magnetometerCalibrator)
        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        whenever(magnetometerCalibratorSpy.baseNoiseLevelRootPsd).thenReturn(value)
//        every { magnetometerCalibratorSpy.baseNoiseLevelRootPsd }.returns(value)
        calibrator.setPrivateProperty(
            "magnetometerCalibrator",
            magnetometerCalibratorSpy
        )

        // check
        assertEquals(value, calibrator.magnetometerBaseNoiseLevelRootPsd)

        verify(magnetometerCalibratorSpy, times(2)).baseNoiseLevelRootPsd
//        verify(exactly = 1) { magnetometerCalibratorSpy.baseNoiseLevelRootPsd }
    }

    @Test
    fun magnetometerThreshold_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        assertNull(calibrator.magnetometerThreshold)

        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        val magnetometerCalibratorSpy = spy(magnetometerCalibrator)
//        val magnetometerCalibratorSpy = spyk(magnetometerCalibrator)
        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        whenever(magnetometerCalibratorSpy.threshold).thenReturn(value)
//        every { magnetometerCalibratorSpy.threshold }.returns(value)
        calibrator.setPrivateProperty(
            "magnetometerCalibrator",
            magnetometerCalibratorSpy
        )

        // check
        assertEquals(value, calibrator.magnetometerThreshold)

        verify(magnetometerCalibratorSpy, times(2)).threshold
//        verify(exactly = 1) { magnetometerCalibratorSpy.threshold }
    }

    @Test
    fun magnetometerThresholdAsMeasurement_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        assertNull(calibrator.magnetometerThresholdAsMeasurement)

        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        val magnetometerCalibratorSpy = spy(magnetometerCalibrator)
//        val magnetometerCalibratorSpy = spyk(magnetometerCalibrator)
        val value = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        whenever(magnetometerCalibratorSpy.thresholdAsMeasurement).thenReturn(value)
//        every { magnetometerCalibratorSpy.thresholdAsMeasurement }.returns(value)
        calibrator.setPrivateProperty(
            "magnetometerCalibrator",
            magnetometerCalibratorSpy
        )

        // check
        assertSame(value, calibrator.magnetometerThresholdAsMeasurement)

        verify(magnetometerCalibratorSpy, times(2)).thresholdAsMeasurement
//        verify(exactly = 1) { magnetometerCalibratorSpy.thresholdAsMeasurement }
    }

    @Test
    fun getMagnetometerThresholdAsMeasurement_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        val result = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(calibrator.getMagnetometerThresholdAsMeasurement(result))

        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        val magnetometerCalibratorSpy = spy(magnetometerCalibrator)
//        val magnetometerCalibratorSpy = spyk(magnetometerCalibrator)
        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        doAnswer { invocation ->
            val b = invocation.getArgument<MagneticFluxDensity>(0)
            b.value = value
            b.unit = MagneticFluxDensityUnit.TESLA
            return@doAnswer true
        }.whenever(magnetometerCalibratorSpy).getThresholdAsMeasurement(any())
/*        every { magnetometerCalibratorSpy.getThresholdAsMeasurement(any()) }.answers { answer ->
            val b = answer.invocation.args[0] as MagneticFluxDensity
            b.value = value
            b.unit = MagneticFluxDensityUnit.TESLA
            return@answers true
        }*/
        calibrator.setPrivateProperty(
            "magnetometerCalibrator",
            magnetometerCalibratorSpy
        )

        // check
        assertTrue(calibrator.getMagnetometerThresholdAsMeasurement(result))
        assertEquals(value, result.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, result.unit)

        verify(magnetometerCalibratorSpy, only()).getThresholdAsMeasurement(result)
//        verify(exactly = 1) { magnetometerCalibratorSpy.getThresholdAsMeasurement(result) }
    }

    @Test
    fun magnetometerAverageTimeInterval_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        assertNull(calibrator.magnetometerAverageTimeInterval)

        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        val magnetometerCalibratorSpy = spy(magnetometerCalibrator)
//        val magnetometerCalibratorSpy = spyk(magnetometerCalibrator)
        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        whenever(magnetometerCalibratorSpy.averageTimeInterval).thenReturn(value)
//        every { magnetometerCalibratorSpy.averageTimeInterval }.returns(value)
        calibrator.setPrivateProperty(
            "magnetometerCalibrator",
            magnetometerCalibratorSpy
        )

        // check
        assertEquals(value, calibrator.magnetometerAverageTimeInterval)

        verify(magnetometerCalibratorSpy, times(2)).averageTimeInterval
//        verify(exactly = 1) { magnetometerCalibratorSpy.averageTimeInterval }
    }

    @Test
    fun magnetometerAverageTimeIntervalAsTime_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        assertNull(calibrator.magnetometerAverageTimeIntervalAsTime)

        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        val magnetometerCalibratorSpy = spy(magnetometerCalibrator)
//        val magnetometerCalibratorSpy = spyk(magnetometerCalibrator)
        val value = Time(0.0, TimeUnit.SECOND)
        whenever(magnetometerCalibratorSpy.averageTimeIntervalAsTime).thenReturn(value)
//        every { magnetometerCalibratorSpy.averageTimeIntervalAsTime }.returns(value)
        calibrator.setPrivateProperty(
            "magnetometerCalibrator",
            magnetometerCalibratorSpy
        )

        // check
        assertEquals(value, calibrator.magnetometerAverageTimeIntervalAsTime)

        verify(magnetometerCalibratorSpy, times(2)).averageTimeIntervalAsTime
//        verify(exactly = 1) { magnetometerCalibratorSpy.averageTimeIntervalAsTime }
    }

    @Test
    fun getMagnetometerAverageTimeIntervalAsTime_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        val result = Time(0.0, TimeUnit.SECOND)
        assertFalse(calibrator.getMagnetometerAverageTimeIntervalAsTime(result))

        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        val magnetometerCalibratorSpy = spy(magnetometerCalibrator)
//        val magnetometerCalibratorSpy = spyk(magnetometerCalibrator)
        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        doAnswer { invocation ->
            val t = invocation.getArgument<Time>(0)
            t.value = value
            t.unit = TimeUnit.SECOND
            return@doAnswer true
        }.whenever(magnetometerCalibratorSpy).getAverageTimeIntervalAsTime(any())
/*        every { magnetometerCalibratorSpy.getAverageTimeIntervalAsTime(any()) }.answers { answer ->
            val t = answer.invocation.args[0] as Time
            t.value = value
            t.unit = TimeUnit.SECOND
            return@answers true
        }*/
        calibrator.setPrivateProperty(
            "magnetometerCalibrator",
            magnetometerCalibratorSpy
        )

        // check
        assertTrue(calibrator.getMagnetometerAverageTimeIntervalAsTime(result))
        assertEquals(value, result.value.toDouble(), 0.0)
        assertEquals(TimeUnit.SECOND, result.unit)

        verify(magnetometerCalibratorSpy, only()).getAverageTimeIntervalAsTime(result)
//        verify(exactly = 1) { magnetometerCalibratorSpy.getAverageTimeIntervalAsTime(result) }
    }

    @Test
    fun magnetometerTimeIntervalVariance_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        assertNull(calibrator.magnetometerTimeIntervalVariance)

        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        val magnetometerCalibratorSpy = spy(magnetometerCalibrator)
//        val magnetometerCalibratorSpy = spyk(magnetometerCalibrator)
        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        whenever(magnetometerCalibratorSpy.timeIntervalVariance).thenReturn(value)
//        every { magnetometerCalibratorSpy.timeIntervalVariance }.returns(value)
        calibrator.setPrivateProperty(
            "magnetometerCalibrator",
            magnetometerCalibratorSpy
        )

        // check
        assertEquals(value, calibrator.magnetometerTimeIntervalVariance)

        verify(magnetometerCalibratorSpy, times(2)).timeIntervalVariance
//        verify(exactly = 1) { magnetometerCalibratorSpy.timeIntervalVariance }
    }

    @Test
    fun magnetometerTimeIntervalStandardDeviation_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        assertNull(calibrator.magnetometerTimeIntervalStandardDeviation)

        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        val magnetometerCalibratorSpy = spy(magnetometerCalibrator)
//        val magnetometerCalibratorSpy = spyk(magnetometerCalibrator)
        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        whenever(magnetometerCalibratorSpy.timeIntervalStandardDeviation).thenReturn(value)
//        every { magnetometerCalibratorSpy.timeIntervalStandardDeviation }.returns(value)
        calibrator.setPrivateProperty(
            "magnetometerCalibrator",
            magnetometerCalibratorSpy
        )

        // check
        assertEquals(value, calibrator.magnetometerTimeIntervalStandardDeviation)

        verify(magnetometerCalibratorSpy, times(2)).timeIntervalStandardDeviation
//        verify(exactly = 1) { magnetometerCalibratorSpy.timeIntervalStandardDeviation }
    }

    @Test
    fun magnetometerTimeIntervalStandardDeviationAsTime_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        assertNull(calibrator.magnetometerTimeIntervalStandardDeviationAsTime)

        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        val magnetometerCalibratorSpy = spy(magnetometerCalibrator)
//        val magnetometerCalibratorSpy = spyk(magnetometerCalibrator)
        val value = Time(0.0, TimeUnit.SECOND)
        whenever(magnetometerCalibratorSpy.timeIntervalStandardDeviationAsTime).thenReturn(value)
//        every { magnetometerCalibratorSpy.timeIntervalStandardDeviationAsTime }.returns(value)
        calibrator.setPrivateProperty(
            "magnetometerCalibrator",
            magnetometerCalibratorSpy
        )

        // check
        assertEquals(value, calibrator.magnetometerTimeIntervalStandardDeviationAsTime)

        verify(magnetometerCalibratorSpy, times(2)).timeIntervalStandardDeviationAsTime
//        verify(exactly = 1) { magnetometerCalibratorSpy.timeIntervalStandardDeviationAsTime }
    }

    @Test
    fun getMagnetometerTimeIntervalStandardDeviationAsTime_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        val result = Time(0.0, TimeUnit.SECOND)
        assertFalse(calibrator.getMagnetometerTimeIntervalStandardDeviationAsTime(result))

        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        val magnetometerCalibratorSpy = spy(magnetometerCalibrator)
//        val magnetometerCalibratorSpy = spyk(magnetometerCalibrator)
        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        doAnswer { invocation ->
            val t = invocation.getArgument<Time>(0)
            t.value = value
            t.unit = TimeUnit.SECOND
            return@doAnswer true
        }.whenever(magnetometerCalibratorSpy).getTimeIntervalStandardDeviationAsTime(any())
/*        every { magnetometerCalibratorSpy.getTimeIntervalStandardDeviationAsTime(any()) }.answers { answer ->
            val t = answer.invocation.args[0] as Time
            t.value = value
            t.unit = TimeUnit.SECOND
            return@answers true
        }*/
        calibrator.setPrivateProperty(
            "magnetometerCalibrator",
            magnetometerCalibratorSpy
        )

        // check
        assertTrue(calibrator.getMagnetometerTimeIntervalStandardDeviationAsTime(result))
        assertEquals(value, result.value.toDouble(), 0.0)
        assertEquals(TimeUnit.SECOND, result.unit)

        verify(magnetometerCalibratorSpy, only()).getTimeIntervalStandardDeviationAsTime(
            result
        )
/*        verify(exactly = 1) {
            magnetometerCalibratorSpy.getTimeIntervalStandardDeviationAsTime(
                result
            )
        }*/
    }

    @Test
    fun initialMagneticFluxDensityNorm_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        assertNull(calibrator.initialMagneticFluxDensityNorm)

        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        val magnetometerCalibratorSpy = spy(magnetometerCalibrator)
//        val magnetometerCalibratorSpy = spyk(magnetometerCalibrator)
        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        whenever(magnetometerCalibratorSpy.initialMagneticFluxDensityNorm).thenReturn(value)
//        every { magnetometerCalibratorSpy.initialMagneticFluxDensityNorm }.returns(value)
        calibrator.setPrivateProperty(
            "magnetometerCalibrator",
            magnetometerCalibratorSpy
        )

        // check
        assertEquals(value, calibrator.initialMagneticFluxDensityNorm)

        verify(magnetometerCalibratorSpy, only()).initialMagneticFluxDensityNorm
//        verify(exactly = 1) { magnetometerCalibratorSpy.initialMagneticFluxDensityNorm }
    }

    @Test
    fun gravityNorm_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.gravityNorm)

        // mock internal calibrator
        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val accelerometerAndGyroscopeCalibratorSpy = spy(accelerometerAndGyroscopeCalibrator)
//        val accelerometerAndGyroscopeCalibratorSpy = spyk(accelerometerAndGyroscopeCalibrator)
        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        whenever(accelerometerAndGyroscopeCalibratorSpy.gravityNorm).thenReturn(value)
//        every { accelerometerAndGyroscopeCalibratorSpy.gravityNorm }.returns(value)
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibratorSpy
        )

        // check
        assertEquals(value, calibrator.gravityNorm)

        verify(accelerometerAndGyroscopeCalibratorSpy, only()).gravityNorm
//        verify(exactly = 1) { accelerometerAndGyroscopeCalibratorSpy.gravityNorm }
    }

    @Test
    fun accelerometerResultUnreliable_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertFalse(calibrator.accelerometerResultUnreliable)

        // mock internal calibrator
        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val accelerometerAndGyroscopeCalibratorSpy = spy(accelerometerAndGyroscopeCalibrator)
//        val accelerometerAndGyroscopeCalibratorSpy = spyk(accelerometerAndGyroscopeCalibrator)
        val randomizer = UniformRandomizer()
        val value = randomizer.nextBoolean()
        whenever(accelerometerAndGyroscopeCalibratorSpy.accelerometerResultUnreliable).thenReturn(value)
//        every { accelerometerAndGyroscopeCalibratorSpy.accelerometerResultUnreliable }.returns(value)
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibratorSpy
        )

        // check
        assertEquals(value, calibrator.accelerometerResultUnreliable)

        verify(accelerometerAndGyroscopeCalibratorSpy, only()).accelerometerResultUnreliable
//        verify(exactly = 1) { accelerometerAndGyroscopeCalibratorSpy.accelerometerResultUnreliable }
    }

    @Test
    fun isGravityNormEstimated_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertTrue(calibrator.isGravityNormEstimated)

        // mock internal calibrator
        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val accelerometerAndGyroscopeCalibratorSpy = spy(accelerometerAndGyroscopeCalibrator)
//        val accelerometerAndGyroscopeCalibratorSpy = spyk(accelerometerAndGyroscopeCalibrator)
        val randomizer = UniformRandomizer()
        val value = randomizer.nextBoolean()
        whenever(accelerometerAndGyroscopeCalibratorSpy.isGravityNormEstimated).thenReturn(value)
//        every { accelerometerAndGyroscopeCalibratorSpy.isGravityNormEstimated }.returns(value)
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibratorSpy
        )

        // check
        assertEquals(value, calibrator.isGravityNormEstimated)

        verify(accelerometerAndGyroscopeCalibratorSpy, only()).isGravityNormEstimated
//        verify(exactly = 1) { accelerometerAndGyroscopeCalibratorSpy.isGravityNormEstimated }
    }

    @Test
    fun accelerometerSensor_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.accelerometerSensor)

        // mock internal calibrator
        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val accelerometerAndGyroscopeCalibratorSpy = spy(accelerometerAndGyroscopeCalibrator)
        whenever(accelerometerAndGyroscopeCalibratorSpy.accelerometerSensor).thenReturn(sensor)
//        val accelerometerAndGyroscopeCalibratorSpy = spyk(accelerometerAndGyroscopeCalibrator)
//        every { accelerometerAndGyroscopeCalibratorSpy.accelerometerSensor }.returns(sensor)
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibratorSpy
        )

        // check
        assertSame(sensor, calibrator.accelerometerSensor)

        verify(accelerometerAndGyroscopeCalibratorSpy, times(2)).accelerometerSensor
//        verify(exactly = 1) { accelerometerAndGyroscopeCalibratorSpy.accelerometerSensor }
    }

    @Test
    fun gyroscopeSensor_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.gyroscopeSensor)

        // mock internal calibrator
        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val accelerometerAndGyroscopeCalibratorSpy = spy(accelerometerAndGyroscopeCalibrator)
        whenever(accelerometerAndGyroscopeCalibratorSpy.gyroscopeSensor).thenReturn(sensor)
//        val accelerometerAndGyroscopeCalibratorSpy = spyk(accelerometerAndGyroscopeCalibrator)
//        every { accelerometerAndGyroscopeCalibratorSpy.gyroscopeSensor }.returns(sensor)
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibratorSpy
        )

        // check
        assertSame(sensor, calibrator.gyroscopeSensor)

        verify(accelerometerAndGyroscopeCalibratorSpy, times(2)).gyroscopeSensor
//        verify(exactly = 1) { accelerometerAndGyroscopeCalibratorSpy.gyroscopeSensor }
    }

    @Test
    fun magnetometerSensor_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.magnetometerSensor)

        // mock internal calibrator
        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        val magnetometerCalibratorSpy = spy(magnetometerCalibrator)
        whenever(magnetometerCalibratorSpy.magnetometerSensor).thenReturn(sensor)
//        val magnetometerCalibratorSpy = spyk(magnetometerCalibrator)
//        every { magnetometerCalibratorSpy.magnetometerSensor }.returns(sensor)
        calibrator.setPrivateProperty(
            "magnetometerCalibrator",
            magnetometerCalibratorSpy
        )

        // check
        assertSame(sensor, calibrator.magnetometerSensor)

        verify(magnetometerCalibratorSpy, times(2)).magnetometerSensor
//        verify(exactly = 1) { magnetometerCalibratorSpy.magnetometerSensor }
    }

    @Test
    fun gravitySensor_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.gravitySensor)

        // mock internal calibrator
        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val accelerometerAndGyroscopeCalibratorSpy = spy(accelerometerAndGyroscopeCalibrator)
//        val accelerometerAndGyroscopeCalibratorSpy = spyk(accelerometerAndGyroscopeCalibrator)
        whenever(accelerometerAndGyroscopeCalibratorSpy.gravitySensor).thenReturn(sensor)
//        every { accelerometerAndGyroscopeCalibratorSpy.gravitySensor }.returns(sensor)
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibratorSpy
        )

        // check
        assertSame(sensor, calibrator.gravitySensor)

        verify(accelerometerAndGyroscopeCalibratorSpy, only()).gravitySensor
//        verify(exactly = 1) { accelerometerAndGyroscopeCalibratorSpy.gravitySensor }
    }

    @Test
    fun minimumRequiredMeasurements_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        val minimumRequiredAccelerometerMeasurements =
            calibrator.minimumRequiredAccelerometerMeasurements
        val minimumRequiredGyroscopeMeasurements = calibrator.minimumRequiredGyroscopeMeasurements
        val minimumRequiredMagnetometerMeasurements =
            calibrator.minimumRequiredMagnetometerMeasurements
        val minimumRequiredMeasurements = max(
            max(
                minimumRequiredAccelerometerMeasurements,
                minimumRequiredGyroscopeMeasurements
            ), minimumRequiredMagnetometerMeasurements
        )
        assertEquals(minimumRequiredMeasurements, calibrator.minimumRequiredMeasurements)
    }

    @Test
    fun estimatedAccelerometerMa_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.estimatedAccelerometerMa)

        // mock internal calibrator
        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val accelerometerAndGyroscopeCalibratorSpy = spy(accelerometerAndGyroscopeCalibrator)
        whenever(accelerometerAndGyroscopeCalibratorSpy.estimatedAccelerometerMa).thenReturn(matrix)
//        val accelerometerAndGyroscopeCalibratorSpy = spyk(accelerometerAndGyroscopeCalibrator)
/*        every { accelerometerAndGyroscopeCalibratorSpy.estimatedAccelerometerMa }.returns(
            matrix
        )*/
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibratorSpy
        )

        // check
        assertEquals(matrix, calibrator.estimatedAccelerometerMa)

        verify(accelerometerAndGyroscopeCalibratorSpy, only()).estimatedAccelerometerMa
//        verify(exactly = 1) { accelerometerAndGyroscopeCalibratorSpy.estimatedAccelerometerMa }
    }

    @Test
    fun estimatedAccelerometerSx_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.estimatedAccelerometerSx)

        // mock internal calibrator
        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val accelerometerAndGyroscopeCalibratorSpy = spy(accelerometerAndGyroscopeCalibrator)
//        val accelerometerAndGyroscopeCalibratorSpy = spyk(accelerometerAndGyroscopeCalibrator)
        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        whenever(accelerometerAndGyroscopeCalibratorSpy.estimatedAccelerometerSx).thenReturn(value)
/*        every { accelerometerAndGyroscopeCalibratorSpy.estimatedAccelerometerSx }.returns(
            value
        )*/
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibratorSpy
        )

        // check
        assertEquals(value, calibrator.estimatedAccelerometerSx)

        verify(accelerometerAndGyroscopeCalibratorSpy, only()).estimatedAccelerometerSx
//        verify(exactly = 1) { accelerometerAndGyroscopeCalibratorSpy.estimatedAccelerometerSx }
    }

    @Test
    fun estimatedAccelerometerSy_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.estimatedAccelerometerSy)

        // mock internal calibrator
        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val accelerometerAndGyroscopeCalibratorSpy = spy(accelerometerAndGyroscopeCalibrator)
//        val accelerometerAndGyroscopeCalibratorSpy = spyk(accelerometerAndGyroscopeCalibrator)
        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        whenever(accelerometerAndGyroscopeCalibratorSpy.estimatedAccelerometerSy).thenReturn(value)
/*        every { accelerometerAndGyroscopeCalibratorSpy.estimatedAccelerometerSy }.returns(
            value
        )*/
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibratorSpy
        )

        // check
        assertEquals(value, calibrator.estimatedAccelerometerSy)

        verify(accelerometerAndGyroscopeCalibratorSpy, only()).estimatedAccelerometerSy
//        verify(exactly = 1) { accelerometerAndGyroscopeCalibratorSpy.estimatedAccelerometerSy }
    }

    @Test
    fun estimatedAccelerometerSz_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.estimatedAccelerometerSz)

        // mock internal calibrator
        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val accelerometerAndGyroscopeCalibratorSpy = spy(accelerometerAndGyroscopeCalibrator)
//        val accelerometerAndGyroscopeCalibratorSpy = spyk(accelerometerAndGyroscopeCalibrator)
        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        whenever(accelerometerAndGyroscopeCalibratorSpy.estimatedAccelerometerSz).thenReturn(value)
/*        every { accelerometerAndGyroscopeCalibratorSpy.estimatedAccelerometerSz }.returns(
            value
        )*/
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibratorSpy
        )

        // check
        assertEquals(value, calibrator.estimatedAccelerometerSz)

        verify(accelerometerAndGyroscopeCalibratorSpy, only()).estimatedAccelerometerSz
//        verify(exactly = 1) { accelerometerAndGyroscopeCalibratorSpy.estimatedAccelerometerSz }
    }

    @Test
    fun estimatedAccelerometerMxy_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.estimatedAccelerometerMxy)

        // mock internal calibrator
        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val accelerometerAndGyroscopeCalibratorSpy = spy(accelerometerAndGyroscopeCalibrator)
//        val accelerometerAndGyroscopeCalibratorSpy = spyk(accelerometerAndGyroscopeCalibrator)
        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        whenever(accelerometerAndGyroscopeCalibratorSpy.estimatedAccelerometerMxy).thenReturn(value)
/*        every { accelerometerAndGyroscopeCalibratorSpy.estimatedAccelerometerMxy }.returns(
            value
        )*/
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibratorSpy
        )

        // check
        assertEquals(value, calibrator.estimatedAccelerometerMxy)

        verify(accelerometerAndGyroscopeCalibratorSpy, only()).estimatedAccelerometerMxy
//        verify(exactly = 1) { accelerometerAndGyroscopeCalibratorSpy.estimatedAccelerometerMxy }
    }

    @Test
    fun estimatedAccelerometerMxz_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.estimatedAccelerometerMxz)

        // mock internal calibrator
        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val accelerometerAndGyroscopeCalibratorSpy = spy(accelerometerAndGyroscopeCalibrator)
//        val accelerometerAndGyroscopeCalibratorSpy = spyk(accelerometerAndGyroscopeCalibrator)
        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        whenever(accelerometerAndGyroscopeCalibratorSpy.estimatedAccelerometerMxz).thenReturn(value)
/*        every { accelerometerAndGyroscopeCalibratorSpy.estimatedAccelerometerMxz }.returns(
            value
        )*/
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibratorSpy
        )

        // check
        assertEquals(value, calibrator.estimatedAccelerometerMxz)

        verify(accelerometerAndGyroscopeCalibratorSpy, only()).estimatedAccelerometerMxz
//        verify(exactly = 1) { accelerometerAndGyroscopeCalibratorSpy.estimatedAccelerometerMxz }
    }

    @Test
    fun estimatedAccelerometerMyx_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.estimatedAccelerometerMyx)

        // mock internal calibrator
        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val accelerometerAndGyroscopeCalibratorSpy = spy(accelerometerAndGyroscopeCalibrator)
//        val accelerometerAndGyroscopeCalibratorSpy = spyk(accelerometerAndGyroscopeCalibrator)
        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        whenever(accelerometerAndGyroscopeCalibratorSpy.estimatedAccelerometerMyx).thenReturn(value)
/*        every { accelerometerAndGyroscopeCalibratorSpy.estimatedAccelerometerMyx }.returns(
            value
        )*/
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibratorSpy
        )

        // check
        assertEquals(value, calibrator.estimatedAccelerometerMyx)

        verify(accelerometerAndGyroscopeCalibratorSpy, only()).estimatedAccelerometerMyx
//        verify(exactly = 1) { accelerometerAndGyroscopeCalibratorSpy.estimatedAccelerometerMyx }
    }

    @Test
    fun estimatedAccelerometerMyz_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.estimatedAccelerometerMyz)

        // mock internal calibrator
        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val accelerometerAndGyroscopeCalibratorSpy = spy(accelerometerAndGyroscopeCalibrator)
//        val accelerometerAndGyroscopeCalibratorSpy = spyk(accelerometerAndGyroscopeCalibrator)
        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        whenever(accelerometerAndGyroscopeCalibratorSpy.estimatedAccelerometerMyz).thenReturn(value)
/*        every { accelerometerAndGyroscopeCalibratorSpy.estimatedAccelerometerMyz }.returns(
            value
        )*/
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibratorSpy
        )

        // check
        assertEquals(value, calibrator.estimatedAccelerometerMyz)

        verify(accelerometerAndGyroscopeCalibratorSpy, only()).estimatedAccelerometerMyz
//        verify(exactly = 1) { accelerometerAndGyroscopeCalibratorSpy.estimatedAccelerometerMyz }
    }

    @Test
    fun estimatedAccelerometerMzx_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.estimatedAccelerometerMzx)

        // mock internal calibrator
        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val accelerometerAndGyroscopeCalibratorSpy = spy(accelerometerAndGyroscopeCalibrator)
//        val accelerometerAndGyroscopeCalibratorSpy = spyk(accelerometerAndGyroscopeCalibrator)
        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        whenever(accelerometerAndGyroscopeCalibratorSpy.estimatedAccelerometerMzx).thenReturn(value)
/*        every { accelerometerAndGyroscopeCalibratorSpy.estimatedAccelerometerMzx }.returns(
            value
        )*/
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibratorSpy
        )

        // check
        assertEquals(value, calibrator.estimatedAccelerometerMzx)

        verify(accelerometerAndGyroscopeCalibratorSpy, only()).estimatedAccelerometerMzx
//        verify(exactly = 1) { accelerometerAndGyroscopeCalibratorSpy.estimatedAccelerometerMzx }
    }

    @Test
    fun estimatedAccelerometerMzy_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.estimatedAccelerometerMzy)

        // mock internal calibrator
        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val accelerometerAndGyroscopeCalibratorSpy = spy(accelerometerAndGyroscopeCalibrator)
//        val accelerometerAndGyroscopeCalibratorSpy = spyk(accelerometerAndGyroscopeCalibrator)
        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        whenever(accelerometerAndGyroscopeCalibratorSpy.estimatedAccelerometerMzy).thenReturn(value)
/*        every { accelerometerAndGyroscopeCalibratorSpy.estimatedAccelerometerMzy }.returns(
            value
        )*/
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibratorSpy
        )

        // check
        assertEquals(value, calibrator.estimatedAccelerometerMzy)

        verify(accelerometerAndGyroscopeCalibratorSpy, only()).estimatedAccelerometerMzy
//        verify(exactly = 1) { accelerometerAndGyroscopeCalibratorSpy.estimatedAccelerometerMzy }
    }

    @Test
    fun estimatedAccelerometerCovariance_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.estimatedAccelerometerCovariance)

        // mock internal calibrator
        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val accelerometerAndGyroscopeCalibratorSpy = spy(accelerometerAndGyroscopeCalibrator)
        whenever(accelerometerAndGyroscopeCalibratorSpy.estimatedAccelerometerCovariance).thenReturn(matrix)
//        val accelerometerAndGyroscopeCalibratorSpy = spyk(accelerometerAndGyroscopeCalibrator)
/*        every { accelerometerAndGyroscopeCalibratorSpy.estimatedAccelerometerCovariance }.returns(
            matrix
        )*/
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibratorSpy
        )

        // check
        assertSame(matrix, calibrator.estimatedAccelerometerCovariance)

        verify(accelerometerAndGyroscopeCalibratorSpy, only()).estimatedAccelerometerCovariance
//        verify(exactly = 1) { accelerometerAndGyroscopeCalibratorSpy.estimatedAccelerometerCovariance }
    }

    @Test
    fun estimatedAccelerometerChiSq_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.estimatedAccelerometerChiSq)

        // mock internal calibrator
        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val accelerometerAndGyroscopeCalibratorSpy = spy(accelerometerAndGyroscopeCalibrator)
//        val accelerometerAndGyroscopeCalibratorSpy = spyk(accelerometerAndGyroscopeCalibrator)
        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        whenever(accelerometerAndGyroscopeCalibratorSpy.estimatedAccelerometerChiSq).thenReturn(value)
/*        every { accelerometerAndGyroscopeCalibratorSpy.estimatedAccelerometerChiSq }.returns(
            value
        )*/
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibratorSpy
        )

        // check
        assertEquals(value, calibrator.estimatedAccelerometerChiSq)

        verify(accelerometerAndGyroscopeCalibratorSpy, only()).estimatedAccelerometerChiSq
//        verify(exactly = 1) { accelerometerAndGyroscopeCalibratorSpy.estimatedAccelerometerChiSq }
    }

    @Test
    fun estimatedAccelerometerMse_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.estimatedAccelerometerMse)

        // mock internal calibrator
        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val accelerometerAndGyroscopeCalibratorSpy = spy(accelerometerAndGyroscopeCalibrator)
//        val accelerometerAndGyroscopeCalibratorSpy = spyk(accelerometerAndGyroscopeCalibrator)
        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        whenever(accelerometerAndGyroscopeCalibratorSpy.estimatedAccelerometerMse).thenReturn(value)
/*        every { accelerometerAndGyroscopeCalibratorSpy.estimatedAccelerometerMse }.returns(
            value
        )*/
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibratorSpy
        )

        // check
        assertEquals(value, calibrator.estimatedAccelerometerMse)

        verify(accelerometerAndGyroscopeCalibratorSpy, only()).estimatedAccelerometerMse
//        verify(exactly = 1) { accelerometerAndGyroscopeCalibratorSpy.estimatedAccelerometerMse }
    }

    @Test
    fun estimatedGyroscopeMg_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.estimatedGyroscopeMg)

        // mock internal calibrator
        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val accelerometerAndGyroscopeCalibratorSpy = spy(accelerometerAndGyroscopeCalibrator)
        whenever(accelerometerAndGyroscopeCalibratorSpy.estimatedGyroscopeMg).thenReturn(matrix)
//        val accelerometerAndGyroscopeCalibratorSpy = spyk(accelerometerAndGyroscopeCalibrator)
/*        every { accelerometerAndGyroscopeCalibratorSpy.estimatedGyroscopeMg }.returns(
            matrix
        )*/
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibratorSpy
        )

        // check
        assertSame(matrix, calibrator.estimatedGyroscopeMg)

        verify(accelerometerAndGyroscopeCalibratorSpy, only()).estimatedGyroscopeMg
//        verify(exactly = 1) { accelerometerAndGyroscopeCalibratorSpy.estimatedGyroscopeMg }
    }

    @Test
    fun estimatedGyroscopeSx_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.estimatedGyroscopeSx)

        // mock internal calibrator
        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val accelerometerAndGyroscopeCalibratorSpy = spy(accelerometerAndGyroscopeCalibrator)
//        val accelerometerAndGyroscopeCalibratorSpy = spyk(accelerometerAndGyroscopeCalibrator)
        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        whenever(accelerometerAndGyroscopeCalibratorSpy.estimatedGyroscopeSx).thenReturn(value)
/*        every { accelerometerAndGyroscopeCalibratorSpy.estimatedGyroscopeSx }.returns(
            value
        )*/
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibratorSpy
        )

        // check
        assertEquals(value, calibrator.estimatedGyroscopeSx)

        verify(accelerometerAndGyroscopeCalibratorSpy, only()).estimatedGyroscopeSx
//        verify(exactly = 1) { accelerometerAndGyroscopeCalibratorSpy.estimatedGyroscopeSx }
    }

    @Test
    fun estimatedGyroscopeSy_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.estimatedGyroscopeSy)

        // mock internal calibrator
        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val accelerometerAndGyroscopeCalibratorSpy = spy(accelerometerAndGyroscopeCalibrator)
//        val accelerometerAndGyroscopeCalibratorSpy = spyk(accelerometerAndGyroscopeCalibrator)
        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        whenever(accelerometerAndGyroscopeCalibratorSpy.estimatedGyroscopeSy).thenReturn(value)
/*        every { accelerometerAndGyroscopeCalibratorSpy.estimatedGyroscopeSy }.returns(
            value
        )*/
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibratorSpy
        )

        // check
        assertEquals(value, calibrator.estimatedGyroscopeSy)

        verify(accelerometerAndGyroscopeCalibratorSpy, only()).estimatedGyroscopeSy
//        verify(exactly = 1) { accelerometerAndGyroscopeCalibratorSpy.estimatedGyroscopeSy }
    }

    @Test
    fun estimatedGyroscopeSz_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.estimatedGyroscopeSz)

        // mock internal calibrator
        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val accelerometerAndGyroscopeCalibratorSpy = spy(accelerometerAndGyroscopeCalibrator)
//        val accelerometerAndGyroscopeCalibratorSpy = spyk(accelerometerAndGyroscopeCalibrator)
        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        whenever(accelerometerAndGyroscopeCalibratorSpy.estimatedGyroscopeSz).thenReturn(value)
/*        every { accelerometerAndGyroscopeCalibratorSpy.estimatedGyroscopeSz }.returns(
            value
        )*/
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibratorSpy
        )

        // check
        assertEquals(value, calibrator.estimatedGyroscopeSz)

        verify(accelerometerAndGyroscopeCalibratorSpy, only()).estimatedGyroscopeSz
//        verify(exactly = 1) { accelerometerAndGyroscopeCalibratorSpy.estimatedGyroscopeSz }
    }

    @Test
    fun estimatedGyroscopeMxy_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.estimatedGyroscopeMxy)

        // mock internal calibrator
        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val accelerometerAndGyroscopeCalibratorSpy = spy(accelerometerAndGyroscopeCalibrator)
//        val accelerometerAndGyroscopeCalibratorSpy = spyk(accelerometerAndGyroscopeCalibrator)
        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        whenever(accelerometerAndGyroscopeCalibratorSpy.estimatedGyroscopeMxy).thenReturn(value)
/*        every { accelerometerAndGyroscopeCalibratorSpy.estimatedGyroscopeMxy }.returns(
            value
        )*/
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibratorSpy
        )

        // check
        assertEquals(value, calibrator.estimatedGyroscopeMxy)

        verify(accelerometerAndGyroscopeCalibratorSpy, only()).estimatedGyroscopeMxy
//        verify(exactly = 1) { accelerometerAndGyroscopeCalibratorSpy.estimatedGyroscopeMxy }
    }

    @Test
    fun estimatedGyroscopeMxz_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.estimatedGyroscopeMxz)

        // mock internal calibrator
        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val accelerometerAndGyroscopeCalibratorSpy = spy(accelerometerAndGyroscopeCalibrator)
//        val accelerometerAndGyroscopeCalibratorSpy = spyk(accelerometerAndGyroscopeCalibrator)
        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        whenever(accelerometerAndGyroscopeCalibratorSpy.estimatedGyroscopeMxz).thenReturn(value)
/*        every { accelerometerAndGyroscopeCalibratorSpy.estimatedGyroscopeMxz }.returns(
            value
        )*/
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibratorSpy
        )

        // check
        assertEquals(value, calibrator.estimatedGyroscopeMxz)

        verify(accelerometerAndGyroscopeCalibratorSpy, only()).estimatedGyroscopeMxz
//        verify(exactly = 1) { accelerometerAndGyroscopeCalibratorSpy.estimatedGyroscopeMxz }
    }

    @Test
    fun estimatedGyroscopeMyx_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.estimatedGyroscopeMyx)

        // mock internal calibrator
        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val accelerometerAndGyroscopeCalibratorSpy = spy(accelerometerAndGyroscopeCalibrator)
//        val accelerometerAndGyroscopeCalibratorSpy = spyk(accelerometerAndGyroscopeCalibrator)
        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        whenever(accelerometerAndGyroscopeCalibratorSpy.estimatedGyroscopeMyx).thenReturn(value)
/*        every { accelerometerAndGyroscopeCalibratorSpy.estimatedGyroscopeMyx }.returns(
            value
        )*/
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibratorSpy
        )

        // check
        assertEquals(value, calibrator.estimatedGyroscopeMyx)

        verify(accelerometerAndGyroscopeCalibratorSpy, only()).estimatedGyroscopeMyx
//        verify(exactly = 1) { accelerometerAndGyroscopeCalibratorSpy.estimatedGyroscopeMyx }
    }

    @Test
    fun estimatedGyroscopeMyz_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.estimatedGyroscopeMyz)

        // mock internal calibrator
        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val accelerometerAndGyroscopeCalibratorSpy = spy(accelerometerAndGyroscopeCalibrator)
//        val accelerometerAndGyroscopeCalibratorSpy = spyk(accelerometerAndGyroscopeCalibrator)
        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        whenever(accelerometerAndGyroscopeCalibratorSpy.estimatedGyroscopeMyz).thenReturn(value)
/*        every { accelerometerAndGyroscopeCalibratorSpy.estimatedGyroscopeMyz }.returns(
            value
        )*/
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibratorSpy
        )

        // check
        assertEquals(value, calibrator.estimatedGyroscopeMyz)

        verify(accelerometerAndGyroscopeCalibratorSpy, only()).estimatedGyroscopeMyz
//        verify(exactly = 1) { accelerometerAndGyroscopeCalibratorSpy.estimatedGyroscopeMyz }
    }

    @Test
    fun estimatedGyroscopeMzx_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.estimatedGyroscopeMzx)

        // mock internal calibrator
        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val accelerometerAndGyroscopeCalibratorSpy = spy(accelerometerAndGyroscopeCalibrator)
//        val accelerometerAndGyroscopeCalibratorSpy = spyk(accelerometerAndGyroscopeCalibrator)
        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        whenever(accelerometerAndGyroscopeCalibratorSpy.estimatedGyroscopeMzx).thenReturn(value)
/*        every { accelerometerAndGyroscopeCalibratorSpy.estimatedGyroscopeMzx }.returns(
            value
        )*/
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibratorSpy
        )

        // check
        assertEquals(value, calibrator.estimatedGyroscopeMzx)

        verify(accelerometerAndGyroscopeCalibratorSpy, only()).estimatedGyroscopeMzx
//        verify(exactly = 1) { accelerometerAndGyroscopeCalibratorSpy.estimatedGyroscopeMzx }
    }

    @Test
    fun estimatedGyroscopeMzy_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.estimatedGyroscopeMzy)

        // mock internal calibrator
        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val accelerometerAndGyroscopeCalibratorSpy = spy(accelerometerAndGyroscopeCalibrator)
//        val accelerometerAndGyroscopeCalibratorSpy = spyk(accelerometerAndGyroscopeCalibrator)
        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        whenever(accelerometerAndGyroscopeCalibratorSpy.estimatedGyroscopeMzy).thenReturn(value)
/*        every { accelerometerAndGyroscopeCalibratorSpy.estimatedGyroscopeMzy }.returns(
            value
        )*/
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibratorSpy
        )

        // check
        assertEquals(value, calibrator.estimatedGyroscopeMzy)

        verify(accelerometerAndGyroscopeCalibratorSpy, only()).estimatedGyroscopeMzy
//        verify(exactly = 1) { accelerometerAndGyroscopeCalibratorSpy.estimatedGyroscopeMzy }
    }

    @Test
    fun estimatedGyroscopeGg_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.estimatedGyroscopeGg)

        // mock internal calibrator
        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val accelerometerAndGyroscopeCalibratorSpy = spy(accelerometerAndGyroscopeCalibrator)
        whenever(accelerometerAndGyroscopeCalibratorSpy.estimatedGyroscopeGg).thenReturn(matrix)
//        val accelerometerAndGyroscopeCalibratorSpy = spyk(accelerometerAndGyroscopeCalibrator)
/*        every { accelerometerAndGyroscopeCalibratorSpy.estimatedGyroscopeGg }.returns(
            matrix
        )*/
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibratorSpy
        )

        // check
        assertSame(matrix, calibrator.estimatedGyroscopeGg)

        verify(accelerometerAndGyroscopeCalibratorSpy, only()).estimatedGyroscopeGg
//        verify(exactly = 1) { accelerometerAndGyroscopeCalibratorSpy.estimatedGyroscopeGg }
    }

    @Test
    fun estimatedGyroscopeCovariance_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.estimatedGyroscopeCovariance)

        // mock internal calibrator
        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val accelerometerAndGyroscopeCalibratorSpy = spy(accelerometerAndGyroscopeCalibrator)
        whenever(accelerometerAndGyroscopeCalibratorSpy.estimatedGyroscopeCovariance).thenReturn(matrix)
//        val accelerometerAndGyroscopeCalibratorSpy = spyk(accelerometerAndGyroscopeCalibrator)
/*        every { accelerometerAndGyroscopeCalibratorSpy.estimatedGyroscopeCovariance }.returns(
            matrix
        )*/
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibratorSpy
        )

        // check
        assertSame(matrix, calibrator.estimatedGyroscopeCovariance)

        verify(accelerometerAndGyroscopeCalibratorSpy, only()).estimatedGyroscopeCovariance
//        verify(exactly = 1) { accelerometerAndGyroscopeCalibratorSpy.estimatedGyroscopeCovariance }
    }

    @Test
    fun estimatedGyroscopeChiSq_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.estimatedGyroscopeChiSq)

        // mock internal calibrator
        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val accelerometerAndGyroscopeCalibratorSpy = spy(accelerometerAndGyroscopeCalibrator)
//        val accelerometerAndGyroscopeCalibratorSpy = spyk(accelerometerAndGyroscopeCalibrator)
        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        whenever(accelerometerAndGyroscopeCalibratorSpy.estimatedGyroscopeChiSq).thenReturn(value)
/*        every { accelerometerAndGyroscopeCalibratorSpy.estimatedGyroscopeChiSq }.returns(
            value
        )*/
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibratorSpy
        )

        // check
        assertEquals(value, calibrator.estimatedGyroscopeChiSq)

        verify(accelerometerAndGyroscopeCalibratorSpy, only()).estimatedGyroscopeChiSq
//        verify(exactly = 1) { accelerometerAndGyroscopeCalibratorSpy.estimatedGyroscopeChiSq }
    }

    @Test
    fun estimatedGyroscopeMse_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.estimatedGyroscopeMse)

        // mock internal calibrator
        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val accelerometerAndGyroscopeCalibratorSpy = spy(accelerometerAndGyroscopeCalibrator)
//        val accelerometerAndGyroscopeCalibratorSpy = spyk(accelerometerAndGyroscopeCalibrator)
        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        whenever(accelerometerAndGyroscopeCalibratorSpy.estimatedGyroscopeMse).thenReturn(value)
/*        every { accelerometerAndGyroscopeCalibratorSpy.estimatedGyroscopeMse }.returns(
            value
        )*/
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibratorSpy
        )

        // check
        assertEquals(value, calibrator.estimatedGyroscopeMse)

        verify(accelerometerAndGyroscopeCalibratorSpy, only()).estimatedGyroscopeMse
//        verify(exactly = 1) { accelerometerAndGyroscopeCalibratorSpy.estimatedGyroscopeMse }
    }

    @Test
    fun estimatedMagnetometerMm_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.estimatedMagnetometerMm)

        // mock internal calibrator
        whenever(magnetometerCalibrator.estimatedMm).thenReturn(matrix)
/*        every { magnetometerCalibrator.estimatedMm }.returns(
            matrix
        )*/
        calibrator.setPrivateProperty(
            "magnetometerCalibrator",
            magnetometerCalibrator
        )

        // check
        assertSame(matrix, calibrator.estimatedMagnetometerMm)

        verify(magnetometerCalibrator, only()).estimatedMm
//        verify(exactly = 1) { magnetometerCalibrator.estimatedMm }
    }

    @Test
    fun estimatedMagnetometerSx_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.estimatedMagnetometerSx)

        // mock internal calibrator
        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        whenever(magnetometerCalibrator.estimatedSx).thenReturn(value)
/*        every { magnetometerCalibrator.estimatedSx }.returns(
            value
        )*/
        calibrator.setPrivateProperty(
            "magnetometerCalibrator",
            magnetometerCalibrator
        )

        // check
        assertEquals(value, calibrator.estimatedMagnetometerSx)

        verify(magnetometerCalibrator, only()).estimatedSx
//        verify(exactly = 1) { magnetometerCalibrator.estimatedSx }
    }

    @Test
    fun estimatedMagnetometerSy_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.estimatedMagnetometerSy)

        // mock internal calibrator
        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        whenever(magnetometerCalibrator.estimatedSy).thenReturn(value)
/*        every { magnetometerCalibrator.estimatedSy }.returns(
            value
        )*/
        calibrator.setPrivateProperty(
            "magnetometerCalibrator",
            magnetometerCalibrator
        )

        // check
        assertEquals(value, calibrator.estimatedMagnetometerSy)

        verify(magnetometerCalibrator, only()).estimatedSy
//        verify(exactly = 1) { magnetometerCalibrator.estimatedSy }
    }

    @Test
    fun estimatedMagnetometerSz_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.estimatedMagnetometerSz)

        // mock internal calibrator
        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        whenever(magnetometerCalibrator.estimatedSz).thenReturn(value)
/*        every { magnetometerCalibrator.estimatedSz }.returns(
            value
        )*/
        calibrator.setPrivateProperty(
            "magnetometerCalibrator",
            magnetometerCalibrator
        )

        // check
        assertEquals(value, calibrator.estimatedMagnetometerSz)

        verify(magnetometerCalibrator, only()).estimatedSz
//        verify(exactly = 1) { magnetometerCalibrator.estimatedSz }
    }

    @Test
    fun estimatedMagnetometerMxy_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.estimatedMagnetometerMxy)

        // mock internal calibrator
        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        whenever(magnetometerCalibrator.estimatedMxy).thenReturn(value)
/*        every { magnetometerCalibrator.estimatedMxy }.returns(
            value
        )*/
        calibrator.setPrivateProperty(
            "magnetometerCalibrator",
            magnetometerCalibrator
        )

        // check
        assertEquals(value, calibrator.estimatedMagnetometerMxy)

        verify(magnetometerCalibrator, only()).estimatedMxy
//        verify(exactly = 1) { magnetometerCalibrator.estimatedMxy }
    }

    @Test
    fun estimatedMagnetometerMxz_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.estimatedMagnetometerMxz)

        // mock internal calibrator
        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        whenever(magnetometerCalibrator.estimatedMxz).thenReturn(value)
/*        every { magnetometerCalibrator.estimatedMxz }.returns(
            value
        )*/
        calibrator.setPrivateProperty(
            "magnetometerCalibrator",
            magnetometerCalibrator
        )

        // check
        assertEquals(value, calibrator.estimatedMagnetometerMxz)

        verify(magnetometerCalibrator, only()).estimatedMxz
//        verify(exactly = 1) { magnetometerCalibrator.estimatedMxz }
    }

    @Test
    fun estimatedMagnetometerMyx_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.estimatedMagnetometerMyx)

        // mock internal calibrator
        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        whenever(magnetometerCalibrator.estimatedMyx).thenReturn(value)
/*        every { magnetometerCalibrator.estimatedMyx }.returns(
            value
        )*/
        calibrator.setPrivateProperty(
            "magnetometerCalibrator",
            magnetometerCalibrator
        )

        // check
        assertEquals(value, calibrator.estimatedMagnetometerMyx)

        verify(magnetometerCalibrator, only()).estimatedMyx
//        verify(exactly = 1) { magnetometerCalibrator.estimatedMyx }
    }

    @Test
    fun estimatedMagnetometerMyz_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.estimatedMagnetometerMyz)

        // mock internal calibrator
        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        whenever(magnetometerCalibrator.estimatedMyz).thenReturn(value)
/*        every { magnetometerCalibrator.estimatedMyz }.returns(
            value
        )*/
        calibrator.setPrivateProperty(
            "magnetometerCalibrator",
            magnetometerCalibrator
        )

        // check
        assertEquals(value, calibrator.estimatedMagnetometerMyz)

        verify(magnetometerCalibrator, only()).estimatedMyz
//        verify(exactly = 1) { magnetometerCalibrator.estimatedMyz }
    }

    @Test
    fun estimatedMagnetometerMzx_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.estimatedMagnetometerMzx)

        // mock internal calibrator
        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        whenever(magnetometerCalibrator.estimatedMzx).thenReturn(value)
/*        every { magnetometerCalibrator.estimatedMzx }.returns(
            value
        )*/
        calibrator.setPrivateProperty(
            "magnetometerCalibrator",
            magnetometerCalibrator
        )

        // check
        assertEquals(value, calibrator.estimatedMagnetometerMzx)

        verify(magnetometerCalibrator, only()).estimatedMzx
//        verify(exactly = 1) { magnetometerCalibrator.estimatedMzx }
    }

    @Test
    fun estimatedMagnetometerMzy_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.estimatedMagnetometerMzy)

        // mock internal calibrator
        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        whenever(magnetometerCalibrator.estimatedMzy).thenReturn(value)
/*        every { magnetometerCalibrator.estimatedMzy }.returns(
            value
        )*/
        calibrator.setPrivateProperty(
            "magnetometerCalibrator",
            magnetometerCalibrator
        )

        // check
        assertEquals(value, calibrator.estimatedMagnetometerMzy)

        verify(magnetometerCalibrator, only()).estimatedMzy
//        verify(exactly = 1) { magnetometerCalibrator.estimatedMzy }
    }

    @Test
    fun estimatedMagnetometerCovariance_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.estimatedMagnetometerCovariance)

        // mock internal calibrator
        whenever(magnetometerCalibrator.estimatedCovariance).thenReturn(matrix)
/*        every { magnetometerCalibrator.estimatedCovariance }.returns(
            matrix
        )*/
        calibrator.setPrivateProperty(
            "magnetometerCalibrator",
            magnetometerCalibrator
        )

        // check
        assertSame(matrix, calibrator.estimatedMagnetometerCovariance)

        verify(magnetometerCalibrator, only()).estimatedCovariance
//        verify(exactly = 1) { magnetometerCalibrator.estimatedCovariance }
    }

    @Test
    fun estimatedMagnetometerChiSq_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.estimatedMagnetometerChiSq)

        // mock internal calibrator
        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        whenever(magnetometerCalibrator.estimatedChiSq).thenReturn(value)
/*        every { magnetometerCalibrator.estimatedChiSq }.returns(
            value
        )*/
        calibrator.setPrivateProperty(
            "magnetometerCalibrator",
            magnetometerCalibrator
        )

        // check
        assertEquals(value, calibrator.estimatedMagnetometerChiSq)

        verify(magnetometerCalibrator, only()).estimatedChiSq
//        verify(exactly = 1) { magnetometerCalibrator.estimatedChiSq }
    }

    @Test
    fun estimatedMagnetometerMse_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNull(calibrator.estimatedMagnetometerMse)

        // mock internal calibrator
        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        whenever(magnetometerCalibrator.estimatedMse).thenReturn(value)
/*        every { magnetometerCalibrator.estimatedMse }.returns(
            value
        )*/
        calibrator.setPrivateProperty(
            "magnetometerCalibrator",
            magnetometerCalibrator
        )

        // check
        assertEquals(value, calibrator.estimatedMagnetometerMse)

        verify(magnetometerCalibrator, only()).estimatedMse
//        verify(exactly = 1) { magnetometerCalibrator.estimatedMse }
    }

    @Test
    fun accelerometerMeasurements_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNotNull(calibrator.accelerometerMeasurements)
        assertTrue(calibrator.accelerometerMeasurements.isEmpty())

        // mock internal calibrator
        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val accelerometerMeasurements =
            accelerometerAndGyroscopeCalibrator.accelerometerMeasurements

        // check
        assertSame(accelerometerMeasurements, calibrator.accelerometerMeasurements)
    }

    @Test
    fun gyroscopeMeasurements_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNotNull(calibrator.gyroscopeMeasurements)
        assertTrue(calibrator.gyroscopeMeasurements.isEmpty())

        // mock internal calibrator
        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val gyroscopeMeasurements = accelerometerAndGyroscopeCalibrator.gyroscopeMeasurements

        // check
        assertSame(gyroscopeMeasurements, calibrator.gyroscopeMeasurements)
    }

    @Test
    fun magnetometerMeasurements_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertNotNull(calibrator.magnetometerMeasurements)
        assertTrue(calibrator.magnetometerMeasurements.isEmpty())

        // mock internal calibrator
        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        val magnetometerMeasurements = magnetometerCalibrator.measurements

        // check
        assertSame(magnetometerMeasurements, calibrator.magnetometerMeasurements)
    }

    @Test
    fun isReadyToSolveAccelerometerCalibration_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertFalse(calibrator.isReadyToSolveAccelerometerCalibration)

        // mock internal calibrator
        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val accelerometerAndGyroscopeCalibratorSpy = spy(accelerometerAndGyroscopeCalibrator)
//        val accelerometerAndGyroscopeCalibratorSpy = spyk(accelerometerAndGyroscopeCalibrator)
        val randomizer = UniformRandomizer()
        val value = randomizer.nextBoolean()
        whenever(accelerometerAndGyroscopeCalibratorSpy.isReadyToSolveAccelerometerCalibration).thenReturn(value)
/*        every { accelerometerAndGyroscopeCalibratorSpy.isReadyToSolveAccelerometerCalibration }.returns(
            value
        )*/
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibratorSpy
        )

        // check
        assertEquals(value, calibrator.isReadyToSolveAccelerometerCalibration)

        verify(accelerometerAndGyroscopeCalibratorSpy, times(2)).isReadyToSolveAccelerometerCalibration
//        verify(exactly = 1) { accelerometerAndGyroscopeCalibratorSpy.isReadyToSolveAccelerometerCalibration }
    }

    @Test
    fun isReadyToSolveGyroscopeCalibration_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertFalse(calibrator.isReadyToSolveGyroscopeCalibration)

        // mock internal calibrator
        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val accelerometerAndGyroscopeCalibratorSpy = spy(accelerometerAndGyroscopeCalibrator)
//        val accelerometerAndGyroscopeCalibratorSpy = spyk(accelerometerAndGyroscopeCalibrator)
        val randomizer = UniformRandomizer()
        val value = randomizer.nextBoolean()
        whenever(accelerometerAndGyroscopeCalibratorSpy.isReadyToSolveGyroscopeCalibration).thenReturn(value)
/*        every { accelerometerAndGyroscopeCalibratorSpy.isReadyToSolveGyroscopeCalibration }.returns(
            value
        )*/
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibratorSpy
        )

        // check
        assertEquals(value, calibrator.isReadyToSolveGyroscopeCalibration)

        verify(accelerometerAndGyroscopeCalibratorSpy, times(2)).isReadyToSolveGyroscopeCalibration
//        verify(exactly = 1) { accelerometerAndGyroscopeCalibratorSpy.isReadyToSolveGyroscopeCalibration }
    }

    @Test
    fun isReadyToSolveMagnetometerCalibration_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertFalse(calibrator.isReadyToSolveMagnetometerCalibration)

        // mock internal calibrator
        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        val magnetometerCalibratorSpy = spy(magnetometerCalibrator)
//        val magnetometerCalibratorSpy = spyk(magnetometerCalibrator)
        val randomizer = UniformRandomizer()
        val value = randomizer.nextBoolean()
        whenever(magnetometerCalibratorSpy.isReadyToSolveCalibration).thenReturn(value)
//        every { magnetometerCalibratorSpy.isReadyToSolveCalibration }.returns(value)
        calibrator.setPrivateProperty("magnetometerCalibrator", magnetometerCalibratorSpy)

        // check
        assertEquals(value, calibrator.isReadyToSolveMagnetometerCalibration)

        verify(magnetometerCalibratorSpy, times(2)).isReadyToSolveCalibration
//        verify(exactly = 1) { magnetometerCalibratorSpy.isReadyToSolveCalibration }
    }

    @Test
    fun isReadyToSolveCalibration_whenAccelerometerNotReady_returnsFalse() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertFalse(calibrator.isReadyToSolveCalibration)

        // mock internal calibrator
        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val accelerometerAndGyroscopeCalibratorSpy = spy(accelerometerAndGyroscopeCalibrator)
//        val accelerometerAndGyroscopeCalibratorSpy = spyk(accelerometerAndGyroscopeCalibrator)
        whenever(accelerometerAndGyroscopeCalibratorSpy.isReadyToSolveAccelerometerCalibration).thenReturn(false)
/*        every { accelerometerAndGyroscopeCalibratorSpy.isReadyToSolveAccelerometerCalibration }.returns(
            false
        )*/
        whenever(accelerometerAndGyroscopeCalibratorSpy.isReadyToSolveGyroscopeCalibration).thenReturn(true)
/*        every { accelerometerAndGyroscopeCalibratorSpy.isReadyToSolveGyroscopeCalibration }.returns(
            true
        )*/
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibratorSpy
        )

        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        val magnetometerCalibratorSpy = spy(magnetometerCalibrator)
        doReturn(true).whenever(magnetometerCalibratorSpy).isReadyToSolveCalibration
//        val magnetometerCalibratorSpy = spyk(magnetometerCalibrator)
//        every { magnetometerCalibratorSpy.isReadyToSolveCalibration }.returns(true)
        calibrator.setPrivateProperty("magnetometerCalibrator", magnetometerCalibratorSpy)

        // check
        assertFalse(calibrator.isReadyToSolveCalibration)

        verify(accelerometerAndGyroscopeCalibratorSpy, times(2)).isReadyToSolveAccelerometerCalibration
        verify(accelerometerAndGyroscopeCalibratorSpy, times(1)).isReadyToSolveGyroscopeCalibration
        verifyNoInteractions(magnetometerCalibratorSpy)
//        verify(exactly = 1) { accelerometerAndGyroscopeCalibratorSpy.isReadyToSolveAccelerometerCalibration }
//        verify(exactly = 0) { accelerometerAndGyroscopeCalibratorSpy.isReadyToSolveGyroscopeCalibration }
//        verify { magnetometerCalibratorSpy wasNot Called }
    }

    @Test
    fun isReadyToSolveCalibration_whenGyroscopeNotReady_returnsFalse() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertFalse(calibrator.isReadyToSolveCalibration)

        // mock internal calibrator
        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val accelerometerAndGyroscopeCalibratorSpy = spy(accelerometerAndGyroscopeCalibrator)
        whenever(accelerometerAndGyroscopeCalibratorSpy.isReadyToSolveAccelerometerCalibration).thenReturn(true)
//        val accelerometerAndGyroscopeCalibratorSpy = spyk(accelerometerAndGyroscopeCalibrator)
/*        every { accelerometerAndGyroscopeCalibratorSpy.isReadyToSolveAccelerometerCalibration }.returns(
            true
        )*/
        whenever(accelerometerAndGyroscopeCalibratorSpy.isReadyToSolveGyroscopeCalibration).thenReturn(false)
/*        every { accelerometerAndGyroscopeCalibratorSpy.isReadyToSolveGyroscopeCalibration }.returns(
            false
        )*/
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibratorSpy
        )

        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        val magnetometerCalibratorSpy = spy(magnetometerCalibrator)
//        val magnetometerCalibratorSpy = spyk(magnetometerCalibrator)
        doReturn(false).whenever(magnetometerCalibratorSpy).isReadyToSolveCalibration
//        every { magnetometerCalibratorSpy.isReadyToSolveCalibration }.returns(false)
        calibrator.setPrivateProperty("magnetometerCalibrator", magnetometerCalibratorSpy)

        // check
        assertFalse(calibrator.isReadyToSolveCalibration)

        verify(accelerometerAndGyroscopeCalibratorSpy, times(2)).isReadyToSolveAccelerometerCalibration
        verify(accelerometerAndGyroscopeCalibratorSpy, times(2)).isReadyToSolveGyroscopeCalibration
        verifyNoInteractions(magnetometerCalibratorSpy)
//        verify(exactly = 1) { accelerometerAndGyroscopeCalibratorSpy.isReadyToSolveAccelerometerCalibration }
//        verify(exactly = 1) { accelerometerAndGyroscopeCalibratorSpy.isReadyToSolveGyroscopeCalibration }
//        verify { magnetometerCalibratorSpy wasNot Called }
    }

    @Test
    fun isReadyToSolveCalibration_whenMagnetometerNotReady_returnsFalse() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertFalse(calibrator.isReadyToSolveCalibration)

        // mock internal calibrator
        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val accelerometerAndGyroscopeCalibratorSpy = spy(accelerometerAndGyroscopeCalibrator)
        whenever(accelerometerAndGyroscopeCalibratorSpy.isReadyToSolveAccelerometerCalibration).thenReturn(true)
        whenever(accelerometerAndGyroscopeCalibratorSpy.isReadyToSolveGyroscopeCalibration).thenReturn(true)
//        val accelerometerAndGyroscopeCalibratorSpy = spyk(accelerometerAndGyroscopeCalibrator)
/*        every { accelerometerAndGyroscopeCalibratorSpy.isReadyToSolveAccelerometerCalibration }.returns(
            true
        )*/
/*        every { accelerometerAndGyroscopeCalibratorSpy.isReadyToSolveGyroscopeCalibration }.returns(
            true
        )*/
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibratorSpy
        )

        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        val magnetometerCalibratorSpy = spy(magnetometerCalibrator)
        whenever(magnetometerCalibratorSpy.isReadyToSolveCalibration).thenReturn(false)
//        val magnetometerCalibratorSpy = spyk(magnetometerCalibrator)
//        every { magnetometerCalibratorSpy.isReadyToSolveCalibration }.returns(false)
        calibrator.setPrivateProperty("magnetometerCalibrator", magnetometerCalibratorSpy)

        // check
        assertFalse(calibrator.isReadyToSolveCalibration)

        verify(accelerometerAndGyroscopeCalibratorSpy, times(2)).isReadyToSolveAccelerometerCalibration
        verify(accelerometerAndGyroscopeCalibratorSpy, times(2)).isReadyToSolveGyroscopeCalibration
        verify(magnetometerCalibratorSpy, times(2)).isReadyToSolveCalibration
//        verify(exactly = 1) { accelerometerAndGyroscopeCalibratorSpy.isReadyToSolveAccelerometerCalibration }
//        verify(exactly = 1) { accelerometerAndGyroscopeCalibratorSpy.isReadyToSolveGyroscopeCalibration }
//        verify(exactly = 1) { magnetometerCalibratorSpy.isReadyToSolveCalibration }
    }

    @Test
    fun isReadyToSolveCalibration_whenReady_returnsTrue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(context)

        // check default value
        assertFalse(calibrator.isReadyToSolveCalibration)

        // mock internal calibrator
        val accelerometerAndGyroscopeCalibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator? =
            calibrator.getPrivateProperty("accelerometerAndGyroscopeCalibrator")
        requireNotNull(accelerometerAndGyroscopeCalibrator)
        val accelerometerAndGyroscopeCalibratorSpy = spy(accelerometerAndGyroscopeCalibrator)
        whenever(accelerometerAndGyroscopeCalibratorSpy.isReadyToSolveAccelerometerCalibration).thenReturn(true)
//        val accelerometerAndGyroscopeCalibratorSpy = spyk(accelerometerAndGyroscopeCalibrator)
/*        every { accelerometerAndGyroscopeCalibratorSpy.isReadyToSolveAccelerometerCalibration }.returns(
            true
        )*/
        whenever(accelerometerAndGyroscopeCalibratorSpy.isReadyToSolveGyroscopeCalibration).thenReturn(true)
/*        every { accelerometerAndGyroscopeCalibratorSpy.isReadyToSolveGyroscopeCalibration }.returns(
            true
        )*/
        calibrator.setPrivateProperty(
            "accelerometerAndGyroscopeCalibrator",
            accelerometerAndGyroscopeCalibratorSpy
        )

        val magnetometerCalibrator: SingleSensorStaticIntervalMagnetometerCalibrator? =
            calibrator.getPrivateProperty("magnetometerCalibrator")
        requireNotNull(magnetometerCalibrator)
        val magnetometerCalibratorSpy = spy(magnetometerCalibrator)
        doReturn(true).whenever(magnetometerCalibratorSpy).isReadyToSolveCalibration
//        val magnetometerCalibratorSpy = spyk(magnetometerCalibrator)
//        every { magnetometerCalibratorSpy.isReadyToSolveCalibration }.returns(true)
        calibrator.setPrivateProperty("magnetometerCalibrator", magnetometerCalibratorSpy)

        // check
        assertTrue(calibrator.isReadyToSolveCalibration)

        verify(accelerometerAndGyroscopeCalibratorSpy, times(2)).isReadyToSolveAccelerometerCalibration
        verify(accelerometerAndGyroscopeCalibratorSpy, times(2)).isReadyToSolveGyroscopeCalibration
        verify(magnetometerCalibratorSpy, only()).isReadyToSolveCalibration
//        verify(exactly = 1) { accelerometerAndGyroscopeCalibratorSpy.isReadyToSolveAccelerometerCalibration }
//        verify(exactly = 1) { accelerometerAndGyroscopeCalibratorSpy.isReadyToSolveGyroscopeCalibration }
//        verify(exactly = 1) { magnetometerCalibratorSpy.isReadyToSolveCalibration }
    }

    private fun getLocation(): Location {
        val randomizer = UniformRandomizer()
        val latitudeDegrees = randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES)
        val longitudeDegrees =
            randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES)
        val height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT)

        whenever(location.latitude).thenReturn(latitudeDegrees)
        whenever(location.longitude).thenReturn(longitudeDegrees)
        whenever(location.altitude).thenReturn(height)
//        every { location.latitude }.returns(latitudeDegrees)
//        every { location.longitude }.returns(longitudeDegrees)
//        every { location.altitude }.returns(height)

        return location
    }

    private companion object {
        const val MA_SIZE = 3
        const val MM_SIZE = 3

        const val MIN_LATITUDE_DEGREES = -90.0
        const val MAX_LATITUDE_DEGREES = 90.0

        const val MIN_LONGITUDE_DEGREES = -180.0
        const val MAX_LONGITUDE_DEGREES = 180.0

        const val MIN_HEIGHT = -100.0
        const val MAX_HEIGHT = 3000.0

        const val TIME_INTERVAL_SECONDS = 0.02

        const val MIN_ANGLE_DEGREES = -180.0
        const val MAX_ANGLE_DEGREES = 180.0

        const val MIN_ANGLE_VARIATION_DEGREES = -2.0
        const val MAX_ANGLE_VARIATION_DEGREES = 2.0

        const val INITIAL_STATIC_SAMPLES = 2500

        const val THRESHOLD_FACTOR = 3.0

        const val INSTANTANEOUS_NOISE_LEVEL_FACTOR = 3.0

        const val BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD = 1e-5

        const val WINDOW_SIZE = 51

        const val REQUIRED_MEASUREMENTS = 30

        const val ROBUST_CONFIDENCE = 0.9

        const val ROBUST_MAX_ITERATIONS = 1000

        const val ROBUST_PRELIMINARY_SUBSET_SIZE = 19

        const val ROBUST_THRESHOLD = 1e-5

        const val ROBUST_THRESHOLD_FACTOR = 2.0

        const val ROBUST_STOP_THRESHOLD_FACTOR = 1e-3

        const val ACCUMULATED_STD = 0.007

        const val MICRO_G_TO_METERS_PER_SECOND_SQUARED = 9.80665E-6
        const val DEG_TO_RAD = 0.01745329252

        const val ABSOLUTE_ERROR = 1e-6

        const val MIN_HARD_IRON = -1e-5
        const val MAX_HARD_IRON = 1e-5

        const val MIN_SOFT_IRON = -1e-6
        const val MAX_SOFT_IRON = 1e-6

        const val MAGNETOMETER_NOISE_STD = 200e-9
    }
}