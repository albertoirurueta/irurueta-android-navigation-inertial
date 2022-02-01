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
package com.irurueta.android.navigation.inertial.calibration.intervals

import android.content.Context
import com.irurueta.android.navigation.inertial.collectors.MagnetometerSensorCollector
import com.irurueta.android.navigation.inertial.collectors.SensorAccuracy
import com.irurueta.android.navigation.inertial.collectors.SensorCollector
import com.irurueta.android.navigation.inertial.collectors.SensorDelay
import com.irurueta.navigation.inertial.calibration.MagneticFluxDensityTriad
import com.irurueta.navigation.inertial.calibration.TimeIntervalEstimator
import com.irurueta.navigation.inertial.calibration.intervals.MagneticFluxDensityTriadStaticIntervalDetector
import com.irurueta.navigation.inertial.calibration.intervals.MagneticFluxDensityTriadStaticIntervalDetectorListener
import com.irurueta.navigation.inertial.calibration.intervals.TriadStaticIntervalDetector
import com.irurueta.units.*

/**
 * Detects static or motion intervals of time by using the magnetometer of the device to determine
 * whether the device is moving or not.
 * When detector is started, initialization occurs to determine the magnetometer noise level while
 * keeping device static. Once the detector is initialized, then static or dynamic intervals can be
 * detected.
 * This detector uses accumulated average values during static intervals, and windowed
 * averages as "instantaneous" values during dynamic intervals.
 * Length of windows, as well as thresholds to determine when changes between static and dynamic
 * intervals occur can be easily configured.
 *
 * @property context Android context.
 * @property sensorType One of the supported magnetometer sensor types.
 * @property sensorDelay Delay of sensor between samples.
 * @property initializationStartedListener listener to notify when this detector starts
 * initialization after being started.
 * @property initializationCompletedListener listener to notify when this detector completes
 * initialization after being started.
 * @property errorListener listener to notify errors such as sudden motion during initialization or
 * sensor unreliability.
 * @property staticIntervalDetectedListener listener to notify when a new static interval is
 * detected.
 * @property dynamicIntervalDetectedListener listener to notify when a new dynamic interval is
 * detected.
 * @property resetListener listener to notify when a reset occurs.
 * @property measurementListener listener to notify collected magnetometer measurements.
 * @property accuracyChangedListener listener to notify when magnetometer accuracy changes.
 */
class MagnetometerIntervalDetector(
    val context: Context,
    val sensorType: MagnetometerSensorCollector.SensorType =
        MagnetometerSensorCollector.SensorType.MAGNETOMETER,
    val sensorDelay: SensorDelay = SensorDelay.FASTEST,
    var initializationStartedListener: OnInitializationStartedListener? = null,
    var initializationCompletedListener: OnInitializationCompletedListener? = null,
    var errorListener: OnErrorListener? = null,
    var staticIntervalDetectedListener: OnStaticIntervalDetectedListener? = null,
    var dynamicIntervalDetectedListener: OnDynamicIntervalDetectedListener? = null,
    var resetListener: OnResetListener? = null,
    var measurementListener: MagnetometerSensorCollector.OnMeasurementListener? = null,
    var accuracyChangedListener: SensorCollector.OnAccuracyChangedListener? = null
) {

    /**
     * Listener for internal interval detector.
     */
    private val internalDetectorListener =
        object : MagneticFluxDensityTriadStaticIntervalDetectorListener {
            override fun onInitializationStarted(
                detector: MagneticFluxDensityTriadStaticIntervalDetector?
            ) {
                initializationStartedListener?.onInitializationStarted(this@MagnetometerIntervalDetector)
            }

            override fun onInitializationCompleted(
                detector: MagneticFluxDensityTriadStaticIntervalDetector?,
                baseNoiseLevel: Double
            ) {
                initializationCompletedListener?.onInitializationCompleted(
                    this@MagnetometerIntervalDetector,
                    baseNoiseLevel
                )
            }

            override fun onError(
                detector: MagneticFluxDensityTriadStaticIntervalDetector?,
                accumulatedNoiseLevel: Double,
                instantaneousNoiseLevel: Double,
                reason: TriadStaticIntervalDetector.ErrorReason
            ) {
                stop()
                errorListener?.onError(this@MagnetometerIntervalDetector, mapErrorReason(reason))
            }

            override fun onStaticIntervalDetected(
                detector: MagneticFluxDensityTriadStaticIntervalDetector?,
                instantaneousAvgX: Double,
                instantaneousAvgY: Double,
                instantaneousAvgZ: Double,
                instantaneousStdX: Double,
                instantaneousStdY: Double,
                instantaneousStdZ: Double
            ) {
                staticIntervalDetectedListener?.onStaticIntervalDetected(
                    this@MagnetometerIntervalDetector,
                    instantaneousAvgX,
                    instantaneousAvgY,
                    instantaneousAvgZ,
                    instantaneousStdX,
                    instantaneousStdY,
                    instantaneousStdZ
                )
            }

            override fun onDynamicIntervalDetected(
                detector: MagneticFluxDensityTriadStaticIntervalDetector?,
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
                dynamicIntervalDetectedListener?.onDynamicIntervalDetected(
                    this@MagnetometerIntervalDetector,
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

            override fun onReset(detector: MagneticFluxDensityTriadStaticIntervalDetector?) {
                resetListener?.onReset(this@MagnetometerIntervalDetector)
            }
        }

    /**
     * Internal interval detector.
     * Processes magnetometer measurements and detects static and dynamic intervals.
     */
    private val internalDetector =
        MagneticFluxDensityTriadStaticIntervalDetector(internalDetectorListener)

    /**
     * Internal time estimator.
     * This can be used to estimate statistics about time intervals of measurements.
     */
    private val timeIntervalEstimator = TimeIntervalEstimator()

    /**
     * Timestamp when detector started.
     */
    private var initialTimestamp: Long = 0L

    /**
     * Internal listener for magnetometer sensor collector.
     * Handles measurements collected by the sensor collector so that they are processed by
     * the internal interval detector.
     */
    private val internalMeasurementListener =
        MagnetometerSensorCollector.OnMeasurementListener { bx, by, bz, hardIronX, hardIronY, hardIronZ, timestamp, accuracy ->
            val status = status
            if (status == Status.INITIALIZING) {
                // during initialization phase, also estimate time interval duration.
                if (numberOfProcessedMeasurements > 0) {
                    val diff = timestamp - initialTimestamp
                    val diffSeconds = TimeConverter.nanosecondToSecond(diff.toDouble())
                    timeIntervalEstimator.addTimestamp(diffSeconds)
                } else {
                    initialTimestamp = timestamp
                }
            }

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

            internalDetector.process(bxT, byT, bzT)
            numberOfProcessedMeasurements++

            if (status == Status.INITIALIZATION_COMPLETED) {
                // once initialized, set time interval into internal detector
                internalDetector.timeInterval = timeIntervalEstimator.averageTimeInterval
                initialized = true
            }

            measurementListener?.onMeasurement(
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

    /**
     * Listener to detect when accuracy of accelerometer changes.
     * When accelerometer becomes unreliable, an error is notified.
     */
    private val internalAccuracyChangedListener =
        SensorCollector.OnAccuracyChangedListener { accuracy ->
            if (accuracy == SensorAccuracy.UNRELIABLE) {
                stop()
                unreliable = true
                errorListener?.onError(
                    this@MagnetometerIntervalDetector,
                    ErrorReason.UNRELIABLE_SENSOR
                )
            }

            accuracyChangedListener?.onAccuracyChanged(accuracy)
        }

    /**
     * Magnetometer sensor collector.
     * Collects magnetometer measurements.
     */
    private val collector = MagnetometerSensorCollector(
        context,
        sensorType,
        sensorDelay,
        internalMeasurementListener,
        internalAccuracyChangedListener
    )

    /**
     * Indicates whether magnetometer sensor has become unreliable, and thus
     * the interval detector is considered to be in [Status.FAILED].
     */
    private var unreliable = false

    /**
     * Indicates whether detector successfully completed initialization.
     */
    private var initialized: Boolean = false

    /**
     * Gets sensor being used to obtain magnetometer measurements, or null if not available.
     * This can be used to obtain additional information about the sensor.
     */
    val sensor
        get() = collector.sensor

    /**
     * Gets or sets length of number of samples to keep within the window being processed to
     * determine instantaneous magnetometer noise level. Window size must always be larger than
     * allowed minimum value, which is 2 and must have an odd value.
     *
     * @throws IllegalArgumentException if provided value is not valid.
     * @throws IllegalStateException if detector is currently running.
     */
    var windowSize
        get() = internalDetector.windowSize
        @Throws(IllegalArgumentException::class, IllegalStateException::class)
        set(value) {
            check(!running)
            internalDetector.windowSize = value
        }

    /**
     * Gets or sets number of samples to be processed initially while keeping the sensor static in
     * order to find the base noise level when device is static.
     *
     * @throws IllegalArgumentException if provided value is less than
     * [TriadStaticIntervalDetector.MINIMUM_INITIAL_STATIC_SAMPLES].
     * @throws IllegalStateException if detector is currently running.
     */
    var initialStaticSamples
        get() = internalDetector.initialStaticSamples
        @Throws(IllegalArgumentException::class, IllegalStateException::class)
        set(value) {
            check(!running)
            internalDetector.initialStaticSamples = value
        }

    /**
     * Gets or sets factor to be applied to detected base noise level in order to determine
     * threshold for static/dynamic period changes. This factor is unit-less.
     *
     * @throws IllegalArgumentException if provided value is zero or negative
     * @throws IllegalStateException if detector is currently running.
     */
    var thresholdFactor
        get() = internalDetector.thresholdFactor
        @Throws(IllegalArgumentException::class, IllegalStateException::class)
        set(value) {
            check(!running)
            internalDetector.thresholdFactor = value
        }

    /**
     * Gets or sets factor to determine that a sudden movement has occurred during initialization if
     * instantaneous noise level exceeds accumulated noise level by this factor amount. This factor
     * is unit-less.
     *
     * @throws IllegalArgumentException if provided value is zero or negative.
     * @throws IllegalStateException if detector is currently running
     */
    var instantaneousNoiseLevelFactor
        get() = internalDetector.instantaneousNoiseLevelFactor
        @Throws(IllegalArgumentException::class, IllegalStateException::class)
        set(value) {
            check(!running)
            internalDetector.instantaneousNoiseLevelFactor = value
        }

    /**
     * Gets or sets overall absolute threshold to determine whether there has been excessive
     * magnetic field noise during the whole initialization phase. Failure will be detected if
     * estimated base noise level exceeds this threshold when initialization completes. This
     * threshold is expressed in Teslas (T).
     *
     * @throws IllegalArgumentException if provided value is zero or negative.
     * @throws IllegalStateException if detector is currently running.
     */
    var baseNoiseLevelAbsoluteThreshold
        get() = internalDetector.baseNoiseLevelAbsoluteThreshold
        @Throws(IllegalArgumentException::class, IllegalStateException::class)
        set(value) {
            check(!running)
            internalDetector.baseNoiseLevelAbsoluteThreshold = value
        }

    /**
     * Gets or sets overall absolute threshold to determine whether there has been excessive
     * magnetic field noise during the whole initialization phase. Failure will be detected if
     * estimated base noise level exceeds this threshold when initialization completes.
     *
     * @throws IllegalArgumentException if provided value is zero or negative.
     * @throws IllegalStateException if detector is currently running.
     */
    var baseNoiseLevelAbsoluteThresholdAsMagneticFluxDensity: MagneticFluxDensity
        get() = internalDetector.baseNoiseLevelAbsoluteThresholdAsMeasurement
        @Throws(IllegalArgumentException::class, IllegalStateException::class)
        set(value) {
            check(!running)
            internalDetector.setBaseNoiseLevelAbsoluteThreshold(value)
        }

    /**
     * Gets overall absolute threshold to determine whether there has been excessive motion during
     * the whole initialization phase. Failure will be detected if estimated base noise level
     * exceeds this threshold when initialization completes.
     *
     * @param result instance where result will be stored.
     */
    fun getBaseNoiseLevelAbsoluteThresholdAsMagneticFluxDensity(result: MagneticFluxDensity) {
        internalDetector.getBaseNoiseLevelAbsoluteThresholdAsMeasurement(result)
    }

    /**
     * Gets magnetometer measurement base noise level that has been detected during initialization
     * expressed in Teslas (T).
     * This is only available once detector completes initialization.
     */
    val baseNoiseLevel
        get() = if (initialized) {
            internalDetector.baseNoiseLevel
        } else {
            null
        }

    /**
     * Gets magnetometer measurement base noise level that has been detected during initialization.
     * This is only available once detector completes initialization.
     */
    val baseNoiseLevelAsMagneticFluxDensity
        get() = if (initialized) {
            internalDetector.baseNoiseLevelAsMeasurement
        } else {
            null
        }

    /**
     * Gets magnetometer measurement base noise level that has been detected during initialization.
     * This is only available once detector completes initialization.
     *
     * @param result instance where result will be stored.
     * @return true if result is available, false otherwise.
     */
    fun getBaseNoiseLevelAsMagneticFluxDensity(result: MagneticFluxDensity): Boolean {
        return if (initialized) {
            internalDetector.getBaseNoiseLevelAsMeasurement(result)
            true
        } else {
            false
        }
    }

    /**
     * Gets measurement base noise level PSD (Power Spectral Density) expressed in T^2 * s).
     * This is only available once detector completes initialization.
     */
    val baseNoiseLevelPsd
        get() = if (initialized) {
            internalDetector.baseNoiseLevelPsd
        } else {
            null
        }

    /**
     * Gets measurement base noise level root PSD (Power Spectral Density) expressed in T * s^0.5.
     * This is only available once detector completes initialization.
     */
    val baseNoiseLevelRootPsd
        get() = if (initialized) {
            internalDetector.baseNoiseLevelRootPsd
        } else {
            null
        }

    /**
     * Gets estimated threshold to determine static/dynamic period changes expressed in Teslas (T).
     * This is only available once detector completes initialization.
     */
    val threshold
        get() = if (initialized) {
            internalDetector.threshold
        } else {
            null
        }

    /**
     * Gets estimated threshold to determine static/dynamic period changes.
     * This is only available once detector completes initialization.
     */
    val thresholdAsMagneticFluxDensity
        get() = if (initialized) {
            internalDetector.thresholdAsMeasurement
        } else {
            null
        }

    /**
     * Gets estimated threshold to determine static/dynamic period changes.
     * This is only available once detector completes initialization.
     *
     * @param result instance where result will be stored.
     * @return true if result is available, false otherwise.
     */
    fun getThresholdAsMeasurement(result: MagneticFluxDensity): Boolean {
        return if (initialized) {
            internalDetector.getThresholdAsMeasurement(result)
            true
        } else {
            false
        }
    }

    /**
     * Gets average x-coordinate of magnetometer measurements accumulated during last static
     * period expressed in Teslas (T).
     * This value is updated when switching from a static period to a dynamic one, or after
     * initialization completes.
     */
    val accumulatedAvgX
        get() = internalDetector.accumulatedAvgX

    /**
     * Gets average x-coordinate of magnetometer measurements accumulated during last static
     * period.
     * This value is updated when switching from a static period to a dynamic one, or after
     * initialization completes.
     */
    val accumulatedAvgXAsMagneticFluxDensity: MagneticFluxDensity
        get() = internalDetector.accumulatedAvgXAsMeasurement

    /**
     * Gets average x-coordinate of magnetometer measurements accumulated during last static
     * period.
     * This value is updated when switching from a static period to a dynamic one, or after
     * initialization completes.
     *
     * @param result instance where result will be stored.
     */
    fun getAccumulatedAvgXAsMagneticFluxDensity(result: MagneticFluxDensity) {
        internalDetector.getAccumulatedAvgXAsMeasurement(result)
    }

    /**
     * Gets average y-coordinate of magnetometer measurements accumulated during last static
     * period expressed in Teslas (T).
     * This value is updated when switching from a static period to a dynamic one, or after
     * initialization completes.
     */
    val accumulatedAvgY
        get() = internalDetector.accumulatedAvgY

    /**
     * Gets average y-coordinate of magnetometer measurements accumulated during last static
     * period.
     * This value is updated when switching from a static period to a dynamic one, or after
     * initialization completes.
     */
    val accumulatedAvgYAsMagneticFluxDensity: MagneticFluxDensity
        get() = internalDetector.accumulatedAvgYAsMeasurement

    /**
     * Gets average y-coordinate of magnetometer measurements accumulated during last static
     * period.
     * This value is updated when switching from a static period to a dynamic one, or after
     * initialization completes.
     *
     * @param result instance where result will be stored.
     */
    fun getAccumulatedAvgYAsMagneticFluxDensity(result: MagneticFluxDensity) {
        internalDetector.getAccumulatedAvgYAsMeasurement(result)
    }

    /**
     * Gets average z-coordinate of magnetometer measurements accumulated during last static
     * period expressed in Teslas (T).
     * This value is updated when switching from a static period to a dynamic one, or after
     * initialization completes.
     */
    val accumulatedAvgZ
        get() = internalDetector.accumulatedAvgZ

    /**
     * Gets average z-coordinate of magnetometer measurements accumulated during last static
     * period.
     * This value is updated when switching from a static period to a dynamic one, or after
     * initialization completes.
     */
    val accumulatedAvgZAsMagneticFluxDensity: MagneticFluxDensity
        get() = internalDetector.accumulatedAvgZAsMeasurement

    /**
     * Gets average z-coordinate of magnetometer measurements accumulated during last static
     * period.
     * This value is updated when switching from a static period to a dynamic one, or after
     * initialization completes.
     *
     * @param result instance where result will be stored.
     */
    fun getAccumulatedAvgZAsMagneticFluxDensity(result: MagneticFluxDensity) {
        internalDetector.getAccumulatedAvgZAsMeasurement(result)
    }

    /**
     * Gets average magnetometer measurements triad accumulated during last static period.
     * This value is updated when switching from a static period to a dynamic one or after
     * initialization completes.
     */
    val accumulatedAvgTriad: MagneticFluxDensityTriad
        get() = internalDetector.accumulatedAvgTriad

    /**
     * Gets average magnetometer measurements triad accumulated during last static period.
     * This value is updated when switching from a static period to a dynamic one or after
     * initialization completes.
     *
     * @param result instance where result will be stored.
     */
    fun getAccumulatedAvgTriad(result: MagneticFluxDensityTriad) {
        internalDetector.getAccumulatedAvgTriad(result)
    }

    /**
     * Gets standard deviation of x-coordinate of magnetometer measurements accumulated during last
     * static period expressed in Teslas (T).
     * This value is updated when switching from a static period to a dynamic one or after
     * completing initialization.
     */
    val accumulatedStdX
        get() = internalDetector.accumulatedStdX

    /**
     * Gets standard deviation of x-coordinate of magnetometer measurements accumulated during last
     * static period.
     * This value is updated when switching from a static period to a dynamic one or after
     * completing initialization.
     */
    val accumulatedStdXAsMagneticFluxDensity: MagneticFluxDensity
        get() = internalDetector.accumulatedStdXAsMeasurement

    /**
     * Gets standard deviation of x-coordinate of magnetometer measurements accumulated during last
     * static period.
     * This value is updated when switching from a static period to a dynamic one or after
     * completing initialization.
     *
     * @param result instance where result will be stored.
     */
    fun getAccumulatedStdXAsMagneticFluxDensity(result: MagneticFluxDensity) {
        internalDetector.getAccumulatedStdXAsMeasurement(result)
    }

    /**
     * Gets standard deviation of y-coordinate of magnetometer measurements accumulated during last
     * static period expressed in Teslas (T).
     * This value is updated when switching from a static period to a dynamic one or after
     * completing initialization.
     */
    val accumulatedStdY
        get() = internalDetector.accumulatedStdY

    /**
     * Gets standard deviation of y-coordinate of magnetometer measurements accumulated during last
     * static period.
     * This value is updated when switching from a static period to a dynamic one or after
     * completing initialization.
     */
    val accumulatedStdYAsMagneticFluxDensity: MagneticFluxDensity
        get() = internalDetector.accumulatedStdYAsMeasurement

    /**
     * Gets standard deviation of y-coordinate of magnetometer measurements accumulated during last
     * static period.
     * This value is updated when switching from a static period to a dynamic one or after
     * completing initialization.
     *
     * @param result instance where result will be stored.
     */
    fun getAccumulatedStdYAsMagneticFluxDensity(result: MagneticFluxDensity) {
        internalDetector.getAccumulatedStdYAsMeasurement(result)
    }

    /**
     * Gets standard deviation of z-coordinate of magnetometer measurements accumulated during last
     * static period expressed in Teslas (T).
     * This value is updated when switching from a static period to a dynamic one or after
     * completing initialization.
     */
    val accumulatedStdZ
        get() = internalDetector.accumulatedStdZ

    /**
     * Gets standard deviation of z-coordinate of magnetometer measurements accumulated during last
     * static period.
     * This value is updated when switching from a static period to a dynamic one or after
     * completing initialization.
     */
    val accumulatedStdZAsMagneticFluxDensity: MagneticFluxDensity
        get() = internalDetector.accumulatedStdZAsMeasurement

    /**
     * Gets standard deviation of z-coordinate of magnetometer measurements accumulated during last
     * static period.
     * This value is updated when switching from a static period to a dynamic one or after
     * completing initialization.
     *
     * @param result instance where result will be stored.
     */
    fun getAccumulatedStdZAsMagneticFluxDensity(result: MagneticFluxDensity) {
        internalDetector.getAccumulatedStdZAsMeasurement(result)
    }

    /**
     * Gets standard deviation of magnetometer measurements accumulated during last static period.
     * This value is updated when switching from a static period to a dynamic one or after
     * completing initialization.
     */
    val accumulatedStdTriad: MagneticFluxDensityTriad
        get() = internalDetector.accumulatedStdTriad

    /**
     * Gets standard deviation of magnetometer measurements accumulated during last static period.
     * This value is updated when switching from a static period to a dynamic one or after
     * completing initialization.
     *
     * @param result instace where result will be stored.
     */
    fun getAccumulatedStdTriad(result: MagneticFluxDensityTriad) {
        internalDetector.getAccumulatedStdTriad(result)
    }

    /**
     * Gets windowed average x-coordinate of magnetometer measurements for each processed triad
     * expressed in Teslas (T).
     * This value is updated for each processed sample containing an average value for the sample
     * within the window.
     */
    val instantaneousAvgX
        get() = internalDetector.instantaneousAvgX

    /**
     * Gets windowed average x-coordinate of magnetometer measurements for each processed triad.
     * This value is updated for each processed sample containing an average value for the sample
     * within the window.
     */
    val instantaneousAvgXAsMagneticFluxDensity: MagneticFluxDensity
        get() = internalDetector.instantaneousAvgXAsMeasurement

    /**
     * Gets windowed average x-coordinate of magnetometer measurements for each processed triad.
     * This value is udpated for each processed sample containing an average value for the sample
     * within the window.
     *
     * @param result instance where result will be stored.
     */
    fun getInstantaneousAvgXAsMagneticFluxDensity(result: MagneticFluxDensity) {
        internalDetector.getInstantaneousAvgXAsMeasurement(result)
    }

    /**
     * Gets windowed average y-coordinate of magnetometer measurements for each processed triad
     * expressed in Teslas (T).
     * This value is updated for each processed sample containing an average value for the sample
     * within the wnidow.
     */
    val instantaneousAvgY
        get() = internalDetector.instantaneousAvgY

    /**
     * Gets windowed average y-coordinate of magnetometer measurements for each processed triad.
     * This value is updated for each processed sample containing an average value for the sample
     * within the window.
     */
    val instantaneousAvgYAsMagneticFluxDensity: MagneticFluxDensity
        get() = internalDetector.instantaneousAvgYAsMeasurement

    /**
     * Gets windowed average y-coordinate of magnetometer measurements for each processed triad.
     * This value is updated for each processed sample containing an average value for the sample
     * within the window.
     *
     * @param result instance where result will be stored.
     */
    fun getInstantaneousAvgYAsMagneticFluxDensity(result: MagneticFluxDensity) {
        internalDetector.getInstantaneousAvgYAsMeasurement(result)
    }

    /**
     * Gets windowed average z-coordinate of magnetometer measurements for each processed triad
     * expressed in Teslas (T).
     * This value is updated for each processed sample containing an average value for the sample
     * within the window.
     */
    val instantaneousAvgZ
        get() = internalDetector.instantaneousAvgZ

    /**
     * Gets windowed average z-coordinate of magnetometer measurements for each processed triad.
     * This value is updated for each processed sample containing an average value for the sample
     * within the window.
     */
    val instantaneousAvgZAsMagneticFluxDensity: MagneticFluxDensity
        get() = internalDetector.instantaneousAvgZAsMeasurement

    /**
     * Gets windowed average z-coordinate of magnetometer measurements for each processed triad.
     * This value is updated for each processed sample containing an average value for the sample
     * within the window.
     *
     * @param result instance where result will be stored.
     */
    fun getInstantaneousAvgZAsMagneticFluxDensity(result: MagneticFluxDensity) {
        internalDetector.getInstantaneousAvgZAsMeasurement(result)
    }

    /**
     * Gets windowed average of magnetometer measurements for each processed triad.
     * This value is updated for each processed sample containing an average value for the samples
     * within the window.
     */
    val instantaneousAvgTriad: MagneticFluxDensityTriad
        get() = internalDetector.instantaneousAvgTriad

    /**
     * Gets windowed average of magnetometer measurements for each processed triad.
     * This value is updated for each processed sample containing an average value for the samples
     * within the window.
     *
     * @param result instance where result will be stored.
     */
    fun getInstantaneousAvgTriad(result: MagneticFluxDensityTriad) {
        internalDetector.getInstantaneousAvgTriad(result)
    }

    /**
     * Gets windowed standard deviation of x-coordinate of magnetometer measurements for each
     * processed triad expressed in Teslas (T).
     * This value is updated for each processed sample containing measured standard deviation for
     * the samples within the window.
     */
    val instantaneousStdX
        get() = internalDetector.instantaneousStdX

    /**
     * Gets windowed standard deviation of x-coordinate of magnetometer measurements for each
     * processed triad.
     * This value is updated for each processed sample containing measured standard deviation for
     * the samples within the window.
     */
    val instantaneousStdXAsMagneticFluxDensity: MagneticFluxDensity
        get() = internalDetector.instantaneousStdXAsMeasurement

    /**
     * Gets windowed standard deviation of x-coordinate of magnetometer measurements for each
     * processed triad.
     * This value is updated for each processed sample containing measured standard deviation for
     * the samples within the window.
     *
     * @param result instance where result will be stored.
     */
    fun getInstantaneousStdXAsMagneticFluxDensity(result: MagneticFluxDensity) {
        internalDetector.getInstantaneousStdXAsMeasurement(result)
    }

    /**
     * Gets windowed standard deviation of y-coordinate of magnetometer measurements for each
     * processed triad expressed in Teslas (T).
     * This value is updated for each processed sample containing measured standard deviation for
     * the samples within the window.
     */
    val instantaneousStdY
        get() = internalDetector.instantaneousStdY

    /**
     * Gets windowed standard deviation of y-coordinate of magnetometer measurements for each
     * processed triad.
     * This value is updated for each processed sample containing measured standard deviation for
     * the samples within the window.
     */
    val instantaneousStdYAsMagneticFluxDensity: MagneticFluxDensity
        get() = internalDetector.instantaneousStdYAsMeasurement

    /**
     * Gets windowed standard deviation of y-coordinate of magnetometer measurements for each
     * processed triad.
     * This value is updated for each processed sample containing measured standard deviation for
     * the samples within the window.
     *
     * @param result instance where result will be stored.
     */
    fun getInstantaneousStdYAsMagneticFluxDensity(result: MagneticFluxDensity) {
        internalDetector.getInstantaneousStdYAsMeasurement(result)
    }

    /**
     * Gets windowed standard deviation of z-coordinate of magnetometer measurements for each
     * processed triad expressed in Teslas (T).
     * This value is updated for each processed sample containing measured standard deviation for
     * the samples within the window.
     */
    val instantaneousStdZ
        get() = internalDetector.instantaneousStdZ

    /**
     * Gets windowed standard deviation of z-coordinate of magnetometer measurements for each
     * processed triad.
     * This value is updated for each processed sample containing measured standard deviation for
     * the samples within the window.
     */
    val instantaneousStdZAsMagneticFluxDensity: MagneticFluxDensity
        get() = internalDetector.instantaneousStdZAsMeasurement

    /**
     * Gets windowed standard deviation of z-coordinate of magnetometer measurements for each
     * processed triad.
     * This value is updated for each processed sample containing measured standard deviation for
     * the samples within the window.
     *
     * @param result instance where result will be stored.
     */
    fun getInstantaneousStdZAsMagneticFluxDensity(result: MagneticFluxDensity) {
        internalDetector.getInstantaneousStdZAsMeasurement(result)
    }

    /**
     * Gets windowed standard deviation of magnetometer measurements for each processed triad.
     * This value is updated for each processed sample containing measured standard deviation for
     * the samples within the window.
     */
    val instantaneousStdTriad: MagneticFluxDensityTriad
        get() = internalDetector.instantaneousStdTriad

    /**
     * Gets windowed standard deviation of magnetometer measurements for each processed triad.
     * This valus is updated for each processed sample containing measured standard deviation for
     * the samples within the window.
     *
     * @param result instance where result will be stored.
     */
    fun getInstantaneousStdTriad(result: MagneticFluxDensityTriad) {
        internalDetector.getInstantaneousStdTriad(result)
    }

    /**
     * Gets average time interval between magnetometer samples expressed in seconds (s).
     * This is only available once detector completes initialization.
     */
    val averageTimeInterval
        get() = if (initialized) {
            timeIntervalEstimator.averageTimeInterval
        } else {
            null
        }

    /**
     * Gets average time interval between magnetometer samples.
     * This is only available once detector completes initialization.
     */
    val averageTimeIntervalAsTime
        get() = if (initialized) {
            timeIntervalEstimator.averageTimeIntervalAsTime
        } else {
            null
        }

    /**
     * Gets average time interval between measurements.
     * This is only available once detector completes initialization.
     *
     * @param result instance where result will be stored.
     * @return true if result is available, false otherwise.
     */
    fun getAverageTimeIntervalAsTime(result: Time): Boolean {
        return if (initialized) {
            timeIntervalEstimator.getAverageTimeIntervalAsTime(result)
            true
        } else {
            false
        }
    }

    /**
     * Gets estimated variance of time interval between measurements expressed in squared
     * seconds (s^2).
     * This is only available once detector completes initialization.
     */
    val timeIntervalVariance
        get() = if (initialized) {
            timeIntervalEstimator.timeIntervalVariance
        } else {
            null
        }

    /**
     * Gets estimated standard deviation of time interval between measurements expressed in
     * second (s).
     * This is only available once detector completes initialization.
     */
    val timeIntervalStandardDeviation
        get() = if (initialized) {
            timeIntervalEstimator.timeIntervalStandardDeviation
        } else {
            null
        }

    /**
     * Gets estimated standard deviation of time interval between measurements.
     * This is only available once detector completes initialization.
     */
    val timeIntervalStandardDeviationAsTime
        get() = if (initialized) {
            timeIntervalEstimator.timeIntervalStandardDeviationAsTime
        } else {
            null
        }

    /**
     * Gets estimated standard deviation of time interval between measurements.
     * This is only available once detector completes initialization.
     *
     * @param result instance where result will be stored.
     * @return true if result is available, false otherwise.
     */
    fun getTimeIntervalStandardDeviationAsTime(result: Time): Boolean {
        return if (initialized) {
            timeIntervalEstimator.getTimeIntervalStandardDeviationAsTime(result)
            true
        } else {
            false
        }
    }

    /**
     * Number of measurements that have been processed.
     */
    var numberOfProcessedMeasurements: Int = 0
        private set

    /**
     * Indicates whether this detector is already running.
     */
    var running = false
        private set

    /**
     * Gets status of interval detector.
     * Initially the inverval detector will be idle.
     * Once it starts, it will start the initialization phase, and once
     * initialization is complete, it will switch between static or dynamic interval
     * until detector is stopped or an error occurs.
     */
    val status: Status
        get() {
            return if (unreliable) {
                Status.FAILED
            } else {
                when (internalDetector.status) {
                    TriadStaticIntervalDetector.Status.IDLE -> Status.IDLE
                    TriadStaticIntervalDetector.Status.INITIALIZING -> Status.INITIALIZING
                    TriadStaticIntervalDetector.Status.INITIALIZATION_COMPLETED -> Status.INITIALIZATION_COMPLETED
                    TriadStaticIntervalDetector.Status.STATIC_INTERVAL -> Status.STATIC_INTERVAL
                    TriadStaticIntervalDetector.Status.DYNAMIC_INTERVAL -> Status.DYNAMIC_INTERVAL
                    TriadStaticIntervalDetector.Status.FAILED -> Status.FAILED
                    else -> Status.IDLE
                }
            }
        }

    /**
     * Starts collection of sensor measurements.
     *
     * @throws IllegalStateException if detector is already running or sensor is not available.
     */
    @Throws(IllegalStateException::class)
    fun start() {
        check(!running)

        reset()

        running = true
        if (!collector.start()) {
            throw IllegalStateException("Unavailable sensor")
        }
    }

    /**
     * Stops collection of sensor measurements.
     */
    fun stop() {
        collector.stop()
        running = false
    }

    /**
     * Resets detector to its initial state.
     */
    private fun reset() {
        timeIntervalEstimator.totalSamples = Integer.MAX_VALUE
        timeIntervalEstimator.reset()
        internalDetector.reset()
        unreliable = false
        initialTimestamp = 0L
        numberOfProcessedMeasurements = 0
        initialized = false
    }

    /**
     * Maps error reason to an [ErrorReason].
     *
     * @param reason reason to map from.
     * @return mapped reason.
     */
    private fun mapErrorReason(reason: TriadStaticIntervalDetector.ErrorReason): ErrorReason {
        return if (unreliable) {
            ErrorReason.UNRELIABLE_SENSOR
        } else {
            return when (reason) {
                TriadStaticIntervalDetector.ErrorReason.OVERALL_EXCESSIVE_MOVEMENT_DETECTED ->
                    ErrorReason.OVERALL_EXCESSIVE_MOVEMENT_DETECTED_DURING_INITIALIZATION
                TriadStaticIntervalDetector.ErrorReason.SUDDEN_EXCESSIVE_MOVEMENT_DETECTED ->
                    ErrorReason.SUDDEN_EXCESSIVE_MOVEMENT_DETECTED_DURING_INITIALIZATION
            }
        }
    }

    /**
     * Interface to notify when detector starts initialization.
     */
    fun interface OnInitializationStartedListener {
        /**
         * Called when initial static period starts so that base noise level starts being estimated.
         *
         * @param detector detector that raise dthe event.
         */
        fun onInitializationStarted(detector: MagnetometerIntervalDetector)
    }

    /**
     * Interface to notify when detector completes its initialization.
     */
    fun interface OnInitializationCompletedListener {
        /**
         * Called when initial static period successfully completes and base noise level is
         * estimated so that static and dynamic periods can be detected.
         *
         * @param detector detector that raised the event.
         * @param baseNoiseLevel base measurement noise level expressed in Teslas (T).
         */
        fun onInitializationCompleted(
            detector: MagnetometerIntervalDetector,
            baseNoiseLevel: Double
        )
    }

    /**
     * Interface to notify when an error occurs.
     */
    fun interface OnErrorListener {
        /**
         * Called when an error is detected, either at initialization because excessive changes in
         * magnetic field are detected, or because sensor becomes unreliable.
         * When an error occurs, detector is stopped and needs to be restarted to be used again.
         *
         * @param detector detector that raised the event.
         * @param reason reason why error was detected.
         */
        fun onError(detector: MagnetometerIntervalDetector, reason: ErrorReason)
    }

    /**
     * Interface to notify when a new static interval is detected.
     * Instantaneous average measurements and standard deviation within the averaging window are
     * provided at the time the new static interval is detected (and consequently detector ends
     * a previous dynamic interval or its initialization stage).
     */
    fun interface OnStaticIntervalDetectedListener {
        /**
         * Called when a static interval has been detected after initialization.
         *
         * @param detector detector that raised the event.
         * @param instantaneousAvgX instantaneous average x-coordinate of measurements within the
         * window expressed in Teslas (T).
         * @param instantaneousAvgY instantaneous average y-coordinate of measurements within the
         * window expressed in Teslas (T).
         * @param instantaneousAvgZ instantaneous average z-coordinate of measurements within the
         * window expressed in Teslas (T).
         * @param instantaneousStdX instantaneous standard deviation of x-coordinate measurements
         * within the window expressed in Teslas (T).
         * @param instantaneousStdY instantaneous standard deviation of y-coordinate measurements
         * within the window expressed in Teslas (T).
         * @param instantaneousStdZ instantaneous standard deviation of z-coordinate measurements
         * within the window expressed in Teslas (T).
         */
        fun onStaticIntervalDetected(
            detector: MagnetometerIntervalDetector,
            instantaneousAvgX: Double,
            instantaneousAvgY: Double,
            instantaneousAvgZ: Double,
            instantaneousStdX: Double,
            instantaneousStdY: Double,
            instantaneousStdZ: Double
        )
    }

    /**
     * Interface to notify when a dynamic interval is detected.
     * Accumulated average measurements and standard deviations through the complete previous
     * static or initialization interval are provided, along with the windowed average and standard
     * deviation of measurements at the time the new dynamic interval is detected (where the
     * detector ends a previous static interval or initialization stage).
     */
    fun interface OnDynamicIntervalDetectedListener {
        /**
         * Called when a dynamic interval has been detected after initialization.
         *
         * @param detector detector that raised the event.
         * @param instantaneousAvgX instantaneous average x-coordinate of measurements within the
         * window expressed in Teslas (T).
         * @param instantaneousAvgY instantaneous average y-coordinate of measurements within the
         * window expressed in Teslas (T).
         * @param instantaneousAvgZ instantaneous average z-coordinate of measurements within the
         * window expressed in Teslas (T).
         * @param instantaneousStdX instantaneous x-coordinate of standard deviation for
         * measurements within the window expressed in Teslas (T).
         * @param instantaneousStdY instantaneous y-coordinate of standard deviation for
         * measurements within the window expressed in Teslas (T).
         * @param instantaneousStdZ instantaneous z-coordinate of standard dviation for
         * measurements within the window expressed in Teslas (T).
         * @param accumulatedAvgX accumulated average x-coordinate of measurements during last
         * static period expressed in Teslas (T).
         * @param accumulatedAvgY accumulated average y-coordinate of measurements during last
         * static period expressed in Teslas (T).
         * @param accumulatedAvgZ accumulated average z-coordinate of measurements during last
         * static period expressed in Teslas (T).
         * @param accumulatedStdX standard deviation of x-coordinate of accumulated measurements
         * during last static period expressed in Teslas (T).
         * @param accumulatedStdY standard deviation of y-coordinate of accumulated measurements
         * during last static period expressed in Teslas (T).
         * @param accumulatedStdZ standard deviation of z-coordinate of accumulated measurements
         * during last static period expressed in Teslas (T).
         */
        fun onDynamicIntervalDetected(
            detector: MagnetometerIntervalDetector,
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
        )
    }

    /**
     * Interface to notify when detector is reset (occurs when starting after stopping the
     * detector).
     */
    fun interface OnResetListener {
        /**
         * Called when detector is reset.
         *
         * @param detector detector that raised the event.
         */
        fun onReset(detector: MagnetometerIntervalDetector)
    }

    /**
     * Detector status values.
     */
    enum class Status {
        /**
         * Detector is in idle status when it hasn't processed any sample yet.
         */
        IDLE,

        /**
         * Detector is processing samples in the initial static interval to determine base noise
         * level.
         */
        INITIALIZING,

        /**
         * Detector has successfully completed processing samples on the initial static period.
         */
        INITIALIZATION_COMPLETED,

        /**
         * A static interval has been detected, where magnetometer is considered to sense have
         * sensed static magnetic field measurements.
         */
        STATIC_INTERVAL,

        /**
         * A dynamic interval has been detected, where magnetometer is considered to sense changing
         * magnetic field measurements.
         */
        DYNAMIC_INTERVAL,

        /**
         * Detector has failed. This happens if magnetometer senses sudden changes in magnetic field
         * while detector is initializing during the initial static period, if there is too much
         * overall noise during initialization, or if magnetometer sensor becomes unreliable.
         * When detector has failed, no new samples will be allowed to be processed until detector
         * is reset.
         */
        FAILED
    }

    /**
     * Reason why this detector has failed.
     */
    enum class ErrorReason {
        /**
         * If a sudden movement is detected during initialization.
         */
        SUDDEN_EXCESSIVE_MOVEMENT_DETECTED_DURING_INITIALIZATION,

        /**
         * If overall noise level is excessive during initialization.
         */
        OVERALL_EXCESSIVE_MOVEMENT_DETECTED_DURING_INITIALIZATION,

        /**
         * If sensor becomes unreliable.
         */
        UNRELIABLE_SENSOR
    }
}