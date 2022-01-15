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
import com.irurueta.android.navigation.inertial.collectors.AccelerometerSensorCollector
import com.irurueta.android.navigation.inertial.collectors.SensorAccuracy
import com.irurueta.android.navigation.inertial.collectors.SensorCollector
import com.irurueta.android.navigation.inertial.collectors.SensorDelay
import com.irurueta.navigation.inertial.calibration.AccelerationTriad
import com.irurueta.navigation.inertial.calibration.TimeIntervalEstimator
import com.irurueta.navigation.inertial.calibration.intervals.AccelerationTriadStaticIntervalDetector
import com.irurueta.navigation.inertial.calibration.intervals.AccelerationTriadStaticIntervalDetectorListener
import com.irurueta.navigation.inertial.calibration.intervals.TriadStaticIntervalDetector
import com.irurueta.units.Acceleration
import com.irurueta.units.Time
import com.irurueta.units.TimeConverter

/**
 * Detects static or motion intervals of time by using the accelerometer of the
 * device to determine whether the device is moving or not.
 * When detector is started, initialization occurs to determine the accelerometer noise level while
 * keeping device static. Once the detector is initialized, then static or dynamic intervals can be
 * detected.
 * This detector uses accumulated average values during static intervals, and windowed
 * averages as "instantaneous" values during dynamic intervals.
 * Length of windows, as well as thresholds to determine when changes between static and dynamic
 * intervals occur can be easily configured.
 *
 * @property context Android context.
 * @property sensorType One of the supported accelerometer sensor types.
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
 */
class IntervalDetector(
    val context: Context,
    val sensorType: AccelerometerSensorCollector.SensorType =
        AccelerometerSensorCollector.SensorType.ACCELEROMETER,
    val sensorDelay: SensorDelay = SensorDelay.FASTEST,
    var initializationStartedListener: OnInitializationStartedListener? = null,
    var initializationCompletedListener: OnInitializationCompletedListener? = null,
    var errorListener: OnErrorListener? = null,
    var staticIntervalDetectedListener: OnStaticIntervalDetectedListener? = null,
    var dynamicIntervalDetectedListener: OnDynamicIntervalDetectedListener? = null,
    var resetListener: OnResetListener? = null,
) {

    /**
     * Listener for internal interval detector.
     */
    private val internalDetectorListener =
        object : AccelerationTriadStaticIntervalDetectorListener {
            override fun onInitializationStarted(
                detector: AccelerationTriadStaticIntervalDetector?
            ) {
                initializationStartedListener?.onInitializationStarted(this@IntervalDetector)
            }

            override fun onInitializationCompleted(
                detector: AccelerationTriadStaticIntervalDetector?,
                baseNoiseLevel: Double
            ) {
                initializationCompletedListener?.onInitializationCompleted(
                    this@IntervalDetector,
                    baseNoiseLevel
                )
            }

            override fun onError(
                detector: AccelerationTriadStaticIntervalDetector?,
                accumulatedNoiseLevel: Double,
                instantaneousNoiseLevel: Double,
                reason: TriadStaticIntervalDetector.ErrorReason
            ) {
                stop()
                errorListener?.onError(this@IntervalDetector, mapErrorReason(reason))
            }

            override fun onStaticIntervalDetected(
                detector: AccelerationTriadStaticIntervalDetector?,
                instantaneousAvgX: Double,
                instantaneousAvgY: Double,
                instantaneousAvgZ: Double,
                instantaneousStdX: Double,
                instantaneousStdY: Double,
                instantaneousStdZ: Double
            ) {
                staticIntervalDetectedListener?.onStaticIntervalDetected(
                    this@IntervalDetector,
                    instantaneousAvgX,
                    instantaneousAvgY,
                    instantaneousAvgZ,
                    instantaneousStdX,
                    instantaneousStdY,
                    instantaneousStdZ
                )
            }

            override fun onDynamicIntervalDetected(
                detector: AccelerationTriadStaticIntervalDetector?,
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
                    this@IntervalDetector,
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

            override fun onReset(detector: AccelerationTriadStaticIntervalDetector?) {
                resetListener?.onReset(this@IntervalDetector)
            }

        }

    /**
     * Internal interval detector.
     * Processes accelerometer measurements and detects static and dynamic intervals.
     */
    private val internalDetector = AccelerationTriadStaticIntervalDetector(internalDetectorListener)

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
     * Listener for accelerometer sensor collector.
     * Handles measurements collected by the sensor collector so that they are processed by
     * the internal interval detector.
     */
    private val measurementListener = object : AccelerometerSensorCollector.OnMeasurementListener {
        override fun onMeasurement(
            ax: Float,
            ay: Float,
            az: Float,
            bx: Float?,
            by: Float?,
            bz: Float?,
            timestamp: Long,
            accuracy: SensorAccuracy?
        ) {
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

            internalDetector.process(ax.toDouble(), ay.toDouble(), az.toDouble())
            numberOfProcessedMeasurements++

            if (status == Status.INITIALIZATION_COMPLETED) {
                // once initialized, set time interval into internal detector
                internalDetector.timeInterval = timeIntervalEstimator.averageTimeInterval
                initialized = true
            }
        }
    }

    /**
     * Listener to detect when accuracy of accelerometer changes.
     * When accelerometer becomes unreliable, an error is notified.
     */
    private val accuracyChangedListener = object : SensorCollector.OnAccuracyChangedListener {
        override fun onAccuracyChanged(accuracy: SensorAccuracy?) {
            if (accuracy == SensorAccuracy.UNRELIABLE) {
                stop()
                unreliable = true
                errorListener?.onError(
                    this@IntervalDetector,
                    ErrorReason.UNRELIABLE_SENSOR
                )
            }
        }
    }

    /**
     * Accelerometer sensor collector.
     * Collects accelerometer measurements.
     */
    private val collector =
        AccelerometerSensorCollector(
            context,
            sensorType,
            sensorDelay,
            measurementListener,
            accuracyChangedListener
        )

    /**
     * Indicates whether accelerometer sensor has become unreliable, and thus
     * the interval detector is considered to be in [Status.FAILED].
     */
    private var unreliable = false

    /**
     * Indicates whether detector successfully completed initialization.
     */
    private var initialized: Boolean = false

    /**
     * Gets or sets length of number of samples to keep within the window being processed to
     * determine instantaneous accelerometer noise leel. Window size must always be larger than
     * allowed minimum value, which is 2 and must have and odd value.
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
     * Gets or sets overall absolute threshold to determine whether there has been excessive motion
     * during the whole initialization phase. Failure will be detected if estimated base noise level
     * exceeds this threshold when initialization completes. This threshold is expressed in meters
     * per squared second (m/s^2).
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
     * Gets or sets overall absolute threshold to determine whether there has been excessive motion
     * during the whole initialization phase. Failure will be detected if estimated base noise level
     * exceeds this threshold when initialization completes.
     *
     * @throws IllegalArgumentException if provided value is zero or negative.
     * @throws IllegalStateException if detector is currently running.
     */
    var baseNoiseLevelAbsoluteThresholdAsAcceleration: Acceleration
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
     */
    fun getBaseNoiseLevelAbsoluteThresholdAsAcceleration(result: Acceleration) {
        check(!running)
        internalDetector.getBaseNoiseLevelAbsoluteThresholdAsMeasurement(result)
    }

    /**
     * Gets accelerometer measurement base noise level that has been detected during initialization
     * expressed in meters per squared second (m/s^2).
     * This is only available once detector completes initialization.
     */
    val baseNoiseLevel
        get() = if (initialized) {
            internalDetector.baseNoiseLevel
        } else {
            null
        }

    /**
     * Gets accelerometer measurement base noise level that has been detected during initialization.
     * This is only available once detector completes initialization.
     */
    val baseNoiseLevelAsAcceleration
        get() = if (initialized) {
            internalDetector.baseNoiseLevelAsMeasurement
        } else {
            null
        }

    /**
     * Gets accelerometer measurement base noise level that has been detected during initialization.
     * This is only available once detector completes initialization.
     *
     * @param result instance where result will be stored.
     * @return true if result is available, false otherwise.
     */
    fun getBaseNoiseLevelAsAcceleration(result: Acceleration): Boolean {
        return if (initialized) {
            internalDetector.getBaseNoiseLevelAsMeasurement(result)
            true
        } else {
            false
        }
    }

    /**
     * Gets measurement base noise level PSD (Power Spectral Density) expressed in (m^2 * s^-3).
     * This is only available once detector completes initialization.
     */
    val baseNoiseLevelPsd
        get() = if (initialized) {
            internalDetector.baseNoiseLevelPsd
        } else {
            null
        }

    /**
     * Gets measurement base noise level root PSD (Power Spectral Density) expressed
     * in (m * s^-1.5).
     * This is only available once detector completes initialization.
     */
    val baseNoiseLevelRootPsd
        get() = if (initialized) {
            internalDetector.baseNoiseLevelRootPsd
        } else {
            null
        }

    /**
     * Gets estimated threshold to determine static/dynamic period changes expressed in meters per
     * squared second (m/s^2).
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
    val thresholdAsAcceleration
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
    fun getThresholdAsMeasurement(result: Acceleration): Boolean {
        return if (initialized) {
            internalDetector.getThresholdAsMeasurement(result)
            true
        } else {
            false
        }
    }

    /**
     * Gets average x-coordinate of accelerometer measurements accumulated during last static
     * period expressed in meters per squared second (m/s^2).
     * This value is updated when switching from a static period to a dynamic one, or after
     * initialization completes.
     */
    val accumulatedAvgX
        get() = internalDetector.accumulatedAvgX

    /**
     * Gets average x-coordinate of accelerometer measurements accumulated during last static
     * period.
     * This value is updated when switching from a static period to a dynamic one, or after
     * initialization completes.
     */
    val accumulatedAvgXAsAcceleration: Acceleration
        get() = internalDetector.accumulatedAvgXAsMeasurement

    /**
     * Gets average x-coordinate of accelerometer measurements accumulated during last static
     * period.
     * This value is updated when switching from a static period to a dynamic one, or after
     * initialization completes.
     *
     * @param result instance where result will be stored.
     */
    fun getAccumulatedAvgXAsAcceleration(result: Acceleration) {
        internalDetector.getAccumulatedAvgXAsMeasurement(result)
    }

    /**
     * Gets average y-coordinate of accelerometer measurements accumulated during last static
     * period expressed in meters per squared second (m/s^2).
     * This value is updated when switching from a static period to a dynamic one, or after
     * initialization completes.
     */
    val accumulatedAvgY
        get() = internalDetector.accumulatedAvgY

    /**
     * Gets average y-coordinate of accelerometer measurements accumulated during last static
     * period.
     * This value is updated when switching from a static period to a dynamic one, or after
     * initialization completes.
     */
    val accumulatedAvgYAsAcceleration: Acceleration
        get() = internalDetector.accumulatedAvgYAsMeasurement

    /**
     * Gets average y-coordinate of accelerometer measurements accumulated during last static
     * period.
     * This value is updated when switching from a static period to a dynamic one, or after
     * initialization completes.
     *
     * @param result instance where result will be stored.
     */
    fun getAccumulatedAvgYAsAcceleration(result: Acceleration) {
        internalDetector.getAccumulatedAvgYAsMeasurement(result)
    }

    /**
     * Gets average z-coordinate of accelerometer measurements accumulated during last static
     * period expressed in meters per squared second (m/s^2).
     * This value is updated when switching from a static period to a dynamic one, or after
     * initialization completes.
     */
    val accumulatedAvgZ
        get() = internalDetector.accumulatedAvgZ

    /**
     * Gets average z-coordinate of accelerometer measurements accumulated during last static
     * period.
     * This value is updated when switching from a static period to a dynamic one, or after
     * initialization completes.
     */
    val accumulatedAvgZAsAcceleration: Acceleration
        get() = internalDetector.accumulatedAvgZAsMeasurement

    /**
     * Gets average z-coordinate of accelerometer measurements accumulated during last static
     * period.
     * This value is updated when switching from a static period to a dynamic one, or after
     * initialization completes.
     *
     * @param result instance where result will be stored.
     */
    fun getAccumulatedAvgZAsAcceleration(result: Acceleration) {
        internalDetector.getAccumulatedAvgZAsMeasurement(result)
    }

    /**
     * Gets average accelerometer measurements triad accumulated during last static period.
     * This value is updated when switching from a static period to a dynamic one or after
     * initialization completes.
     */
    val accumulatedAvgTriad: AccelerationTriad
        get() = internalDetector.accumulatedAvgTriad

    /**
     * Gets average accelerometer measurements triad accumulated during last static period.
     * This value is updated when switching from a static period to a dynamic one or after
     * initialization completes.
     *
     * @param result instance where result will be stored.
     */
    fun getAccumulatedAvgTriad(result: AccelerationTriad) {
        internalDetector.getAccumulatedAvgTriad(result)
    }

    /**
     * Gets standard deviation of x-coordinate of accelerometer measurements accumulated during last
     * static period expressed in meters per squared second (m/s^2).
     * This value is updated when switching from a static period to a dynamic one or after
     * completing initialization.
     */
    val accumulatedStdX
        get() = internalDetector.accumulatedStdX

    /**
     * Gets standard deviation of x-coordinate of accelerometer measurements accumulated during last
     * static period.
     * This value is updated when switching from a static period to a dynamic one or after
     * completing initialization.
     */
    val accumulatedStdXAsAcceleration: Acceleration
        get() = internalDetector.accumulatedStdXAsMeasurement

    /**
     * Gets standard deviation of x-coordinate of accelerometer measurements accumulated during last
     * static period.
     * This value is updated when switching from a static period to a dynamic one or after
     * completing initialization.
     *
     * @param result instance where result will be stored.
     */
    fun getAccumulatedStdXAsAcceleration(result: Acceleration) {
        internalDetector.getAccumulatedStdXAsMeasurement(result)
    }

    /**
     * Gets standard deviation of y-coordinate of accelerometer measurements accumulated during last
     * static period expressed in meters per squared second (m/s^2).
     * This value is updated when switching from a static period to a dynamic one or after
     * completing initialization.
     */
    val accumulatedStdY
        get() = internalDetector.accumulatedStdY

    /**
     * Gets standard deviation of y-coordinate of accelerometer measurements accumulated during last
     * static period.
     * This value is updated when switching from a static period to a dynamic one or after
     * completing initialization.
     */
    val accumulatedStdYAsAcceleration: Acceleration
        get() = internalDetector.accumulatedStdYAsMeasurement

    /**
     * Gets standard deviation of y-coordinate of accelerometer measurements accumulated during last
     * static period.
     * This value is updated when switching from a static period to a dynamic one or after
     * completing initialization.
     *
     * @param result instance where result will be stored.
     */
    fun getAccumulatedStdYAsAcceleration(result: Acceleration) {
        internalDetector.getAccumulatedStdYAsMeasurement(result)
    }

    /**
     * Gets standard deviation of z-coordinate of accelerometer measurements accumulated during last
     * static period expressed in meters per squared second (m/s^2).
     * This value is updated when switching from a static period to a dynamic one or after
     * completing initialization.
     */
    val accumulatedStdZ
        get() = internalDetector.accumulatedStdZ

    /**
     * Gets standard deviation of z-coordinate of accelerometer measurements accumulated during last
     * static period.
     * This value is updated when switching from a static period to a dynamic one or after
     * completing initialization.
     */
    val accumulatedStdZAsAcceleration: Acceleration
        get() = internalDetector.accumulatedStdZAsMeasurement

    /**
     * Gets standard deviation of z-coordinate of accelerometer measurements accumulated during last
     * static period.
     * This value is updated when switching from a static period to a dynamic one or after
     * completing initialization.
     *
     * @param result instance where result will be stored.
     */
    fun getAccumulatedStdZAsAcceleration(result: Acceleration) {
        internalDetector.getAccumulatedStdZAsMeasurement(result)
    }

    /**
     * Gets standard deviation of accelerometer measurements accumulated during last static period.
     * This value is updated when switching from a static period to a dynamic one or after
     * completing initialization.
     */
    val accumulatedStdTriad: AccelerationTriad
        get() = internalDetector.accumulatedStdTriad

    /**
     * Gets standard deviation of accelerometer measurements accumulated during last static period.
     * This value is updated when switching from a static period to a dynamic one or after
     * completing initialization.
     *
     * @param result instance where result will be stored.
     */
    fun getAccumulatedStdTriad(result: AccelerationTriad) {
        internalDetector.getAccumulatedStdTriad(result)
    }

    /**
     * Gets windowed average x-coordinate of accelerometer measurements for each processed triad
     * expressed in meters per squared second (m/s^2).
     * This value is updated for each processed sample containing an average value for the sample
     * within the window.
     */
    val instantaneousAvgX
        get() = internalDetector.instantaneousAvgX

    /**
     * Gets windowed average x-coordinate of accelerometer measurements for each processed triad.
     * This value is updated for each processed sample containing an average value for the sample
     * within the window.
     */
    val instantaneousAvgXAsAcceleration: Acceleration
        get() = internalDetector.instantaneousAvgXAsMeasurement

    /**
     * Gets windowed average x-coordinate of accelerometer measurements for each processed triad.
     * This value is updated for each processed sample containing an average value for the sample
     * within the window.
     *
     * @param result instance where result will be stored.
     */
    fun getInstantaneousAvgXAsAcceleration(result: Acceleration) {
        internalDetector.getInstantaneousAvgXAsMeasurement(result)
    }

    /**
     * Gets windowed average y-coordinate of accelerometer measurements for each processed triad
     * expressed in meters per squared second (m/s^2).
     * This value is updated for each processed sample containing an average value for the sample
     * within the window.
     */
    val instantaneousAvgY
        get() = internalDetector.instantaneousAvgY

    /**
     * Gets windowed average y-coordinate of accelerometer measurements for each processed triad.
     * This value is updated for each processed sample containing an average value for the sample
     * within the window.
     */
    val instantaneousAvgYAsAcceleration: Acceleration
        get() = internalDetector.instantaneousAvgYAsMeasurement

    /**
     * Gets windowed average y-coordinate of accelerometer measurements for each processed triad.
     * This value is updated for each processed sample containing an average value for the sample
     * within the window.
     *
     * @param result instance where result will be stored.
     */
    fun getInstantaneousAvgYAsAcceleration(result: Acceleration) {
        internalDetector.getInstantaneousAvgYAsMeasurement(result)
    }

    /**
     * Gets windowed average z-coordinate of accelerometer measurements for each processed triad
     * expressed in meters per squared second (m/s^2).
     * This value is updated for each processed sample containing an average value for the sample
     * within the window.
     */
    val instantaneousAvgZ
        get() = internalDetector.instantaneousAvgZ

    /**
     * Gets windowed average z-coordinate of accelerometer measurements for each processed triad.
     * This value is updated for each processed sample containing an average value for the sample
     * within the window.
     */
    val instantaneousAvgZAsAcceleration: Acceleration
        get() = internalDetector.instantaneousAvgZAsMeasurement

    /**
     * Gets windowed average z-coordinate of accelerometer measurements for each processed triad.
     * This value is updated for each processed sample containing an average value for the sample
     * within the window.
     *
     * @param result instance where result will be stored.
     */
    fun getInstantaneousAvgZAsAcceleration(result: Acceleration) {
        internalDetector.getInstantaneousAvgZAsMeasurement(result)
    }

    /**
     * Gets windowed average of accelerometer measurements for each processed triad.
     * This value is updated for each processed sample containing an average value for the samples
     * within the window.
     */
    val instantaneousAvgTriad: AccelerationTriad
        get() = internalDetector.instantaneousAvgTriad

    /**
     * Gets windowed average of accelerometer measurements for each processed triad.
     * This value is updated for each processed sample containing an average value for the samples
     * within the window.
     *
     * @param result instance where result will be stored.
     */
    fun getInstantaneousAvgTriad(result: AccelerationTriad) {
        internalDetector.getInstantaneousAvgTriad(result)
    }

    /**
     * Gets windowed standard deviation of x-coordinate of accelerometer measurements for each
     * processed triad expressed in meters per squared second (m/s^2).
     * This value is updated for each processed sample containing measured standard deviation for
     * the samples within the window.
     */
    val instantaneousStdX
        get() = internalDetector.instantaneousStdX

    /**
     * Gets windowed standard deviation of x-coordinate of accelerometer measurements for each
     * processed triad.
     * This value is updated for each processed sample containing measured standard deviation for
     * the samples within the window.
     */
    val instantaneousStdXAsAcceleration: Acceleration
        get() = internalDetector.instantaneousStdXAsMeasurement

    /**
     * Gets windowed standard deviation of x-coordinate of accelerometer measurements for each
     * processed triad.
     * This value is updated for each processed sample containing measured standard deviation for
     * the samples within the window.
     *
     * @param result instance where result will be stored.
     */
    fun getInstantaneousStdXAsAcceleration(result: Acceleration) {
        internalDetector.getInstantaneousStdXAsMeasurement(result)
    }

    /**
     * Gets windowed standard deviation of y-coordinate of accelerometer measurements for each
     * processed triad expressed in meters per squared second (m/s^2).
     * This value is updated for each processed sample containing measured standard deviation for
     * the samples within the window.
     */
    val instantaneousStdY
        get() = internalDetector.instantaneousStdY

    /**
     * Gets windowed standard deviation of y-coordinate of accelerometer measurements for each
     * processed triad.
     * This value is updated for each processed sample containing measured standard deviation for
     * the samples within the window.
     */
    val instantaneousStdYAsAcceleration: Acceleration
        get() = internalDetector.instantaneousStdYAsMeasurement

    /**
     * Gets windowed standard deviation of y-coordinate of accelerometer measurements for each
     * processed triad.
     * This value is updated for each processed sample containing measured standard deviation for
     * the samples within the window.
     *
     * @param result instance where result will be stored.
     */
    fun getInstantaneousStdYAsAcceleration(result: Acceleration) {
        internalDetector.getInstantaneousStdYAsMeasurement(result)
    }

    /**
     * Gets windowed standard deviation of z-coordinate of accelerometer measurements for each
     * processed triad expressed in meters per squared second (m/s^2).
     * This value is updated for each processed sample containing measured standard deviation for
     * the samples within the window.
     */
    val instantaneousStdZ
        get() = internalDetector.instantaneousStdZ

    /**
     * Gets windowed standard deviation of z-coordinate of accelerometer measurements for each
     * processed triad.
     * This value is updated for each processed sample containing measured standard deviation for
     * the samples within the window.
     */
    val instantaneousStdZAsAcceleration: Acceleration
        get() = internalDetector.instantaneousStdZAsMeasurement

    /**
     * Gets windowed standard deviation of z-coordinate of accelerometer measurements for each
     * processed triad.
     * This value is updated for each processed sample containing measured standard deviation for
     * the samples within the window.
     *
     * @param result instance where result will be stored.
     */
    fun getInstantaneousStdZAsAcceleration(result: Acceleration) {
        internalDetector.getInstantaneousStdZAsMeasurement(result)
    }

    /**
     * Gets windowed standard deviation of accelerometer measurements for each processed triad.
     * This value is updated for each processed sample containing measured standard deviation for
     * the samples within the window.
     */
    val instantaneousStdTriad: AccelerationTriad
        get() = internalDetector.instantaneousStdTriad

    /**
     * Gets windowed standard deviation of measurements for each processed triad.
     * This value is updated for each processed sample containing measured standard deviation for
     * the samples within the window.
     */
    fun getInstantaneousStdTriad(result: AccelerationTriad) {
        internalDetector.getInstantaneousStdTriad(result)
    }

    /**
     * Gets average time interval between accelerometer samples expressed in seconds (s).
     * This is only available once detector completes initialization.
     */
    val averageTimeInterval
        get() = if (initialized) {
            timeIntervalEstimator.averageTimeInterval
        } else {
            null
        }

    /**
     * Gets average time interval between accelerometer samples.
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
     * seconds (s).
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
     * Initially the interval detector will be idle.
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
     * @throws IllegalStateException if detector is already running.
     */
    @Throws(IllegalStateException::class)
    fun start() {
        check(!running)

        reset()

        running = true
        collector.start()
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
     * Maps error reason to an [ErrorReason]
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
    interface OnInitializationStartedListener {
        /**
         * Called when initial static period starts so that base noise level starts being estimated.
         *
         * @param detector detector that raised the event.
         */
        fun onInitializationStarted(detector: IntervalDetector)
    }

    /**
     * Interface to notify when detector completes its initialization.
     */
    interface OnInitializationCompletedListener {
        /**
         * Called when initial static period successfully completes and base noise level is
         * estimated so that static and dynamic periods can be detected.
         *
         * @param detector detector that raised the event.
         * @param baseNoiseLevel base measurement noise level expressed in meters per squared
         * second (m/s^2).
         */
        fun onInitializationCompleted(
            detector: IntervalDetector,
            baseNoiseLevel: Double
        )
    }

    /**
     * Interface to notify when an error occurs.
     */
    interface OnErrorListener {
        /**
         * Called when an error is detected, either at initialization because excessive movement
         * forces are detected, or because sensor becomes unreliable.
         * When an error occurs, detector is stopped and needs to be restarted to be used again.
         *
         * @param detector detector that raised the event.
         * @param reason reason why error was detected.
         */
        fun onError(
            detector: IntervalDetector,
            reason: ErrorReason
        )
    }

    /**
     * Interface to notify when a new static interval is detected.
     * Instantaneous average measurements and standard deviation within the averaging window are
     * provided at the time the new static interval is detected (and consequently detector ends
     * a previous dynamic interval or its initialization stage).
     */
    interface OnStaticIntervalDetectedListener {
        /**
         * Called when a static interval has been detected after initialization.
         *
         * @param detector detector that raised the event.
         * @param instantaneousAvgX instantaneous average x-coordinate of measurements within the
         * window expressed in meters per squared second (m/s^2).
         * @param instantaneousAvgY instantaneous average y-coordinate of measurements within the
         * window expressed in meters per squared second (m/s^2).
         * @param instantaneousAvgZ instantaneous average z-coordinate of measurements within the
         * window expressed in meters per squared second (m/s^2).
         * @param instantaneousStdX instantaneous standard deviation of x-coordinate measurements
         * within the window expressed in meters per squared second (m/s^2).
         * @param instantaneousStdY instantaneous standard deviation of y-coordinate measurements
         * within the window expressed in meters per squared second (m/s^2).
         * @param instantaneousStdZ instantaneous standard deviation of z-coordinate measurements
         * within the window expressed in meters per squared second (m/s^2).
         */
        fun onStaticIntervalDetected(
            detector: IntervalDetector,
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
    interface OnDynamicIntervalDetectedListener {
        /**
         * Called when a dynamic interval has been detected after initialization.
         *
         * @param detector detector that raised the event.
         * @param instantaneousAvgX instantaneous average x-coordinate of measurements within the
         * window expressed in meters per squared second (m/s^2).
         * @param instantaneousAvgY instantaneous average y-coordinate of measurements within the
         * window expressed in meters per squared second (m/s^2).
         * @param instantaneousAvgZ instantaneous average z-coordinate of measurements within the
         * window expressed in meters per squared second (m/s^2).
         * @param instantaneousStdX instantaneous x-coordinate of standard deviation for
         * measurements within the window expressed in meters per squared second (m/s^2).
         * @param instantaneousStdY instantaneous y-coordinate of standard deviation for
         * measurements within the window expressed in meters per squared second (m/s^2).
         * @param instantaneousStdZ instantaneous z-coordinate of standard deviation for
         * measurements within the window expressed in meters per squared second (m/s^2).
         * @param accumulatedAvgX accumulated average x-coordinate of measurements during last
         * static period expressed in meters per squared second (m/s^2).
         * @param accumulatedAvgY accumulated average y-coordinate of measurements during last
         * static period expressed in meters per squared second (m/s^2).
         * @param accumulatedAvgZ accumulated average z-coordinate of measurements during last
         * static period expressed in meters per squared second (m/s^2).
         * @param accumulatedStdX standard deviation of x-coordinate of accumulated measurements
         * during last static period expressed in meters per squared second (m/s^2).
         * @param accumulatedStdY standard deviation of y-coordinate of accumulated measurements
         * during last static period expressed in meters per squared second (m/s^2).
         * @param accumulatedStdZ standard deviation of z-coordinate of accumulated measurements
         * during last static period expressed in meters per squared second (m/s^2).
         */
        fun onDynamicIntervalDetected(
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
        )
    }

    /**
     * Interface to notify when detector is reset (occurs when starting after stopping the
     * detector).
     */
    interface OnResetListener {
        /**
         * Called when detector is reset.
         *
         * @param detector detector that raised the event.
         */
        fun onReset(detector: IntervalDetector)
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
         * A static interval has been detected, where accelerometer is considered to be subject to
         * no substantial movement forces.
         */
        STATIC_INTERVAL,

        /**
         * A dynamic interval has been detected, where accelerometer is considered to be subject to
         * substantial movement forces.
         */
        DYNAMIC_INTERVAL,

        /**
         * Detector has failed. This happens if accelerometer is subject to sudden movement forces
         * while detector is initializing during the initial static period, if there is too much
         * overall motion during initialization, or if accelerometer sensor becomes unreliable.
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