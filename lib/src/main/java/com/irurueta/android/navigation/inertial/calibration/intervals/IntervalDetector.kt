package com.irurueta.android.navigation.inertial.calibration.intervals

import android.content.Context
import com.irurueta.android.navigation.inertial.collectors.SensorAccuracy
import com.irurueta.android.navigation.inertial.collectors.SensorCollector
import com.irurueta.android.navigation.inertial.collectors.SensorDelay
import com.irurueta.navigation.inertial.calibration.TimeIntervalEstimator
import com.irurueta.navigation.inertial.calibration.Triad
import com.irurueta.navigation.inertial.calibration.intervals.TriadStaticIntervalDetector
import com.irurueta.navigation.inertial.calibration.intervals.TriadStaticIntervalDetectorListener
import com.irurueta.units.Measurement
import com.irurueta.units.Time

/**
 * Base class for interval detectors.
 * Interval detectors detect static or motion intervals of measurements captured by different
 * sensors (accelerometer, gyroscope or magnetometer), which typically indicate whether the device
 * is moving or not.
 * When detector is tarted, initialization occurs to determine the noise level of the sensor while
 * keeping device static. Once the detector is initialized, then static or dynamic intervals can be
 * detected.
 * This detector uses accumulated average values during static intervals, and windowed
 * averages as "instantaneous" values during dynamic intervals.
 * Length of windows, as well as thresholds to determine when changes between static and dynamic
 * intervals occur can be easily configured.
 *
 * @param I an implementation of [IntervalDetector]
 * @param S an implementation of [SensorCollector]
 * @param U type of unit.
 * @param M a type of measurement.
 * @param T a triad type.
 * @param D a detector type.
 * @param L a listener type.
 * @property context Android context.
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
 * @property accuracyChangedListener listener to notify when sensor accuracy changes.
 */
abstract class IntervalDetector<I : IntervalDetector<I, S, U, M, T, D, L>, S : SensorCollector,
        U : Enum<*>, M : Measurement<U>, T : Triad<U, M>,
        D : TriadStaticIntervalDetector<U, M, T, D, L>,
        L : TriadStaticIntervalDetectorListener<U, M, T, D>>(
    val context: Context,
    val sensorDelay: SensorDelay,
    var initializationStartedListener: OnInitializationStartedListener<I>? = null,
    var initializationCompletedListener: OnInitializationCompletedListener<I>? = null,
    var errorListener: OnErrorListener<I>? = null,
    var staticIntervalDetectedListener: OnStaticIntervalDetectedListener<I>? = null,
    var dynamicIntervalDetectedListener: OnDynamicIntervalDetectedListener<I>? = null,
    var resetListener: OnResetListener<I>? = null,
    var accuracyChangedListener: SensorCollector.OnAccuracyChangedListener? = null
) {
    /**
     * Listener for internal interval detector.
     */
    protected abstract val internalDetectorListener : L

    /**
     * Internal interval detector.
     * Processes sensor measurements and detects static and dynamic intervals.
     */
    protected abstract val internalDetector: D

    /**
     * Sensor collector.
     * Collects sensor measurements.
     */
    protected abstract val collector: S

    /**
     * Internal time estimator.
     * This can be used to estimate statistics about time intervals of measurements.
     */
    protected val timeIntervalEstimator = TimeIntervalEstimator()

    /**
     * Timestamp when detector started.
     */
    protected var initialTimestamp: Long = 0L

    /**
     * Listener to detect when accuracy of sensor changes.
     * When sensor becomes unreliable, an error is notified.
     */
    @Suppress("UNCHECKED_CAST")
    protected val internalAccuracyChangedListener =
        SensorCollector.OnAccuracyChangedListener { accuracy ->
            if (accuracy == SensorAccuracy.UNRELIABLE) {
                stop()
                unreliable = true
                errorListener?.onError(
                    this@IntervalDetector as I,
                    ErrorReason.UNRELIABLE_SENSOR
                )
            }

            accuracyChangedListener?.onAccuracyChanged(accuracy)
        }

    /**
     * Indicates whether detector successfully completed initialization.
     */
    protected var initialized: Boolean = false

    /**
     * Indicates whether sensor has become unreliable, and thus
     * the interval detector is considered to be in [Status.FAILED].
     */
    private var unreliable = false

    /**
     * Gets sensor being used to obtain measurements, or null if not available.
     * This can be used to obtain additional information about the sensor.
     */
    val sensor
        get() = collector.sensor

    /**
     * Gets or sets length of number of samples to keep within the window being processed to
     * determine instantaneous sensor noise level. Window size must always be larger than
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
     * Gets or sets overall absolute threshold to determine whether there has been excessive motion
     * during the whole initialization phase. Failure will be detected if estimated base noise level
     * exceeds this threshold when initialization completes. This threshold is expressed in meters
     * per squared second (m/s^2) for accelerometer, radians per second (rad/s) for gyroscope and
     * Teslas (T) for magnetometer.
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
    var baseNoiseLevelAbsoluteThresholdAsMeasurement: M
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
    fun getBaseNoiseLevelAbsoluteThresholdAsMeasurement(result: M) {
        internalDetector.getBaseNoiseLevelAbsoluteThresholdAsMeasurement(result)
    }

    /**
     * Gets accelerometer measurement base noise level that has been detected during initialization
     * expressed in meters per squared second (m/s^2) for accelerometer, radians per second (rad/s)
     * for gyroscope and Teslas (T) for magnetometer.
     * This is only available once detector completes initialization.
     */
    val baseNoiseLevel
        get() = if (initialized) {
            internalDetector.baseNoiseLevel
        } else {
            null
        }

    /**
     * Gets sensor measurement base noise level that has been detected during initialization.
     * This is only available once detector completes initialization.
     */
    val baseNoiseLevelAsMeasurement
        get() = if (initialized) {
            internalDetector.baseNoiseLevelAsMeasurement
        } else {
            null
        }

    /**
     * Gets sensor measurement base noise level that has been detected during initialization.
     * This is only available once detector completes initialization.
     *
     * @param result instance where result will be stored.
     * @return true if result is available, false otherwise.
     */
    fun getBaseNoiseLevelAsMeasurement(result: M): Boolean {
        return if (initialized) {
            internalDetector.getBaseNoiseLevelAsMeasurement(result)
            true
        } else {
            false
        }
    }

    /**
     * Gets measurement base noise level PSD (Power Spectral Density) expressed in (m^2 * s^-3) for
     * accelerometer, (rad^2/s) for gyroscope or (T^2 * s) for magnetometer..
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
     * in (m * s^-1.5) for accelerometer, (rad * s^-0.5) for gyroscope or (T * s^0.5) for
     * magnetometer.
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
     * squared second (m/s^2) for accelerometer, radians per second (rad/s) for gyroscope or
     * Teslas (T) for magnetometer.
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
    val thresholdAsMeasurement
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
    fun getThresholdAsMeasurement(result: M): Boolean {
        return if (initialized) {
            internalDetector.getThresholdAsMeasurement(result)
            true
        } else {
            false
        }
    }

    /**
     * Gets average x-coordinate of sensor measurements accumulated during last static
     * period expressed in meters per squared second (m/s^2) for accelerometer, radians per second
     * (rad/s) for gyroscope or Teslas (T) for magnetometer.
     * This value is updated when switching from a static period to a dynamic one, or after
     * initialization completes.
     */
    val accumulatedAvgX
        get() = internalDetector.accumulatedAvgX

    /**
     * Gets average x-coordinate of sensor measurements accumulated during last static
     * period.
     * This value is updated when switching from a static period to a dynamic one, or after
     * initialization completes.
     */
    val accumulatedAvgXAsMeasurement: M
        get() = internalDetector.accumulatedAvgXAsMeasurement

    /**
     * Gets average x-coordinate of sensor measurements accumulated during last static
     * period.
     * This value is updated when switching from a static period to a dynamic one, or after
     * initialization completes.
     *
     * @param result instance where result will be stored.
     */
    fun getAccumulatedAvgXAsMeasurement(result: M) {
        internalDetector.getAccumulatedAvgXAsMeasurement(result)
    }

    /**
     * Gets average y-coordinate of sensor measurements accumulated during last static
     * period expressed in meters per squared second (m/s^2) for accelerometer, radians per second
     * (rad/s) for gyroscope or Teslas (T) for magnetometer.
     * This value is updated when switching from a static period to a dynamic one, or after
     * initialization completes.
     */
    val accumulatedAvgY
        get() = internalDetector.accumulatedAvgY

    /**
     * Gets average y-coordinate of sensor measurements accumulated during last static
     * period.
     * This value is updated when switching from a static period to a dynamic one, or after
     * initialization completes.
     */
    val accumulatedAvgYAsMeasurement: M
        get() = internalDetector.accumulatedAvgYAsMeasurement

    /**
     * Gets average y-coordinate of sensor measurements accumulated during last static
     * period.
     * This value is updated when switching from a static period to a dynamic one, or after
     * initialization completes.
     *
     * @param result instance where result will be stored.
     */
    fun getAccumulatedAvgYAsMeasurement(result: M) {
        internalDetector.getAccumulatedAvgYAsMeasurement(result)
    }

    /**
     * Gets average z-coordinate of sensor measurements accumulated during last static
     * period expressed in meters per squared second (m/s^2) for accelerometer, radians per second
     * (rad/s) for gyroscope or Teslas (T) for magnetometer.
     * This value is updated when switching from a static period to a dynamic one, or after
     * initialization completes.
     */
    val accumulatedAvgZ
        get() = internalDetector.accumulatedAvgZ

    /**
     * Gets average z-coordinate of sensor measurements accumulated during last static
     * period.
     * This value is updated when switching from a static period to a dynamic one, or after
     * initialization completes.
     */
    val accumulatedAvgZAsMeasurement: M
        get() = internalDetector.accumulatedAvgZAsMeasurement

    /**
     * Gets average z-coordinate of sensor measurements accumulated during last static
     * period.
     * This value is updated when switching from a static period to a dynamic one, or after
     * initialization completes.
     *
     * @param result instance where result will be stored.
     */
    fun getAccumulatedAvgZAsMeasurement(result: M) {
        internalDetector.getAccumulatedAvgZAsMeasurement(result)
    }

    /**
     * Gets average sensor measurements triad accumulated during last static period.
     * This value is updated when switching from a static period to a dynamic one or after
     * initialization completes.
     */
    val accumulatedAvgTriad: T
        get() = internalDetector.accumulatedAvgTriad

    /**
     * Gets average sensor measurements triad accumulated during last static period.
     * This value is updated when switching from a static period to a dynamic one or after
     * initialization completes.
     *
     * @param result instance where result will be stored.
     */
    fun getAccumulatedAvgTriad(result: T) {
        internalDetector.getAccumulatedAvgTriad(result)
    }

    /**
     * Gets standard deviation of x-coordinate of sensor measurements accumulated during last
     * static period expressed in meters per squared second (m/s^2) for accelerometer, radians per
     * second (rad/s) for gyroscope or Teslas (T) for magnetometer.
     * This value is updated when switching from a static period to a dynamic one or after
     * completing initialization.
     */
    val accumulatedStdX
        get() = internalDetector.accumulatedStdX

    /**
     * Gets standard deviation of x-coordinate of sensor measurements accumulated during last
     * static period.
     * This value is updated when switching from a static period to a dynamic one or after
     * completing initialization.
     */
    val accumulatedStdXAsMeasurement: M
        get() = internalDetector.accumulatedStdXAsMeasurement

    /**
     * Gets standard deviation of x-coordinate of sensor measurements accumulated during last
     * static period.
     * This value is updated when switching from a static period to a dynamic one or after
     * completing initialization.
     *
     * @param result instance where result will be stored.
     */
    fun getAccumulatedStdXAsMeasurement(result: M) {
        internalDetector.getAccumulatedStdXAsMeasurement(result)
    }

    /**
     * Gets standard deviation of y-coordinate of sensor measurements accumulated during last
     * static period expressed in meters per squared second (m/s^2) for accelerometer, radians per
     * second (rad/s) for gyroscope or Teslas (T) for magnetometer.
     * This value is updated when switching from a static period to a dynamic one or after
     * completing initialization.
     */
    val accumulatedStdY
        get() = internalDetector.accumulatedStdY

    /**
     * Gets standard deviation of y-coordinate of sensor measurements accumulated during last
     * static period.
     * This value is updated when switching from a static period to a dynamic one or after
     * completing initialization.
     */
    val accumulatedStdYAsMeasurement: M
        get() = internalDetector.accumulatedStdYAsMeasurement

    /**
     * Gets standard deviation of y-coordinate of sensor measurements accumulated during last
     * static period.
     * This value is updated when switching from a static period to a dynamic one or after
     * completing initialization.
     *
     * @param result instance where result will be stored.
     */
    fun getAccumulatedStdYAsMeasurement(result: M) {
        internalDetector.getAccumulatedStdYAsMeasurement(result)
    }

    /**
     * Gets standard deviation of z-coordinate of sensor measurements accumulated during last
     * static period expressed in meters per squared second (m/s^2) for accelerometer, radians per
     * second (rad/s) for gyroscope or Teslas (T) for magnetometer.
     * This value is updated when switching from a static period to a dynamic one or after
     * completing initialization.
     */
    val accumulatedStdZ
        get() = internalDetector.accumulatedStdZ

    /**
     * Gets standard deviation of z-coordinate of sensor measurements accumulated during last
     * static period.
     * This value is updated when switching from a static period to a dynamic one or after
     * completing initialization.
     */
    val accumulatedStdZAsMeasurement: M
        get() = internalDetector.accumulatedStdZAsMeasurement

    /**
     * Gets standard deviation of z-coordinate of sensor measurements accumulated during last
     * static period.
     * This value is updated when switching from a static period to a dynamic one or after
     * completing initialization.
     *
     * @param result instance where result will be stored.
     */
    fun getAccumulatedStdZAsMeasurement(result: M) {
        internalDetector.getAccumulatedStdZAsMeasurement(result)
    }

    /**
     * Gets standard deviation of sensor measurements accumulated during last static period.
     * This value is updated when switching from a static period to a dynamic one or after
     * completing initialization.
     */
    val accumulatedStdTriad: T
        get() = internalDetector.accumulatedStdTriad

    /**
     * Gets standard deviation of sensor measurements accumulated during last static period.
     * This value is updated when switching from a static period to a dynamic one or after
     * completing initialization.
     *
     * @param result instance where result will be stored.
     */
    fun getAccumulatedStdTriad(result: T) {
        internalDetector.getAccumulatedStdTriad(result)
    }

    /**
     * Gets windowed average x-coordinate of sensor measurements for each processed triad
     * expressed in meters per squared second (m/s^2) for accelerometer, radians per second (rad/s)
     * for gyroscope or Teslas (T) for magnetometer.
     * This value is updated for each processed sample containing an average value for the sample
     * within the window.
     */
    val instantaneousAvgX
        get() = internalDetector.instantaneousAvgX

    /**
     * Gets windowed average x-coordinate of sensor measurements for each processed triad.
     * This value is updated for each processed sample containing an average value for the sample
     * within the window.
     */
    val instantaneousAvgXAsMeasurement: M
        get() = internalDetector.instantaneousAvgXAsMeasurement

    /**
     * Gets windowed average x-coordinate of sensor measurements for each processed triad.
     * This value is updated for each processed sample containing an average value for the sample
     * within the window.
     *
     * @param result instance where result will be stored.
     */
    fun getInstantaneousAvgXAsMeasurement(result: M) {
        internalDetector.getInstantaneousAvgXAsMeasurement(result)
    }

    /**
     * Gets windowed average y-coordinate of sensor measurements for each processed triad
     * expressed in meters per squared second (m/s^2) for accelerometer, radians per second (rad/s)
     * for gyroscope or Teslas (T) for magnetometer.
     * This value is updated for each processed sample containing an average value for the sample
     * within the window.
     */
    val instantaneousAvgY
        get() = internalDetector.instantaneousAvgY

    /**
     * Gets windowed average y-coordinate of sensor measurements for each processed triad.
     * This value is updated for each processed sample containing an average value for the sample
     * within the window.
     */
    val instantaneousAvgYAsMeasurement: M
        get() = internalDetector.instantaneousAvgYAsMeasurement

    /**
     * Gets windowed average y-coordinate of sensor measurements for each processed triad.
     * This value is updated for each processed sample containing an average value for the sample
     * within the window.
     *
     * @param result instance where result will be stored.
     */
    fun getInstantaneousAvgYAsMeasurement(result: M) {
        internalDetector.getInstantaneousAvgYAsMeasurement(result)
    }

    /**
     * Gets windowed average z-coordinate of sensor measurements for each processed triad
     * expressed in meters per squared second (m/s^2) for accelerometer, radians per second (rad/s)
     * for gyroscope or Teslas (T) for magnetometer.
     * This value is updated for each processed sample containing an average value for the sample
     * within the window.
     */
    val instantaneousAvgZ
        get() = internalDetector.instantaneousAvgZ

    /**
     * Gets windowed average z-coordinate of sensor measurements for each processed triad.
     * This value is updated for each processed sample containing an average value for the sample
     * within the window.
     */
    val instantaneousAvgZAsMeasurement: M
        get() = internalDetector.instantaneousAvgZAsMeasurement

    /**
     * Gets windowed average z-coordinate of sensor measurements for each processed triad.
     * This value is updated for each processed sample containing an average value for the sample
     * within the window.
     *
     * @param result instance where result will be stored.
     */
    fun getInstantaneousAvgZAsMeasurement(result: M) {
        internalDetector.getInstantaneousAvgZAsMeasurement(result)
    }

    /**
     * Gets windowed average of sensor measurements for each processed triad.
     * This value is updated for each processed sample containing an average value for the samples
     * within the window.
     */
    val instantaneousAvgTriad: T
        get() = internalDetector.instantaneousAvgTriad

    /**
     * Gets windowed average of sensor measurements for each processed triad.
     * This value is updated for each processed sample containing an average value for the samples
     * within the window.
     *
     * @param result instance where result will be stored.
     */
    fun getInstantaneousAvgTriad(result: T) {
        internalDetector.getInstantaneousAvgTriad(result)
    }

    /**
     * Gets windowed standard deviation of x-coordinate of sensor measurements for each
     * processed triad expressed in meters per squared second (m/s^2) for accelerometer, radians per
     * second (rad/s) for gyroscope or Teslas (T) for magnetometer.
     * This value is updated for each processed sample containing measured standard deviation for
     * the samples within the window.
     */
    val instantaneousStdX
        get() = internalDetector.instantaneousStdX

    /**
     * Gets windowed standard deviation of x-coordinate of sensor measurements for each
     * processed triad.
     * This value is updated for each processed sample containing measured standard deviation for
     * the samples within the window.
     */
    val instantaneousStdXAsMeasurement: M
        get() = internalDetector.instantaneousStdXAsMeasurement

    /**
     * Gets windowed standard deviation of x-coordinate of sensor measurements for each
     * processed triad.
     * This value is updated for each processed sample containing measured standard deviation for
     * the samples within the window.
     *
     * @param result instance where result will be stored.
     */
    fun getInstantaneousStdXAsMeasurement(result: M) {
        internalDetector.getInstantaneousStdXAsMeasurement(result)
    }

    /**
     * Gets windowed standard deviation of y-coordinate of sensor measurements for each
     * processed triad expressed in meters per squared second (m/s^2) for accelerometer, radians
     * per second (rad/s) for gyroscope or Teslas (T) for magnetometer.
     * This value is updated for each processed sample containing measured standard deviation for
     * the samples within the window.
     */
    val instantaneousStdY
        get() = internalDetector.instantaneousStdY

    /**
     * Gets windowed standard deviation of y-coordinate of sensor measurements for each
     * processed triad.
     * This value is updated for each processed sample containing measured standard deviation for
     * the samples within the window.
     */
    val instantaneousStdYAsMeasurement: M
        get() = internalDetector.instantaneousStdYAsMeasurement

    /**
     * Gets windowed standard deviation of y-coordinate of sensor measurements for each
     * processed triad.
     * This value is updated for each processed sample containing measured standard deviation for
     * the samples within the window.
     *
     * @param result instance where result will be stored.
     */
    fun getInstantaneousStdYAsMeasurement(result: M) {
        internalDetector.getInstantaneousStdYAsMeasurement(result)
    }

    /**
     * Gets windowed standard deviation of z-coordinate of sensor measurements for each
     * processed triad expressed in meters per squared second (m/s^2) for accelerometer, radians
     * per second (rad/s) for gyroscope or Teslas (T) for magnetometer.
     * This value is updated for each processed sample containing measured standard deviation for
     * the samples within the window.
     */
    val instantaneousStdZ
        get() = internalDetector.instantaneousStdZ

    /**
     * Gets windowed standard deviation of z-coordinate of sensor measurements for each
     * processed triad.
     * This value is updated for each processed sample containing measured standard deviation for
     * the samples within the window.
     */
    val instantaneousStdZAsMeasurement: M
        get() = internalDetector.instantaneousStdZAsMeasurement

    /**
     * Gets windowed standard deviation of z-coordinate of sensor measurements for each
     * processed triad.
     * This value is updated for each processed sample containing measured standard deviation for
     * the samples within the window.
     *
     * @param result instance where result will be stored.
     */
    fun getInstantaneousStdZAsMeasurement(result: M) {
        internalDetector.getInstantaneousStdZAsMeasurement(result)
    }

    /**
     * Gets windowed standard deviation of sensor measurements for each processed triad.
     * This value is updated for each processed sample containing measured standard deviation for
     * the samples within the window.
     */
    val instantaneousStdTriad: T
        get() = internalDetector.instantaneousStdTriad

    /**
     * Gets windowed standard deviation of measurements for each processed triad.
     * This value is updated for each processed sample containing measured standard deviation for
     * the samples within the window.
     *
     * @param result instance where result will be stored.
     */
    fun getInstantaneousStdTriad(result: T) {
        internalDetector.getInstantaneousStdTriad(result)
    }

    /**
     * Gets average time interval between sensor samples expressed in seconds (s).
     * This is only available once detector completes initialization.
     */
    val averageTimeInterval
        get() = if (initialized) {
            timeIntervalEstimator.averageTimeInterval
        } else {
            null
        }

    /**
     * Gets average time interval between sensor samples.
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
        protected set

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
     * Maps error reason to an [ErrorReason]
     *
     * @param reason reason to map from.
     * @return mapped reason.
     */
    protected fun mapErrorReason(reason: TriadStaticIntervalDetector.ErrorReason): ErrorReason {
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
     *
     * @param T an implementation of [IntervalDetector]
     */
    fun interface OnInitializationStartedListener<T : IntervalDetector<T, *, *, *, *, *, *>> {
        /**
         * Called when initial static period starts so that base noise level starts being estimated.
         *
         * @param detector detector that raised the event.
         */
        fun onInitializationStarted(detector: T)
    }

    /**
     * Interface to notify when detector completes its initialization.
     *
     * @param T an implementation of [IntervalDetector]
     */
    fun interface OnInitializationCompletedListener<T : IntervalDetector<T, *, *, *, *, *, *>> {
        /**
         * Called when initial static period successfully completes and base noise level is
         * estimated so that static and dynamic periods can be detected.
         *
         * @param detector detector that raised the event.
         * @param baseNoiseLevel base measurement noise level expressed in meters per squared second
         * (m/s^2) for accelerometer sensor, radians per second (rad/s) for gyroscope sensor and
         * Teslas (T) for magnetometer sensor.
         */
        fun onInitializationCompleted(detector: T, baseNoiseLevel: Double)
    }

    /**
     * Interfae to notify when an error occurs.
     *
     * @param T an implementation of [IntervalDetector]
     */
    fun interface OnErrorListener<T : IntervalDetector<T, *, *, *, *, *, *>> {
        /**
         * Called when an error is detected, either at initialization because excessive changes
         * in sensor measurements are found, or because sensor becomes unreliable.
         * When an error occurs, detector is stopped and needs to be restarted to be used again.
         *
         * @param detector detector that raised the event.
         * @param reason reason why error was detected.
         */
        fun onError(detector: T, reason: ErrorReason)
    }

    /**
     * Interface to notify when a new static interval is detected.
     * Instantaneous average measurements and standard deviation within the averaging window are
     * provided at the time the new static interval is detected (and consequently detector ends
     * a previous dynamic interval or its initialization stage).
     *
     * @param T an implementation of [IntervalDetector]
     */
    fun interface OnStaticIntervalDetectedListener<T : IntervalDetector<T, *, *, *, *, *, *>> {
        /**
         * Called when a static interval has been detected after initialization.
         *
         * @param detector detector that raised the event
         * @param instantaneousAvgX instantaneous average x-coordinate of measurements within the
         * window expressed in meters per squared second (m/s^2) for acceleration, radians per
         * second (rad/s) for angular speed or Teslas (T) for magnetic flux density.
         * @param instantaneousAvgY instantaneous average y-coordinate of measurements within the
         * window expressed in meters per squared second (m/s^2) for acceleration, radians per
         * second (rad/s) for angular speed or Teslas (T) for magnetic flux density.
         * @param instantaneousAvgZ instantaneous average z-coordinate of measurements within the
         * window expressed in meters per squared second (m/s^2) for acceleration, radians per
         * second (rad/s) for angular speed or Teslas (T) for magnetic flux density.
         * @param instantaneousStdX instantaneous standard deviation of x-coordinate measurements
         * within the window expressed in meters per squared second (m/s^2) for acceleration,
         * radians per second (rad/s) for angular speed or Teslas (T) for magnetic flux density.
         * @param instantaneousStdY instantaneous standard deviation of y-coordinate measurements
         * within the window expressed in meters per squared second (m/s^2) for acceleration,
         * radians per second (rad/s) for angular speed or Teslas (T) for magnetic flux density.
         * @param instantaneousStdZ instantaneous standard deviation of z-coordinate measurements
         * within the window expressed in meters per squared second (m/s^2) for acceleration,
         * radians per second (rad/s) for angular speed or Teslas (T) for magnetic flux density.
         */
        fun onStaticIntervalDetected(
            detector: T,
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
     *
     * @param T an implementation of [IntervalDetector]
     */
    fun interface OnDynamicIntervalDetectedListener<T : IntervalDetector<T, *, *, *, *, *, *>> {
        /**
         * Called when a dynamic interval has been detected after initialization.
         *
         * @param detector detector that raised the event.
         * @param instantaneousAvgX instantaneous average x-coordinate of measurements within the
         * window expressed in meters per squared second (m/s^2) for acceleration, radians per
         * second (rad/s) for angular speed or Teslas (T) for magnetic flux density.
         * @param instantaneousAvgY instantaneous average y-coordinate of measurements within the
         * window expressed in meters per squared second (m/s^2) for acceleration, radians per
         * second (rad/s) for angular speed or Teslas (T) for magnetic flux density.
         * @param instantaneousAvgZ instantaneous average z-coordinate of measurements within the
         * window expressed in meters per squared second (m/s^2) for acceleration, radians per
         * second (rad/s) for angular speed or Teslas (T) for magnetic flux density.
         * @param instantaneousStdX instantaneous x-coordinate of standard deviation for
         * measurements within the window expressed in meters per squared second (m/s^2) for
         * acceleration, radians per second (rad/s) for angular speed or Teslas (T) for magnetic
         * flux density.
         * @param instantaneousStdY instantaneous y-coordinate of standard deviation for
         * measurements within the window expressed in meters per squared second (m/s^2) for
         * acceleration, radians per second (rad/s) for angular speed or Teslas (T) for magnetic
         * flux density.
         * @param instantaneousStdZ instantaneous z-coordinate of standard deviation for
         * measurements within the window expressed in meters per squared second (m/s^2) for
         * acceleration, radians per second (rad/s) for angular speed or Teslas (T) for magnetic
         * flux density.
         * @param accumulatedAvgX accumulated average x-coordinate of measurements during last
         * static period expressed in meters per squared second (m/s^2) for acceleration, radians
         * per second (rad/s) for angular speed or Teslas (T) for magnetic flux density.
         * @param accumulatedAvgY accumulated average y-coordinate of measurements during last
         * static period expressed in meters per squared second (m/s^2) for acceleration, radians
         * per second (rad/s) for angular speed or Teslas (T) for magnetic flux density.
         * @param accumulatedAvgZ accumulated average z-coordinate of measurements during last
         * static period expressed in meters per squared second (m/s^2) for acceleration, radians
         * per second (rad/s) for angular speed or Teslas (T) for magnetic flux density.
         * @param accumulatedStdX standard deviation of x-coordinate of accumulated measurements
         * during last static period expressed in meters per squared second (m/s^2) for
         * acceleration, radians per second (rad/s) for angular speed or Teslas (T) for magnetic
         * flux density.
         * @param accumulatedStdY standard deviation of y-coordinate of accumulated measurements
         * during last static period expressed in meters per squared second (m/s^2) for
         * acceleration, radians per second (rad/s) for angular speed or Teslas (T) for magnetic
         * flux density.
         * @param accumulatedStdZ standard deviation of z-coordinate of accumulated measurements
         * during last static period expressed in meters per squared second (m/s^2) for
         * acceleration, radians per second (rad/s) for angular speed or Teslas (T) for magnetic
         * flux density.
         */
        fun onDynamicIntervalDetected(
            detector: T,
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
     *
     * @param T an implementation of [IntervalDetector]
     */
    fun interface OnResetListener<T : IntervalDetector<T, *, *, *, *, *, *>> {
        /**
         * Called when detector is reset.
         *
         * @param detector detector that raised the event.
         */
        fun onReset(detector: T)
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
         * A static interval has been detected, where sensor is considered to be subject to
         * no substantial movement forces.
         */
        STATIC_INTERVAL,

        /**
         * A dynamic interval has been detected, where sensor is considered to be subject to
         * substantial movement forces.
         */
        DYNAMIC_INTERVAL,

        /**
         * Detector has failed. This happens if sensor is subject to sudden movement forces
         * while detector is initializing during the initial static period, if there is too much
         * overall motion during initialization, or if sensor becomes unreliable.
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