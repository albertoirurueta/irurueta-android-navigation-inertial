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
package com.irurueta.android.navigation.inertial.calibration.intervals.measurements

import android.content.Context
import com.irurueta.android.navigation.inertial.ENUtoNEDTriadConverter
import com.irurueta.android.navigation.inertial.calibration.intervals.ErrorReason
import com.irurueta.android.navigation.inertial.calibration.intervals.Status
import com.irurueta.android.navigation.inertial.collectors.*
import com.irurueta.navigation.inertial.BodyKinematics
import com.irurueta.navigation.inertial.calibration.*
import com.irurueta.navigation.inertial.calibration.generators.AccelerometerAndGyroscopeMeasurementsGenerator
import com.irurueta.navigation.inertial.calibration.generators.AccelerometerAndGyroscopeMeasurementsGeneratorListener
import com.irurueta.navigation.inertial.calibration.intervals.TriadStaticIntervalDetector
import com.irurueta.navigation.inertial.calibration.noise.AccumulatedAngularSpeedTriadNoiseEstimator
import com.irurueta.units.Acceleration
import com.irurueta.units.AngularSpeed
import com.irurueta.units.AngularSpeedUnit

/**
 * Generates measurements that can later be used both by accelerometer and gyroscope calibrators.
 * Measurements are generated by taking into account static and dynamic intervals on the device,
 * when the device is kept static (e.g. motionless), or when some force is applied to the device
 * changing its position or orientation.
 * Static and dynamic intervals are always measured using the accelerometer.
 * Additionally, the gyroscope is also used, and measurements are generated to calibrate both
 * accelerometers and gyroscopes.
 * Measurement generator converts device ENU measurements into measurements expressed in local
 * tangent plane NED coordinates.
 *
 * @property context Android context.
 * @property accelerometerSensorType One of the supported accelerometer sensor types.
 * @property accelerometerSensorDelay Delay of accelerometer sensor between samples.
 * @property gyroscopeSensorType One of the supported gyroscope sensor types.
 * @property gyroscopeSensorDelay Delay of gyroscope sensor between samples.
 * @property initializationStartedListener listener to notify when initialization starts.
 * @property initializationCompletedListener listener to notify when initialization successfully
 * completes.
 * @property errorListener listener to notify errors.
 * @property staticIntervalDetectedListener listener to notify when a static interval is detected.
 * @property dynamicIntervalDetectedListener listener to notify when a dynamic interval is detected.
 * @property staticIntervalSkippedListener listener to notify when a static interval is skipped if
 * its duration is too short.
 * @property dynamicIntervalSkippedListener listener to notify when a dynamic interval is skipped if
 * its duration is too long.
 * @property generatedAccelerometerMeasurementListener listener to notify when a new accelerometer
 * calibration measurement is generated.
 * @property generatedGyroscopeMeasurementListener listener to notify when a new gyroscope
 * calibration measurement is generated.
 * @property resetListener listener to notify when generator is restarted.
 * @property accelerometerMeasurementListener listener to notify when a new accelerometer
 * measurement is received.
 * @property gyroscopeMeasurementListener listener to notify when a new gyroscope measurement is
 * received.
 * @property accuracyChangedListener listener to notify when sensor accuracy changes.
 */
class AccelerometerAndGyroscopeMeasurementGenerator(
    context: Context,
    accelerometerSensorType: AccelerometerSensorCollector.SensorType =
        AccelerometerSensorCollector.SensorType.ACCELEROMETER,
    accelerometerSensorDelay: SensorDelay = SensorDelay.FASTEST,
    val gyroscopeSensorType: GyroscopeSensorCollector.SensorType =
        GyroscopeSensorCollector.SensorType.GYROSCOPE,
    val gyroscopeSensorDelay: SensorDelay = SensorDelay.FASTEST,
    var initializationStartedListener: OnInitializationStartedListener? = null,
    var initializationCompletedListener: OnInitializationCompletedListener? = null,
    var errorListener: OnErrorListener? = null,
    var staticIntervalDetectedListener: OnStaticIntervalDetectedListener? = null,
    var dynamicIntervalDetectedListener: OnDynamicIntervalDetectedListener? = null,
    var staticIntervalSkippedListener: OnStaticIntervalSkippedListener? = null,
    var dynamicIntervalSkippedListener: OnDynamicIntervalSkippedListener? = null,
    var generatedAccelerometerMeasurementListener: OnGeneratedAccelerometerMeasurementListener? = null,
    var generatedGyroscopeMeasurementListener: OnGeneratedGyroscopeMeasurementListener? = null,
    var resetListener: OnResetListener? = null,
    var accelerometerMeasurementListener: AccelerometerSensorCollector.OnMeasurementListener? = null,
    var gyroscopeMeasurementListener: GyroscopeSensorCollector.OnMeasurementListener? = null,
    accuracyChangedListener: SensorCollector.OnAccuracyChangedListener? = null
) : CalibrationMeasurementGenerator<TimedBodyKinematics>(
    context,
    accelerometerSensorType,
    accelerometerSensorDelay,
    accuracyChangedListener
) {
    /**
     * Triad containing acceleration samples converted from device ENU coordinates to local plane
     * NED coordinates.
     * This is reused for performance reasons.
     */
    private val acceleration = AccelerationTriad()

    /**
     * Triad containing angular speed samples converted from device ENU coordinates to local plane
     * NED coordinates.
     * This is reused for performance reasons.
     */
    private val angularSpeed = AngularSpeedTriad()

    /**
     * Listener for internal measurement generator.
     */
    private val measurementsGeneratorListener =
        object : AccelerometerAndGyroscopeMeasurementsGeneratorListener {
            override fun onInitializationStarted(generator: AccelerometerAndGyroscopeMeasurementsGenerator?) {
                initializationStartedListener?.onInitializationStarted(
                    this@AccelerometerAndGyroscopeMeasurementGenerator
                )
            }

            override fun onInitializationCompleted(
                generator: AccelerometerAndGyroscopeMeasurementsGenerator?,
                accelerometerBaseNoiseLevel: Double
            ) {
                initializationCompletedListener?.onInitializationCompleted(
                    this@AccelerometerAndGyroscopeMeasurementGenerator,
                    accelerometerBaseNoiseLevel
                )
            }

            override fun onError(
                generator: AccelerometerAndGyroscopeMeasurementsGenerator?,
                reason: TriadStaticIntervalDetector.ErrorReason
            ) {
                errorListener?.onError(
                    this@AccelerometerAndGyroscopeMeasurementGenerator,
                    mapErrorReason(reason)
                )
            }

            override fun onStaticIntervalDetected(
                generator: AccelerometerAndGyroscopeMeasurementsGenerator?
            ) {
                staticIntervalDetectedListener?.onStaticIntervalDetected(
                    this@AccelerometerAndGyroscopeMeasurementGenerator
                )
            }

            override fun onDynamicIntervalDetected(generator: AccelerometerAndGyroscopeMeasurementsGenerator?) {
                dynamicIntervalDetectedListener?.onDynamicIntervalDetected(
                    this@AccelerometerAndGyroscopeMeasurementGenerator
                )
            }

            override fun onStaticIntervalSkipped(generator: AccelerometerAndGyroscopeMeasurementsGenerator?) {
                staticIntervalSkippedListener?.onStaticIntervalSkipped(
                    this@AccelerometerAndGyroscopeMeasurementGenerator
                )
            }

            override fun onDynamicIntervalSkipped(generator: AccelerometerAndGyroscopeMeasurementsGenerator?) {
                dynamicIntervalSkippedListener?.onDynamicIntervalSkipped(
                    this@AccelerometerAndGyroscopeMeasurementGenerator
                )
            }

            override fun onGeneratedAccelerometerMeasurement(
                generator: AccelerometerAndGyroscopeMeasurementsGenerator?,
                measurement: StandardDeviationBodyKinematics
            ) {
                generatedAccelerometerMeasurementListener?.onGeneratedAccelerometerMeasurement(
                    this@AccelerometerAndGyroscopeMeasurementGenerator,
                    measurement
                )
            }

            override fun onGeneratedGyroscopeMeasurement(
                generator: AccelerometerAndGyroscopeMeasurementsGenerator?,
                measurement: BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>
            ) {
                generatedGyroscopeMeasurementListener?.onGeneratedGyroscopeMeasurement(
                    this@AccelerometerAndGyroscopeMeasurementGenerator,
                    measurement
                )
            }

            override fun onReset(generator: AccelerometerAndGyroscopeMeasurementsGenerator?) {
                resetListener?.onReset(this@AccelerometerAndGyroscopeMeasurementGenerator)
            }
        }

    /**
     * Internal measurements generator for accelerometer and gyroscope calibration.
     */
    private val measurementsGenerator =
        AccelerometerAndGyroscopeMeasurementsGenerator(measurementsGeneratorListener)

    /**
     * Processes sample in internal measurements generator.
     */
    override fun processSampleInInternalGenerator() {
        measurementsGenerator.process(sample)
    }

    /**
     * Updates time interval of internal generator once the interval detector has been initialized.
     */
    override fun updateTimeIntervalOfInternalGenerator() {
        measurementsGenerator.timeInterval =
            accelerometerTimeIntervalEstimator.averageTimeInterval
    }

    /**
     * Notifies that an accelerometer measurement has been received.
     *
     * @param ax acceleration on device x-axis expressed in meters per squared second (m/s^2).
     * @param ay acceleration on device y-axis expressed in meters per squared second (m/s^2).
     * @param az acceleration on device z-axis expressed in meters per squared second (m/s^2).
     * @param bx bias on device x-axis expressed in meters per squared second (m/s^2). Only
     * available when using [AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED].
     * If available, this value remains constant with calibrated bias value.
     * @param by bias on device y-axis expressed in meters per squared second (m/s^2). Only
     * available when using [AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED].
     * If available, this value remains constant with calibrated bias value.
     * @param bz bias on device z-axis expressed in meters per squared second (m/s^2). Only
     * available when using [AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED].
     * If available, this value remains constant with calibrated bias value.
     * @param timestamp time in nanoseconds at which the measurement was made. Each measurement
     * will be monotonically increasing using the same time base as
     * [android.os.SystemClock.elapsedRealtimeNanos].
     * @param accuracy accelerometer sensor accuracy.
     */
    override fun notifyAccelerometerMeasurement(
        ax: Float,
        ay: Float,
        az: Float,
        bx: Float?,
        by: Float?,
        bz: Float?,
        timestamp: Long,
        accuracy: SensorAccuracy?
    ) {
        accelerometerMeasurementListener?.onMeasurement(
            ax,
            ay,
            az,
            bx,
            by,
            bz,
            timestamp,
            accuracy
        )
    }

    /**
     * Internal listener for gyroscope sensor collector.
     * Handles measurements collected by the gyroscope sensor so that they are processed
     * by the internal measurement generator.
     */
    private val gyroscopeCollectorMeasurementListener =
        GyroscopeSensorCollector.OnMeasurementListener { wx, wy, wz, bx, by, bz, timestamp, accuracy ->
            // convert from device ENU coordinates to local plane NED coordinates
            ENUtoNEDTriadConverter.convert(
                wx.toDouble(),
                wy.toDouble(),
                wz.toDouble(),
                angularSpeed
            )

            // set gyroscope information
            kinematics.angularRateX = angularSpeed.valueX
            kinematics.angularRateY = angularSpeed.valueY
            kinematics.angularRateZ = angularSpeed.valueZ

            if (status == Status.INITIALIZING) {
                gyroscopeAccumulatedNoiseEstimator.addTriad(
                    kinematics.angularRateX,
                    kinematics.angularRateY,
                    kinematics.angularRateZ
                )
            }

            numberOfProcessedGyroscopeMeasurements++

            if (gyroscopeBaseNoiseLevel == null && (status == Status.INITIALIZATION_COMPLETED || status == Status.STATIC_INTERVAL || status == Status.DYNAMIC_INTERVAL)) {
                gyroscopeBaseNoiseLevel =
                    gyroscopeAccumulatedNoiseEstimator.standardDeviationNorm
            }

            gyroscopeMeasurementListener?.onMeasurement(wx, wy, wz, bx, by, bz, timestamp, accuracy)
        }

    /**
     * Notifies that sensor has become unreliable.
     */
    override fun notifyUnreliableSensor() {
        errorListener?.onError(this, ErrorReason.UNRELIABLE_SENSOR)
    }

    /**
     * Gyroscope sensor collector.
     * Collects gyroscope measurements.
     */
    private val gyroscopeCollector = GyroscopeSensorCollector(
        context,
        gyroscopeSensorType,
        gyroscopeSensorDelay,
        gyroscopeCollectorMeasurementListener,
        collectorAccuracyChangedListener
    )

    /**
     * Body kinematics being reused for efficiency purposes.
     */
    private val kinematics = BodyKinematics()

    /**
     * Estimates accumulated average and noise of gyroscope values during static periods.
     */
    private val gyroscopeAccumulatedNoiseEstimator =
        AccumulatedAngularSpeedTriadNoiseEstimator()

    /**
     * Sample used by internal measurement generator.
     */
    override val sample = TimedBodyKinematics()

    /**
     * Gets gyroscope sensor being used to obtain measurements, or null if not available.
     * This can be used to obtain additional information about the sensor.
     */
    val gyroscopeSensor
        get() = gyroscopeCollector.sensor

    /**
     * Number of gyroscope measurements that have been processed.
     */
    var numberOfProcessedGyroscopeMeasurements: Int = 0
        private set

    /**
     * Gets gyroscope measurement base noise level that has been detected during initialization
     * expressed in radians per second (rad/s).
     * This is only available once generator completes initialization.
     */
    var gyroscopeBaseNoiseLevel: Double? = null
        private set

    /**
     * Gets gyroscope measurement base noise level that has been detected during initialization.
     * This is only available once generator completes initialization.
     */
    val gyroscopeBaseNoiseLevelAsMeasurement: AngularSpeed?
        get() {
            val value = gyroscopeBaseNoiseLevel ?: return null
            return AngularSpeed(value, AngularSpeedUnit.RADIANS_PER_SECOND)
        }

    /**
     * Gets gyroscope measurement base noise level that has been detected during initialization.
     * This is only available once generator completes initialization.
     *
     * @param result instance where result will be stored.
     * @return true if result is available, false otherwise.
     */
    fun getGyroscopeBaseNoiseLevelAsMeasurement(result: AngularSpeed): Boolean {
        val value = gyroscopeBaseNoiseLevel ?: return false
        result.value = value
        result.unit = AngularSpeedUnit.RADIANS_PER_SECOND
        return true
    }

    /**
     * Gets or sets minimum number of samples required in a static interval to be taken into
     * account. Smaller static intervals will be discarded.
     *
     * @throws IllegalArgumentException if provided value is less than 2.
     * @throws IllegalStateException if generator is currently running.
     */
    override var minStaticSamples
        get() = measurementsGenerator.minStaticSamples
        @Throws(IllegalArgumentException::class, IllegalStateException::class)
        set(value) {
            check(!running)
            measurementsGenerator.minStaticSamples = value
        }

    /**
     * Gets or sets maximum number of samples allowed in dynamic intervals.
     * Dynamic intervals exceeding this value are discarded.
     *
     * @throws IllegalArgumentException if provided value is less than 2.
     * @throws IllegalStateException if generator is currently running.
     */
    override var maxDynamicSamples
        get() = measurementsGenerator.maxDynamicSamples
        @Throws(IllegalArgumentException::class, IllegalStateException::class)
        set(value) {
            check(!running)
            measurementsGenerator.maxDynamicSamples = value
        }

    /**
     * Gets or sets length of number of samples to keep within the window being processed to
     * determine instantaneous sensor noise level. Window size must always be larger than
     * allowed minimum value, which is 2 and must have an odd value.
     *
     * @throws IllegalArgumentException if provided value is not valid.
     * @throws IllegalStateException if generator is currently running.
     */
    override var windowSize
        get() = measurementsGenerator.windowSize
        @Throws(IllegalArgumentException::class, IllegalStateException::class)
        set(value) {
            check(!running)
            measurementsGenerator.windowSize = value
        }

    /**
     * Gets or sets number of samples to be processed initially while keeping the sensor static in
     * order to find the base noise level when device is static.
     *
     * @throws IllegalArgumentException if provided value is less than
     * [TriadStaticIntervalDetector.MINIMUM_INITIAL_STATIC_SAMPLES].
     * @throws IllegalStateException if generator is currently running.
     */
    override var initialStaticSamples
        get() = measurementsGenerator.initialStaticSamples
        @Throws(IllegalArgumentException::class, IllegalStateException::class)
        set(value) {
            check(!running)
            measurementsGenerator.initialStaticSamples = value
        }

    /**
     * Gets or sets factor to be applied to detected base noise level in order to determine
     * threshold for static/dynamic period changes. This factor is unit-less.
     *
     * @throws IllegalArgumentException if provided value is zero or negative
     * @throws IllegalStateException if generator is currently running.
     */
    override var thresholdFactor
        get() = measurementsGenerator.thresholdFactor
        @Throws(IllegalArgumentException::class, IllegalStateException::class)
        set(value) {
            check(!running)
            measurementsGenerator.thresholdFactor = value
        }

    /**
     * Gets or sets factor to determine that a sudden movement has occurred during initialization if
     * instantaneous noise level exceeds accumulated noise level by this factor amount. This factor
     * is unit-less.
     *
     * @throws IllegalArgumentException if provided value is zero or negative.
     * @throws IllegalStateException if generator is currently running
     */
    override var instantaneousNoiseLevelFactor
        get() = measurementsGenerator.instantaneousNoiseLevelFactor
        @Throws(IllegalArgumentException::class, IllegalStateException::class)
        set(value) {
            check(!running)
            measurementsGenerator.instantaneousNoiseLevelFactor = value
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
    override var baseNoiseLevelAbsoluteThreshold
        get() = measurementsGenerator.baseNoiseLevelAbsoluteThreshold
        @Throws(IllegalArgumentException::class, IllegalStateException::class)
        set(value) {
            check(!running)
            measurementsGenerator.baseNoiseLevelAbsoluteThreshold = value
        }

    /**
     * Gets or sets overall absolute threshold to determine whether there has been excessive motion
     * during the whole initialization phase. Failure will be detected if estimated base noise level
     * exceeds this threshold when initialization completes.
     *
     * @throws IllegalArgumentException if provided value is zero or negative.
     * @throws IllegalStateException if detector is currently running.
     */
    override var baseNoiseLevelAbsoluteThresholdAsMeasurement: Acceleration
        get() = measurementsGenerator.baseNoiseLevelAbsoluteThresholdAsMeasurement
        @Throws(IllegalArgumentException::class, IllegalStateException::class)
        set(value) {
            check(!running)
            measurementsGenerator.setBaseNoiseLevelAbsoluteThreshold(value)
        }

    /**
     * Gets overall absolute threshold to determine whether there has been excessive motion during
     * the whole initialization phase. Failure will be detected if estimated base noise level
     * exceeds this threshold when initialization completes.
     *
     * @param result instance where result will be stored.
     */
    override fun getBaseNoiseLevelAbsoluteThresholdAsMeasurement(result: Acceleration) {
        measurementsGenerator.getBaseNoiseLevelAbsoluteThresholdAsMeasurement(result)
    }

    /**
     * Gets accelerometer measurement base noise level that has been detected during initialization
     * expressed in meters per squared second (m/s^2).
     * This is only available once detector completes initialization.
     */
    override val accelerometerBaseNoiseLevel
        get() = if (initialized) {
            measurementsGenerator.accelerometerBaseNoiseLevel
        } else {
            null
        }

    /**
     * Gets sensor measurement base noise level that has been detected during initialization.
     * This is only available once detector completes initialization.
     */
    override val accelerometerBaseNoiseLevelAsMeasurement
        get() = if (initialized) {
            measurementsGenerator.accelerometerBaseNoiseLevelAsMeasurement
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
    override fun getAccelerometerBaseNoiseLevelAsMeasurement(result: Acceleration): Boolean {
        return if (initialized) {
            measurementsGenerator.getAccelerometerBaseNoiseLevelAsMeasurement(result)
            true
        } else {
            false
        }
    }

    /**
     * Gets measurement base noise level PSD (Power Spectral Density) expressed in (m^2 * s^-3).
     */
    override val accelerometerBaseNoiseLevelPsd
        get() = if (initialized) {
            measurementsGenerator.accelerometerBaseNoiseLevelPsd
        } else {
            null
        }

    /**
     * Gets measurement base noise level root PSD (Power Spectral Density) expressed
     * in (m * s^-1.5).
     * This is only available once detector completes initialization.
     */
    override val accelerometerBaseNoiseLevelRootPsd
        get() = if (initialized) {
            measurementsGenerator.accelerometerBaseNoiseLevelRootPsd
        } else {
            null
        }

    /**
     * Gets estimated threshold to determine static/dynamic period changes expressed in meters per
     * squared second (m/s^2).
     * This is only available once detector completes initialization.
     */
    override val threshold
        get() = if (initialized) {
            measurementsGenerator.threshold
        } else {
            null
        }

    /**
     * Gets estimated threshold to determine static/dynamic period changes.
     * This is only available once detector completes initialization.
     */
    override val thresholdAsMeasurement
        get() = if (initialized) {
            measurementsGenerator.thresholdAsMeasurement
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
    override fun getThresholdAsMeasurement(result: Acceleration): Boolean {
        return if (initialized) {
            measurementsGenerator.getThresholdAsMeasurement(result)
            true
        } else {
            false
        }
    }

    /**
     * Gets number of samples that have been processed in a static period so far.
     */
    override val processedStaticSamples
        get() = measurementsGenerator.processedStaticSamples

    /**
     * Gets number of samples that have been processed in a dynamic period so far.
     */
    override val processedDynamicSamples
        get() = measurementsGenerator.processedDynamicSamples

    /**
     * Indicates whether last static interval must be skipped.
     */
    override val isStaticIntervalSkipped
        get() = measurementsGenerator.isStaticIntervalSkipped

    /**
     * Indicates whether last dynamic interval must be skipped.
     */
    override val isDynamicIntervalSkipped
        get() = measurementsGenerator.isDynamicIntervalSkipped

    /**
     * Gets status of measurement generator.
     * Initially the generator will be idle.
     * Once it starts, it will start the initialization phase, and once
     * initialization is complete, it will switch between static or dynamic interval
     * until generator is stopped or an error occurs.
     */
    override val status: Status
        get() = Status.mapStatus(measurementsGenerator.status, unreliable)

    /**
     * Starts collection of sensor measurements.
     *
     * @throws IllegalStateException if generator is already running or sensor is not available.
     */
    @Throws(IllegalStateException::class)
    override fun start() {
        check(!running)

        reset()

        running = true
        if (!accelerometerCollector.start()) {
            running = false
            throw IllegalStateException("Unavailable accelerometer sensor")
        }
        if (!gyroscopeCollector.start()) {
            stop()
            throw IllegalStateException("Unavailable gyroscope sensor")
        }
    }

    /**
     * Stops collection of sensor measurements.
     */
    override fun stop() {
        gyroscopeCollector.stop()
        accelerometerCollector.stop()
        running = false
    }

    /**
     * Resets generator to its initial state.
     */
    private fun reset() {
        accelerometerTimeIntervalEstimator.totalSamples = Integer.MAX_VALUE
        accelerometerTimeIntervalEstimator.reset()
        measurementsGenerator.reset()
        unreliable = false
        initialAccelerometerTimestamp = 0L
        numberOfProcessedAccelerometerMeasurements = 0
        numberOfProcessedGyroscopeMeasurements = 0
        initialized = false
        gyroscopeBaseNoiseLevel = null
        gyroscopeAccumulatedNoiseEstimator.reset()
    }

    /**
     * Processes an accelerometer measurement to generate an instance of type [TimedBodyKinematics]
     * to be used by the internal measurement generator.
     *
     * @param ax acceleration on device x-axis expressed in meters per squared second (m/s^2).
     * @param ay acceleration on device y-axis expressed in meters per squared second (m/s^2).
     * @param az acceleration on device z-axis expressed in meters per squared second (m/s^2).
     * @param diffSeconds elapsed seconds since accelerometer started.
     * @param result instance where processed sample result will be stored.
     */
    override fun processSample(
        ax: Float,
        ay: Float,
        az: Float,
        diffSeconds: Double,
        result: TimedBodyKinematics
    ) {
        // convert from device ENU coordinates to local plane NED coordinates
        ENUtoNEDTriadConverter.convert(ax.toDouble(), ay.toDouble(), az.toDouble(), acceleration)

        // set accelerometer information (gyroscope information is filled on the corresponding
        // gyroscope listener)
        kinematics.fx = acceleration.valueX
        kinematics.fy = acceleration.valueY
        kinematics.fz = acceleration.valueZ
        result.kinematics = kinematics
        result.timestampSeconds = diffSeconds
    }

    /**
     * Interface to notify when generator starts initialization.
     */
    fun interface OnInitializationStartedListener {
        /**
         * Called when initial static period starts so that base noise level starts being estimated.
         *
         * @param generator generator that raised the event.
         */
        fun onInitializationStarted(generator: AccelerometerAndGyroscopeMeasurementGenerator)
    }

    /**
     * Interface to notify when generator completes its initialization.
     */
    fun interface OnInitializationCompletedListener {
        /**
         * Called when initial static period successfully completes and accelerometer base noise
         * level is estimated so that static and dynamic periods can be detected.
         *
         * @param generator generator that raised the event.
         * @param baseNoiseLevel base measurement noise level expressed in meters per squared second
         * (m/s^2).
         */
        fun onInitializationCompleted(
            generator: AccelerometerAndGyroscopeMeasurementGenerator,
            baseNoiseLevel: Double
        )
    }

    /**
     * Interface to notify when an error occurs.
     */
    fun interface OnErrorListener {
        /**
         * Called when an error is detected, either at initialization because excessive changes in
         * sensor measurements are found, or because sensor becomes unreliable.
         * When an error occurs, generator is stopped and needs to be restarted to be used again.
         *
         * @param generator generator that raised the event.
         * @param reason reason why error was detected.
         */
        fun onError(generator: AccelerometerAndGyroscopeMeasurementGenerator, reason: ErrorReason)
    }

    /**
     * Interface to notify a new static interval is detected.
     */
    fun interface OnStaticIntervalDetectedListener {
        /**
         * Called when a static interval has been detected after initialization.
         *
         * @param generator generator that raised the event.
         */
        fun onStaticIntervalDetected(generator: AccelerometerAndGyroscopeMeasurementGenerator)
    }

    /**
     * Interface to notify when a dynamic interval is detected.
     */
    fun interface OnDynamicIntervalDetectedListener {
        /**
         * Called when a dynamic interval has been detected after initialization.
         *
         * @param generator generator that raised the event.
         */
        fun onDynamicIntervalDetected(generator: AccelerometerAndGyroscopeMeasurementGenerator)
    }

    /**
     * Interface to notify when a static interval is skipped.
     * This happens when interval is too short.
     */
    fun interface OnStaticIntervalSkippedListener {
        /**
         * Called when a detected static interval is skipped because there are not enough samples to
         * be processed.
         *
         * @param generator generator that raised the event.
         */
        fun onStaticIntervalSkipped(generator: AccelerometerAndGyroscopeMeasurementGenerator)
    }

    /**
     * Interface to notify when a dynamic interval is skipped.
     * This happens when interval is too long.
     */
    fun interface OnDynamicIntervalSkippedListener {
        /**
         * Called when a detected dynamic interval is skipped because it has too many samples in it.
         */
        fun onDynamicIntervalSkipped(generator: AccelerometerAndGyroscopeMeasurementGenerator)
    }

    /**
     * Interface to notify when a new accelerometer calibration measurement is generated.
     */
    fun interface OnGeneratedAccelerometerMeasurementListener {
        /**
         * Called when a new accelerometer calibration measurement is generated.
         *
         * @param generator generator that raised the event.
         * @param measurement generated accelerometer calibration measurement.
         */
        fun onGeneratedAccelerometerMeasurement(
            generator: AccelerometerAndGyroscopeMeasurementGenerator,
            measurement: StandardDeviationBodyKinematics
        )
    }

    /**
     * Interface to notify when a new gyroscope calibration measurement is generated.
     */
    fun interface OnGeneratedGyroscopeMeasurementListener {
        /**
         * Called when a new gyroscope calibration measurement is generated.
         *
         * @param generator generator that raised the event.
         * @param measurement generated gyroscope calibration measurement.
         */
        fun onGeneratedGyroscopeMeasurement(
            generator: AccelerometerAndGyroscopeMeasurementGenerator,
            measurement: BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>
        )
    }

    /**
     * Interface to notify when generator is reset (occurs when starting after stopping the
     * generator).
     */
    fun interface OnResetListener {
        /**
         * Called when generator is reset.
         *
         * @param generator generator that raised the event.
         */
        fun onReset(generator: AccelerometerAndGyroscopeMeasurementGenerator)
    }
}