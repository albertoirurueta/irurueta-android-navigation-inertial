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
/*
import android.content.Context
import android.location.Location
import com.irurueta.android.navigation.inertial.calibration.intervals.AccelerometerIntervalDetector
import com.irurueta.android.navigation.inertial.collectors.MagnetometerSensorCollector
import com.irurueta.android.navigation.inertial.collectors.SensorDelay
import com.irurueta.navigation.inertial.BodyMagneticFluxDensity
import com.irurueta.navigation.inertial.calibration.StandardDeviationBodyMagneticFluxDensity
import com.irurueta.navigation.inertial.calibration.intervals.thresholdfactor.DefaultMagnetometerQualityScoreMapper
import com.irurueta.navigation.inertial.calibration.intervals.thresholdfactor.QualityScoreMapper
import com.irurueta.navigation.inertial.wmm.WorldMagneticModel
import java.util.*


*/
/**
 * Collects magnetometer measurements by detecting periods when device captures a constant magnetic
 * field (typically when device orientation remains static).
 * Such static periods are used to obtain measurements and solve calibration parameters.
 *
 * @property context Android context.
 * @property sensorType One of the supported magnetometer sensor types.
 * @property sensorDelay Delay of sensor between samples.
 * @property solveCalibrationWhenEnoughMeasurements true to automatically solve calibration once
 * enough measurements are available, false otherwise.
 * @property initializationStartedListener listener to notify when initialization starts.
 * @property initializationCompletedListener listener to notify when initialization completes.
 * @property errorListener listener to notify errors.
 * @property initialHardIronAvailableListener listener to notify when a guess of hard iron values is
 * obtained.
 * @property newCalibrationMeasurementAvailableListener listener to notify when a new calibration
 * measurement is obtained.
 * @property readyToSolveCalibrationListener listener to notify when calibrator is ready to be
 * solved.
 * @property calibrationSolvingStartedListener listener to notify when calibration solving starts.
 * @property calibrationCompletedListener listener to notify when calibration is successfully
 * completed.
 * @property stoppedListener listener to notify when measurement collection stops.
 * @property magnetometerMeasurementListener listener to notify collected magnetometer measurements.
 * @property qualityScoreMapper mapper to convert collected measurements into quality scores,
 * based on the amount ot standard deviation (the larger the variability, the worse the score
 * will be).
 *//*

class StaticIntervalMagnetometerCalibrator private constructor(
    val context: Context,
    val sensorType: MagnetometerSensorCollector.SensorType,
    val sensorDelay: SensorDelay,
    val solveCalibrationWhenEnoughMeasurements: Boolean,
    var initializationStartedListener: OnInitializationStartedListener?,
    var initializationCompletedListener: OnInitializationCompletedListener?,
    var errorListener: OnErrorListener?,
    var initialHardIronAvailableListener: OnInitialHardIronAvailableListener?,
    var newCalibrationMeasurementAvailableListener: OnNewCalibrationMeasurementAvailableListener?,
    var readyToSolveCalibrationListener: OnReadyToSolveCalibrationListener?,
    var calibrationSolvingStartedListener: OnCalibrationSolvingStartedListener?,
    var calibrationCompletedListener: OnCalibrationCompletedListener?,
    var stoppedListener: OnStoppedListener?,
    var magnetometerMeasurementListener: MagnetometerSensorCollector.OnMeasurementListener?,
    var qualityScoreMapper: QualityScoreMapper<StandardDeviationBodyMagneticFluxDensity>
) {
    */
/**
     * Constructor.
     *
     * @param context Android context.
     * @param location Current device location.
     * @param timestamp Current timestamp
     * @param worldMagneticModel Earth's magnetic model. Null to use default model.
     * @param sensorType One of the supported magnetometer sensor types.
     * @param sensorDelay Delay of sensor between samples.
     * @param solveCalibrationWhenEnoughMeasurements true to automatically solve calibration once
     * enough measurements are available, false otherwise.
     * @param initializationStartedListener listener to notify when initialization starts.
     * @param initializationCompletedListener listener to notify when initialization completes.
     * @param errorListener listener to notify errors.
     * @param initialHardIronAvailableListener listener to notify when a guess of hard iron values is
     * obtained.
     * @param newCalibrationMeasurementAvailableListener listener to notify when a new calibration
     * measurement is obtained.
     * @param readyToSolveCalibrationListener listener to notify when calibrator is ready to be
     * solved.
     * @param calibrationSolvingStartedListener listener to notify when calibration solving starts.
     * @param calibrationCompletedListener listener to notify when calibration is successfully
     * completed.
     * @param stoppedListener listener to notify when measurement collection stops.
     * @param magnetometerMeasurementListener listener to notify collected magnetometer measurements.
     * @param qualityScoreMapper mapper to convert collected measurements into quality scores,
     * based on the amount ot standard deviation (the larger the variability, the worse the score
     * will be).
     *//*

    constructor(
        context: Context,
        location: Location,
        timestamp: Date = Date(),
        worldMagneticModel: WorldMagneticModel? = null,
        sensorType: MagnetometerSensorCollector.SensorType =
            MagnetometerSensorCollector.SensorType.MAGNETOMETER,
        sensorDelay: SensorDelay = SensorDelay.FASTEST,
        solveCalibrationWhenEnoughMeasurements: Boolean = true,
        isGroundTruthInitialHardIron: Boolean = false,
        initializationStartedListener: OnInitializationStartedListener? = null,
        initializationCompletedListener: OnInitializationCompletedListener? = null,
        errorListener: OnErrorListener? = null,
        initialHardIronAvailableListener: OnInitialHardIronAvailableListener? = null,
        newCalibrationMeasurementAvailableListener: OnNewCalibrationMeasurementAvailableListener? = null,
        readyToSolveCalibrationListener: OnReadyToSolveCalibrationListener? = null,
        calibrationSolvingStartedListener: OnCalibrationSolvingStartedListener? = null,
        calibrationCompletedListener: OnCalibrationCompletedListener? = null,
        stoppedListener: OnStoppedListener? = null,
        magnetometerMeasurementListener: MagnetometerSensorCollector.OnMeasurementListener? = null,
        qualityScoreMapper: QualityScoreMapper<StandardDeviationBodyMagneticFluxDensity> = DefaultMagnetometerQualityScoreMapper()
    ) : this(
        context,
        sensorType,
        sensorDelay,
        solveCalibrationWhenEnoughMeasurements,
        initializationStartedListener,
        initializationCompletedListener,
        errorListener,
        initialHardIronAvailableListener,
        newCalibrationMeasurementAvailableListener,
        readyToSolveCalibrationListener,
        calibrationSolvingStartedListener,
        calibrationCompletedListener,
        stoppedListener,
        magnetometerMeasurementListener,
        qualityScoreMapper
    ) {
        */
/*this.location = location
        this.timestamp = timestamp
        this.worldMagneticModel = worldMagneticModel
        this.isGroundTruthInitialHardIron = isGroundTruthInitialHardIron*//*

    }

    */
/**
     * Listener used by internal detector to handle events when initialization starts.
     *//*

    private val intervalDetectorInitializationStartedListener =
        AccelerometerIntervalDetector.OnInitializationStartedListener {
            initializationStartedListener?.onInitializationStarted(
                this@StaticIntervalMagnetometerCalibrator
            )
        }

    */
/**
     * Listener used by internal interval detector to handle events when initialization is
     * completed.
     *//*

    private val intervalDetectorInitializationCompletedListener =
        AccelerometerIntervalDetector.OnInitializationCompletedListener { _, _ ->
            initializationCompletedListener?.onInitializationCompleted(
                this@StaticIntervalMagnetometerCalibrator
            )
        }

    */
/**
     * Listener used by the internal interval detector to handle errors during interval detection.
     *//*

    private val intervalDetectorErrorListener = AccelerometerIntervalDetector.OnErrorListener { _, reason ->
        //stop()
        //errorListener?.onError(this@StaticIntervalMagnetometerCalibrator, mapErrorReason(reason))
    }

    */
/**
     * Listener used by the internal interval detector when a static period ends and a dynamic
     * period starts. This listener contains accumulated magnetometer average values during static
     * period, that will be used as a measurement to solve calibration.
     *//*

    private val intervalDetectorDynamicIntervalDetectorListener =
        AccelerometerIntervalDetector.OnDynamicIntervalDetectedListener { _, _, _, _, _, _, _, accumulatedAvgX, accumulatedAvgY, accumulatedAvgZ, accumulatedStdX, accumulatedStdY, accumulatedStdZ ->
            val b = BodyMagneticFluxDensity()
            val measurement = StandardDeviationBodyMagneticFluxDensity()
        }

    */
/**
     * Interface to notify when calibrator starts initialization.
     *//*

    fun interface OnInitializationStartedListener {
        */
/**
         * Called when calibrator starts initialization to determine base noise level when device
         * remains static.
         *
         * @param calibrator calibrator that raised the event.
         *//*

        fun onInitializationStarted(calibrator: StaticIntervalMagnetometerCalibrator)
    }

    */
/**
     * Interface to notify when calibrator successfully completes initialization.
     *//*

    fun interface OnInitializationCompletedListener {
        */
/**
         * Called when calibrator successfully completes initialization to determine base noise
         * level when device remains static.
         *
         * @param calibrator calibrator that raised the event.
         *//*

        fun onInitializationCompleted(calibrator: StaticIntervalMagnetometerCalibrator)
    }

    */
/**
     * Interface to notify when an error occurs.
     *//*

    fun interface OnErrorListener {
        */
/**
         * Called when an error is detected, either at initialization because excessive noise
         * is detected, because magnetometer sensor becomes unreliable or because obtained
         * measurements produce a numerically unstable calibration solution.
         *
         * @param calibrator calibrator that raised the event.
         * @param errorReason reason why error was detected.
         *//*

        fun onError(calibrator: StaticIntervalMagnetometerCalibrator, errorReason: ErrorReason)
    }

    */
/**
     * Interface to notify when initial hard iron guess is available.
     * If [isGroundTruthInitialHardIron] is true, then initial hard iron is considered the true
     * value after solving calibration, otherwise, initial hard iron is considered only an initial
     * guess.
     *//*

    fun interface OnInitialHardIronAvailableListener {
        */
/**
         * Called when initial hard iron is available.
         * If [isGroundTruthInitialHardIron] is true, then initial hard iron is considered the true
         * value after solving calibration, otherwise, initial hard iron is considered only on
         * initial guess.
         *
         * @param calibrator calibrator that raised the event.
         * @param hardIronX x-coordinate of hard iron expressed in micro-Teslas (µT).
         * @param hardIronY y-coordinate of hard iron expressed in micro-Teslas (µT).
         * @param hardIronZ z-coordinate of hard iron expressed in micro-Teslas (µT).
         *//*

        fun onInitialHardIronAvailable(
            calibrator: StaticIntervalMagnetometerCalibrator,
            hardIronX: Double,
            hardIronY: Double,
            hardIronZ: Double
        )
    }

    */
/**
     * Interface to notify when a new measurement is obtained for calibration purposes.
     * Such measurements are determined each time a static period finishes (when sensor magnetic
     * measurements stop being static).
     * When enough of these measurements are obtained, calibration can actually be solved.
     *//*

    fun interface OnNewCalibrationMeasurementAvailableListener {
        */
/**
         * Called when a new measurement for calibration is found.
         * A new measurement each time a static period finishes (when sensor magnetic measurements
         * stop being static).
         * When enough of these measurements are obtained, calibration can actually be solved.
         * This listener can be used to modify each measurement as it is being collected.
         * Notice that changes to new measurements should be done carefully as they might affect
         * result of solved calibration.
         *
         * @param calibrator calibrator that raised the event.
         * @param newMeasurement new measurement that has been found.
         * @param measurementsFoundSoFar number of measurements that have been found so far.
         * @param requiredMeasurements required number of measurements to solve calibration.
         *//*

        fun onNewCalibrationMeasurementAvailable(
            calibrator: StaticIntervalMagnetometerCalibrator,
            newMeasurement: StandardDeviationBodyMagneticFluxDensity,
            measurementsFoundSoFar: Int,
            requiredMeasurements: Int
        )
    }

    */
/**
     * Interface to notify when enough measurements are obtained to start solving calibration.
     *//*

    fun interface OnReadyToSolveCalibrationListener {
        */
/**
         * Called when enough measurements are obtained to start solving calibration.
         *
         * @param calibrator calibrator that raised the event.
         *//*

        fun onReadyToSolveCalibration(calibrator: StaticIntervalMagnetometerCalibrator)
    }

    */
/**
     * Interface to notify when calibration starts being solved.
     *//*

    fun interface OnCalibrationSolvingStartedListener {
        */
/**
         * Called when calibration starts being solved after enough measurements are found.
         * Calibration can automatically started when enough measurements are available if
         * [solveCalibrationWhenEnoughMeasurements] is true, otherwise [calibrate] must be called
         * after enough measurements are found, which raises this event.
         *
         * @param calibration calibrator that raised the event.
         *//*

        fun onCalibrationSolvingStarted(calibration: StaticIntervalMagnetometerCalibrator)
    }

    */
/**
     * Interface to notify when calibration is solved and completed.
     *//*

    fun interface OnCalibrationCompletedListener {
        */
/**
         * Called when calibration successfully completes.
         *
         * @param calibrator calibrator that raised the event.
         *//*

        fun onCalibrationCompleted(calibrator: StaticIntervalMagnetometerCalibrator)
    }

    */
/**
     * Interface to notify when measurement collection stops.
     * This happens automatically when enough measurements are found after periods when
     * device stops being static, or if an error occurs.
     *//*

    fun interface OnStoppedListener {
        */
/**
         * Called when calibration on measurement collection stops.
         * This happens automatically when enough measurements are found after periods when
         * device stops being static, or if an error occurs.
         *
         * @param calibrator calibrator that raised the event.
         *//*

        fun onStopped(calibrator: StaticIntervalMagnetometerCalibrator)
    }

    */
/**
     * Reasons why this calibrator can fail
     *//*

    enum class ErrorReason {
        */
/**
         * If a sudden movement is detected during initialization.
         *//*

        SUDDEN_EXCESSIVE_MOVEMENT_DETECTED_DURING_INITIALIZATION,

        */
/**
         * If overall noise level is excessive during initialization.
         *//*

        OVERALL_EXCESSIVE_MOVEMENT_DETECTED_DURING_INITIALIZATION,

        */
/**
         * If sensor becomes unreliable.
         *//*

        UNRELIABLE_SENSOR,

        */
/**
         * Occurs if obtained measurements cannot yield a numerically stable solution
         * during calibration estimation.
         *//*

        NUMERICAL_INSTABILITY_DURING_CALIBRATION
    }
}*/
