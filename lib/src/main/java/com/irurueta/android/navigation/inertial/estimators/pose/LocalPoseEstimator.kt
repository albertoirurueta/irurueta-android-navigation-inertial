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
package com.irurueta.android.navigation.inertial.estimators.pose

import android.content.Context
import android.location.Location
import com.irurueta.algebra.Matrix
import com.irurueta.algebra.Utils
import com.irurueta.android.navigation.inertial.ENUtoNEDTriadConverter
import com.irurueta.android.navigation.inertial.collectors.AccelerometerSensorCollector
import com.irurueta.android.navigation.inertial.collectors.GyroscopeSensorCollector
import com.irurueta.android.navigation.inertial.collectors.MagnetometerSensorCollector
import com.irurueta.android.navigation.inertial.collectors.SensorDelay
import com.irurueta.android.navigation.inertial.estimators.attitude.FusedGeomagneticAttitudeEstimator
import com.irurueta.android.navigation.inertial.estimators.attitude.GravityEstimator
import com.irurueta.android.navigation.inertial.estimators.filter.AveragingFilter
import com.irurueta.android.navigation.inertial.estimators.filter.LowPassAveragingFilter
import com.irurueta.android.navigation.inertial.toNEDPosition
import com.irurueta.geometry.EuclideanTransformation3D
import com.irurueta.geometry.InhomogeneousPoint3D
import com.irurueta.geometry.Quaternion
import com.irurueta.geometry.Rotation3D
import com.irurueta.navigation.frames.*
import com.irurueta.navigation.frames.converters.NEDtoECEFFrameConverter
import com.irurueta.navigation.inertial.calibration.AccelerationTriad
import com.irurueta.navigation.inertial.calibration.AngularSpeedTriad
import com.irurueta.navigation.inertial.estimators.NEDGravityEstimator
import com.irurueta.navigation.inertial.estimators.RadiiOfCurvatureEstimator
import com.irurueta.navigation.inertial.navigators.NEDInertialNavigator
import com.irurueta.navigation.inertial.wmm.WorldMagneticModel
import java.util.*
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.tan

/**
 * Estimates pose with 6DOF (Degrees of Freedom) containing absolute attitude and position using
 * local plane navigation.
 *
 * @property context Android context.
 * @property sensorDelay Delay of sensors between samples.
 * @property accelerometerSensorType One of the supported accelerometer sensor types. It is
 * suggested to avoid using non-calibrated accelerometers.
 * @property gyroscopeSensorType One of the supported gyroscope sensor types. It is suggested to
 * avoid using non-calibrated gyroscopes.
 * @property magnetometerSensorType One of the supported magnetometer sensor types.
 * @property accelerometerAveragingFilter an averaging filter for accelerometer samples to obtain
 * sensed gravity component of specific force.
 * @property estimatePoseTransformation true to estimate 3D metric transformation, false to skip
 * transformation computation.
 * @property poseAvailableListener notifies when a new estimated pose is available.
 * @property accelerometerMeasurementListener listener to notify new accelerometer measurements.
 * @property gyroscopeMeasurementListener listener to notify new gyroscope measurements.
 * @property initialVelocity initial velocity of device expressed in NED coordinates.
 * @param initialLocation initial location value.
 */
class LocalPoseEstimator private constructor(
    override val context: Context,
    override val sensorDelay: SensorDelay,
    override val accelerometerSensorType: AccelerometerSensorCollector.SensorType,
    override val gyroscopeSensorType: GyroscopeSensorCollector.SensorType,
    override val magnetometerSensorType: MagnetometerSensorCollector.SensorType,
    override val accelerometerAveragingFilter: AveragingFilter,
    override val estimatePoseTransformation: Boolean,
    var poseAvailableListener: OnPoseAvailableListener?,
    override var accelerometerMeasurementListener: AccelerometerSensorCollector.OnMeasurementListener?,
    override var gyroscopeMeasurementListener: GyroscopeSensorCollector.OnMeasurementListener?,
    override var gravityEstimationListener: GravityEstimator.OnEstimationListener?,
    override val initialVelocity: NEDVelocity,
    initialLocation: Location
) : AbsolutePoseEstimator {

    /**
     * Constructor.
     *
     * @param context Android context.
     * @param location current device location.
     * @param initialVelocity initial velocity of device expressed in NED coordinates.
     * @param sensorDelay Delay of sensors between samples.
     * @param accelerometerSensorType One of the supported accelerometer sensor types. It is
     * suggested to avoid using non-calibrated accelerometers.
     * @param gyroscopeSensorType One of the supported gyroscope sensor types. It is suggested to
     * avoid using non-calibrated gyroscopes.
     * @param magnetometerSensorType One of the supported magnetometer sensor types.
     * @param accelerometerAveragingFilter an averaging filter for accelerometer samples to obtain
     * sensed gravity component of specific force.
     * @param worldMagneticModel Earth's magnetic model. Null to use default model
     * when [useWorldMagneticModel] is true. If [useWorldMagneticModel] is false, this is ignored.
     * @param timestamp Timestamp when World Magnetic Model will be evaluated to obtain current.
     * Only taken into account if [useWorldMagneticModel] is tue.
     * @param useWorldMagneticModel true so that world magnetic model is taken into account to
     * adjust attitude yaw angle by current magnetic declination based on current World Magnetic
     * Model, location and timestamp, false to ignore declination.
     * @param useAccurateLevelingEstimator true to use accurate leveling, false to use a normal one.
     * @param useAccurateRelativeGyroscopeAttitudeEstimator true to use accurate non-leveled
     * relative attitude estimator, false to use non-accurate non-leveled relative attitude
     * estimator.
     * @param estimatePoseTransformation true to estimate 3D metric pose transformation, false to
     * skip transformation computation.
     * @param poseAvailableListener notifies when a new estimated pose is available.
     * @param accelerometerMeasurementListener listener to notify new accelerometer measurements.
     * @param gyroscopeMeasurementListener listener to notify new gyroscope measurements.
     * @param magnetometerMeasurementListener listener to notify new magnetometer measurements.
     * @param gravityEstimationListener listener to notify when a new gravity estimation is
     * available.
     */
    constructor(
        context: Context,
        location: Location,
        initialVelocity: NEDVelocity = NEDVelocity(),
        sensorDelay: SensorDelay = SensorDelay.GAME,
        accelerometerSensorType: AccelerometerSensorCollector.SensorType =
            AccelerometerSensorCollector.SensorType.ACCELEROMETER,
        gyroscopeSensorType: GyroscopeSensorCollector.SensorType =
            GyroscopeSensorCollector.SensorType.GYROSCOPE,
        magnetometerSensorType: MagnetometerSensorCollector.SensorType =
            MagnetometerSensorCollector.SensorType.MAGNETOMETER,
        accelerometerAveragingFilter: AveragingFilter = LowPassAveragingFilter(),
        worldMagneticModel: WorldMagneticModel? = null,
        timestamp: Date = Date(),
        useWorldMagneticModel: Boolean = false,
        useAccurateLevelingEstimator: Boolean = true,
        useAccurateRelativeGyroscopeAttitudeEstimator: Boolean = true,
        estimatePoseTransformation: Boolean = false,
        poseAvailableListener: OnPoseAvailableListener? = null,
        accelerometerMeasurementListener: AccelerometerSensorCollector.OnMeasurementListener? = null,
        gyroscopeMeasurementListener: GyroscopeSensorCollector.OnMeasurementListener? = null,
        magnetometerMeasurementListener: MagnetometerSensorCollector.OnMeasurementListener? = null,
        gravityEstimationListener: GravityEstimator.OnEstimationListener? = null
    ) : this(
        context,
        sensorDelay,
        accelerometerSensorType,
        gyroscopeSensorType,
        magnetometerSensorType,
        accelerometerAveragingFilter,
        estimatePoseTransformation,
        poseAvailableListener,
        accelerometerMeasurementListener,
        gyroscopeMeasurementListener,
        gravityEstimationListener,
        initialVelocity,
        location
    ) {
        this.magnetometerMeasurementListener = magnetometerMeasurementListener
        this.initialLocation = location
        this.worldMagneticModel = worldMagneticModel
        this.timestamp = timestamp
        this.useWorldMagneticModel = useWorldMagneticModel
        this.useAccurateLevelingEstimator = useAccurateLevelingEstimator
        this.useAccurateRelativeGyroscopeAttitudeEstimator =
            useAccurateRelativeGyroscopeAttitudeEstimator
    }

    /**
     * Acceleration measured by accelerometer sensor.
     */
    private val acceleration = AccelerationTriad()

    /**
     * Angular speed measured by gyroscope sensor.
     */
    private val angularSpeed = AngularSpeedTriad()

    /**
     * True indicates that estimator has been initialized with an initial frame,
     * containing estimated absolute attitude and current device location (if available).
     */
    private var initializedFrame = false

    /**
     * Device frame expressed in ECEF coordinates when estimator starts.
     * This is reused for performance reasons.
     */
    private val initialEcefFrame = ECEFFrame()

    /**
     * Device frame expressed in NED coordinates when estimator starts.
     * This is reused for performance reasons.
     */
    private val initialNedFrame = NEDFrame()

    /**
     * Device frame expressed in ECEF coordinates for previous set of measurements.
     * This is reused for performance reasons.
     */
    private val previousEcefFrame = ECEFFrame()

    /**
     * Device frame expressed in NED coordinates for previous set of measurements.
     * This is reused for performance reasons.
     */
    private val previousNedFrame = NEDFrame()

    /**
     * Current device frame expressed in ECEF coordinates.
     * This is reused for performance reasons.
     */
    private val currentEcefFrame = ECEFFrame()

    /**
     * Current device frame expressed in NED coordinates.
     * This is reused for performance reasons.
     */
    private val currentNedFrame = NEDFrame()

    /**
     * 3D metric transformation relating initial frame and current frame.
     * This is reused for performance reasons.
     */
    private val poseTransformation = EuclideanTransformation3D()

    /**
     * Attitude of previous sample.
     */
    private val previousAttitude = Quaternion()

    /**
     * Attitude of current sample.
     */
    private val currentAttitude = Quaternion()

    /**
     * Initial attitude when estimator starts. This is used to obtain a relative leveled pose
     * respect to the initial attitude.
     */
    private val initialAttitude = Quaternion()

    /**
     * Average attitude between current and previous attitudes.
     */
    private val averageAttitude = Quaternion()

    /**
     * Current estimated specific force related to gravity at current location.
     */
    private val gravity = AccelerationTriad()

    /**
     * Rotation to be reused for ENU / NED coordinates conversion during pose transformation
     * computation.
     */
    private val conversionRotation = ENUtoNEDTriadConverter.conversionRotation

    /**
     * Array containing euler angles. This is reused for performance reasons during pose
     * transformation computation.
     */
    private val eulerAngles = DoubleArray(Rotation3D.INHOM_COORDS)

    /**
     * Contains pose transformation attitude. This is reused for performance reasons.
     */
    private val transformationRotation = Quaternion()

    /**
     * Contains position variation respect to initial position expressed in ECEF coordinates.
     * This is reused for performance reasons during pose transformation computation.
     */
    private val ecefDiffPosition = InhomogeneousPoint3D()

    /**
     * Contains initial attitude in ECEF coordinates system. This is reused for performance reasons
     * during pose transformation computation.
     */
    private val startEcefRotation = Quaternion()

    /**
     * Contains inverse of initial attitude in ECEF coordinates system.
     * This is used to obtain local coordinates for translation during pose transformation
     * computation.
     */
    private val inverseEcefRotation = Quaternion()

    /**
     * Contains position variation expressed in local coordinates.
     * This is reused for performance reasons during pose transformation computation.
     */
    private val localDiffPosition = InhomogeneousPoint3D()

    /**
     * Listener to notify new magnetometer measurements.
     */
    override var magnetometerMeasurementListener: MagnetometerSensorCollector.OnMeasurementListener? =
        null
        set(value) {
            field = value
            absoluteAttitudeEstimator.magnetometerMeasurementListener = value
        }

    /**
     * Gets or sets device initial location
     *
     * @throws IllegalStateException if estimator is running and a null value is set.
     */
    override var initialLocation: Location = initialLocation
        set(value) {
            absoluteAttitudeEstimator.location = value
            field = value
        }

    /**
     * Earth's magnetic model. If null, the default model is used if [useWorldMagneticModel] is
     * true. If [useWorldMagneticModel] is false, this is ignored.
     *
     * @throws IllegalStateException if estimator is running.
     */
    override var worldMagneticModel: WorldMagneticModel? = null
        @Throws(IllegalStateException::class)
        set(value) {
            check(!running)

            absoluteAttitudeEstimator.worldMagneticModel = value
            field = value
        }

    /**
     * Timestamp when World Magnetic Model will be evaluated to obtain current.
     * Only taken into account if [useWorldMagneticModel] is tue.
     */
    override var timestamp: Date = Date()
        set(value) {
            absoluteAttitudeEstimator.timestamp = value
            field = value
        }

    /**
     * Indicates whether world magnetic model is taken into account to adjust attitude yaw angle by
     * current magnetic declination based on current World Magnetic Model, location and timestamp.
     *
     * @throws IllegalStateException if estimator is running.
     */
    override var useWorldMagneticModel: Boolean = false
        @Throws(IllegalStateException::class)
        set(value) {
            check(!running)

            absoluteAttitudeEstimator.useWorldMagneticModel = value
            field = value
        }

    /**
     * Indicates whether accurate leveling must be used or not.
     *
     * @throws IllegalStateException if estimator is running.
     */
    override var useAccurateLevelingEstimator: Boolean = false
        @Throws(IllegalStateException::class)
        set(value) {
            check(!running)

            absoluteAttitudeEstimator.useAccurateLevelingEstimator = value
            field = value
        }

    /**
     * Indicates whether accurate non-leveled relative attitude estimator must be used or not.
     *
     * @throws IllegalStateException if estimator is running.
     */
    override var useAccurateRelativeGyroscopeAttitudeEstimator: Boolean = false
        @Throws(IllegalStateException::class)
        set(value) {
            check(!running)

            absoluteAttitudeEstimator.useAccurateRelativeGyroscopeAttitudeEstimator = value
            field = value
        }

    /**
     * Indicates whether fusion between leveling and relative attitudes occurs based
     * on changing interpolation value that depends on actual relative attitude rotation
     * velocity.
     */
    override var useIndirectAttitudeInterpolation: Boolean = true
        set(value) {
            absoluteAttitudeEstimator.useIndirectInterpolation = value
            field = value
        }

    /**
     * Interpolation value to be used to combine both leveling and relative attitudes.
     * Must be between 0.0 and 1.0 (both included).
     * The closer to 0.0 this value is, the more resemblance the result will have to a pure
     * leveling (which feels more jerky). On the contrary, the closer to 1.0 this value is,
     * the more resemblance the result will have to a pure non-leveled relative attitude (which
     * feels softer but might have arbitrary roll and pitch Euler angles).
     *
     * @throws IllegalArgumentException if value is not between 0.0 and 1.0 (both included).
     */
    override var attitudeInterpolationValue: Double =
        FusedGeomagneticAttitudeEstimator.DEFAULT_INTERPOLATION_VALUE
        @Throws(IllegalArgumentException::class)
        set(value) {
            absoluteAttitudeEstimator.interpolationValue = value
            field = value
        }

    /**
     * Factor to take into account when interpolation value is computed and
     * [useIndirectAttitudeInterpolation] is enabled to determine actual interpolation value based
     * on current relative attitude rotation velocity.
     *
     * @throws IllegalArgumentException if value is zero or negative.
     */
    override var attitudeIndirectInterpolationWeight: Double =
        FusedGeomagneticAttitudeEstimator.DEFAULT_INDIRECT_INTERPOLATION_WEIGHT
        @Throws(IllegalArgumentException::class)
        set(value) {
            absoluteAttitudeEstimator.indirectInterpolationWeight = value
            field = value
        }

    /**
     * Threshold to determine that current leveling appears to be an outlier respect
     * to estimated fused attitude.
     * When leveling and fused attitudes diverge, fusion is not performed, and instead
     * only gyroscope relative attitude is used for fusion estimation.
     *
     * @throws IllegalStateException if estimator is already running.
     * @throws IllegalArgumentException if value is not between 0.0 and 1.0 (both included).
     */
    override var attitudeOutlierThreshold: Double =
        FusedGeomagneticAttitudeEstimator.DEFAULT_OUTLIER_THRESHOLD
        @Throws(IllegalStateException::class, IllegalArgumentException::class)
        set(value) {
            absoluteAttitudeEstimator.outlierThreshold = value
            field = value
        }

    /**
     * Threshold to determine that leveling has largely diverged and if situation is not
     * reverted soon, attitude will be reset to leveling.
     *
     * @throws IllegalStateException if estimator is already running.
     * @throws IllegalArgumentException if value is not between 0.0 and 1.0 (both included).
     */
    override var attitudeOutlierPanicThreshold: Double =
        FusedGeomagneticAttitudeEstimator.DEFAULT_OUTLIER_PANIC_THRESHOLD
        @Throws(IllegalStateException::class, IllegalArgumentException::class)
        set(value) {
            absoluteAttitudeEstimator.outlierPanicThreshold = value
            field = value
        }

    /**
     * Threshold to determine when fused attitude has largely diverged for a given
     * number of samples and must be reset.
     *
     * @throws IllegalStateException if estimator is already running.
     * @throws IllegalArgumentException if provided value is zero or negative.
     */
    override var attitudePanicCounterThreshold: Int =
        FusedGeomagneticAttitudeEstimator.DEFAULT_PANIC_COUNTER_THRESHOLD
        @Throws(IllegalStateException::class, IllegalArgumentException::class)
        set(value) {
            absoluteAttitudeEstimator.panicCounterThreshold = value
            field = value
        }

    /**
     * Indicates whether this estimator is running or not.
     */
    override var running: Boolean = false
        private set

    /**
     * Gets average time interval between gyroscope samples expressed in seconds.
     */
    override val averageTimeInterval
        get() = absoluteAttitudeEstimator.gyroscopeAverageTimeInterval

    /**
     * True indicates that attitude is leveled and expressed respect to estimator start.
     * False indicates that attitude is absolute.
     */
    override var useLeveledRelativeAttitudeRespectStart = true
        set(value) {
            check(!running)
            field = value
        }

    /**
     * Starts this estimator.
     *
     * @return true if estimator successfully started, false otherwise.
     * @throws IllegalStateException if estimator is already running.
     */
    @Throws(IllegalStateException::class)
    override fun start(): Boolean {
        check(!running)

        reset()
        running = absoluteAttitudeEstimator.start()
        if (!running) {
            stop()
        }

        return running
    }

    /**
     * Stops this estimator.
     */
    override fun stop() {
        absoluteAttitudeEstimator.stop()
        running = false
    }

    /**
     * Estimates device absolute attitude using accelerometer, gyroscope and magnetometer sensors.
     */
    private val absoluteAttitudeEstimator = FusedGeomagneticAttitudeEstimator(
        context,
        this.initialLocation,
        sensorDelay,
        true,
        accelerometerSensorType,
        magnetometerSensorType,
        accelerometerAveragingFilter,
        gyroscopeSensorType,
        worldMagneticModel,
        timestamp,
        useWorldMagneticModel,
        useAccurateLevelingEstimator,
        useAccurateRelativeGyroscopeAttitudeEstimator,
        estimateCoordinateTransformation = true,
        estimateEulerAngles = false,
        attitudeAvailableListener = { estimator, attitude, timestamp, _, _, _, coordinateTransformation ->
            if (!initialize(attitude, coordinateTransformation)) {
                return@FusedGeomagneticAttitudeEstimator
            }

            val timeInterval = estimator.gyroscopeAverageTimeInterval

            // current attitude is expressed in NED coordinates system
            attitude.copyTo(currentAttitude)
            // obtain average attitude between current and previous attitude
            Quaternion.slerp(previousAttitude, currentAttitude, 0.5, averageAttitude)

            // Transform specific force to NED frame
            val fibb = acceleration.valuesAsMatrix
            val fibn = averageAttitude.asInhomogeneousMatrix().multiplyAndReturnNew(fibb)

            val gravity = NEDGravityEstimator.estimateGravityAndReturnNew(currentNedFrame)
            val g = gravity.asMatrix()

            val oldVelocity = previousNedFrame.velocity
            val oldVn = oldVelocity.vn
            val oldVe = oldVelocity.ve
            val oldVd = oldVelocity.vd
            val oldVebn = Matrix(ROWS, 1)
            oldVebn.setElementAtIndex(0, oldVn)
            oldVebn.setElementAtIndex(1, oldVe)
            oldVebn.setElementAtIndex(2, oldVd)

            // From (2.123), determine the angular rate of the ECEF frame with respect
            // the ECI frame, resolved about NED
            val oldLatitude = previousNedFrame.latitude
            val omegaIen = Matrix(ROWS, 1)
            omegaIen.setElementAtIndex(
                0,
                cos(oldLatitude) * NEDInertialNavigator.EARTH_ROTATION_RATE
            )
            omegaIen.setElementAtIndex(
                2,
                -sin(oldLatitude) * NEDInertialNavigator.EARTH_ROTATION_RATE
            )

            // From (5.44), determine the angular rate of the NED frame with respect
            // the ECEF frame, resolved about NED
            val oldRadiiOfCurvature = RadiiOfCurvatureEstimator
                .estimateRadiiOfCurvatureAndReturnNew(oldLatitude)
            val oldRe = oldRadiiOfCurvature.re
            val oldRn = oldRadiiOfCurvature.rn

            val oldHeight = previousNedFrame.height

            val oldOmegaEnN = Matrix(ROWS, 1)
            oldOmegaEnN.setElementAtIndex(0, oldVe / (oldRe + oldHeight))
            oldOmegaEnN.setElementAtIndex(1, -oldVn / (oldRn + oldHeight))
            oldOmegaEnN.setElementAtIndex(2, -oldVe * tan(oldLatitude) / (oldRe + oldHeight))

            val skewOmega2 = Utils.skewMatrix(
                oldOmegaEnN.addAndReturnNew(
                    omegaIen.multiplyByScalarAndReturnNew(2.0)
                )
            )

            // Update velocity
            // From (5.54),
            val vEbn = oldVebn.addAndReturnNew(
                fibn.addAndReturnNew(g).subtractAndReturnNew(
                    skewOmega2.multiplyAndReturnNew(oldVebn)
                )
                    .multiplyByScalarAndReturnNew(timeInterval)
            )

            val vn = vEbn.getElementAtIndex(0)
            val ve = vEbn.getElementAtIndex(1)
            val vd = vEbn.getElementAtIndex(2)

            // Update curvilinear position
            // Update height using (5.56)
            val height: Double = oldHeight - 0.5 * timeInterval * (oldVd + vd)

            // Update latitude using (5.56)
            val latitude: Double = (oldLatitude
                    + 0.5 * timeInterval * (oldVn / (oldRn + oldHeight) + vn / (oldRn + height)))

            // Calculate meridian and transverse radii of curvature
            val radiiOfCurvature = RadiiOfCurvatureEstimator
                .estimateRadiiOfCurvatureAndReturnNew(latitude)
            val re = radiiOfCurvature.re

            // Update longitude using (5.56)
            val oldLongitude = previousNedFrame.longitude
            val longitude: Double = (oldLongitude
                    + 0.5 * timeInterval * (oldVe / ((oldRe + oldHeight) * cos(oldLatitude))
                    + ve / ((re + height) * cos(latitude))))

            currentNedFrame.setPosition(latitude, longitude, height)
            currentNedFrame.setVelocityCoordinates(vn, ve, vd)
            currentNedFrame.coordinateTransformationRotation = currentAttitude

            NEDtoECEFFrameConverter.convertNEDtoECEF(currentNedFrame, currentEcefFrame)


            // compute transformation between initial and current frame and previous and current frame
            if (estimatePoseTransformation) {
                computeTransformation(
                    initialEcefFrame,
                    currentEcefFrame,
                    initialAttitude,
                    currentAttitude,
                    poseTransformation
                )
            }

            // notify
            val initialTransformation =
                if (estimatePoseTransformation) poseTransformation else null

            poseAvailableListener?.onPoseAvailable(
                this,
                currentEcefFrame,
                previousEcefFrame,
                initialEcefFrame,
                timestamp,
                initialTransformation
            )

            // update previous frame
            previousEcefFrame.copyFrom(currentEcefFrame)
            previousNedFrame.copyFrom(currentNedFrame)
            currentAttitude.copyTo(previousAttitude)
        },
        accelerometerMeasurementListener = { ax, ay, az, bx, by, bz, timestamp, accuracy ->

            val currentAx = if (bx != null)
                ax.toDouble() - bx.toDouble()
            else
                ax.toDouble()
            val currentAy = if (by != null)
                ay.toDouble() - by.toDouble()
            else
                ay.toDouble()
            val currentAz = if (bz != null)
                az.toDouble() - bz.toDouble()
            else
                az.toDouble()

            ENUtoNEDTriadConverter.convert(currentAx, currentAy, currentAz, acceleration)

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
        },
        gyroscopeMeasurementListener = { wx, wy, wz, bx, by, bz, timestamp, accuracy ->
            val currentWx = if (bx != null)
                wx.toDouble() - bx.toDouble()
            else
                wx.toDouble()

            val currentWy = if (by != null)
                wy.toDouble() - by.toDouble()
            else
                wy.toDouble()

            val currentWz = if (bz != null)
                wz.toDouble() - bz.toDouble()
            else
                wz.toDouble()

            ENUtoNEDTriadConverter.convert(currentWx, currentWy, currentWz, angularSpeed)

            gyroscopeMeasurementListener?.onMeasurement(
                wx,
                wy,
                wz,
                bx,
                by,
                bz,
                timestamp,
                accuracy
            )
        },
        gravityEstimationListener = { estimator, fx, fy, fz, timestamp ->
            gravity.setValueCoordinates(fx, fy, fz)

            gravityEstimationListener?.onEstimation(estimator, fx, fy, fz, timestamp)
        },
        magnetometerMeasurementListener = magnetometerMeasurementListener
    )

    /**
     * Resets internal parameters.
     */
    private fun reset() {
        initializedFrame = false
    }

    /**
     * Estimates initial ECEF and ND frames when the first absolute attitude is obtained after
     * starting this estimator.
     *
     * @param attitude first obtained attitude.
     * @param coordinateTransformation first obtained coordinate transformation containing initial
     * absolute attitude.
     * @return true if this estimator was already initialized, false if it wasn't.
     */
    private fun initialize(
        attitude: Quaternion,
        coordinateTransformation: CoordinateTransformation?
    ): Boolean {
        val result = initializedFrame
        if (!initializedFrame && coordinateTransformation != null) {
            attitude.copyTo(initialAttitude)

            initialNedFrame.coordinateTransformation = coordinateTransformation
            initialNedFrame.position = initialLocation.toNEDPosition()
            initialNedFrame.velocity = initialVelocity

            NEDtoECEFFrameConverter.convertNEDtoECEF(initialNedFrame, initialEcefFrame)

            initialEcefFrame.copyTo(previousEcefFrame)
            initialNedFrame.copyTo(previousNedFrame)

            initialAttitude.fromQuaternion(attitude)
            previousAttitude.fromQuaternion(attitude)

            initializedFrame = true
        }
        return result
    }

    /**
     * Computes 3D metric transformation between procided ECEF frames.
     *
     * @param initialEcefFrame starting frame.
     * @param currentEcefFrame end frame.
     * @param initialAttitude start body attitude respect to local coordinates.
     * @param currentAttitude end body attitude respect to local coordinates.
     * @param result instance where result will be stored.
     */
    private fun computeTransformation(
        initialEcefFrame: ECEFFrame,
        currentEcefFrame: ECEFFrame,
        initialAttitude: Quaternion,
        currentAttitude: Quaternion,
        result: EuclideanTransformation3D,
    ) {
        // set rotation
        if (useLeveledRelativeAttitudeRespectStart) {
            initialAttitude.toEulerAngles(eulerAngles)
            val initYaw = eulerAngles[2]

            currentAttitude.toEulerAngles(eulerAngles)
            val currentRoll = eulerAngles[0]
            val currentPitch = eulerAngles[1]
            val currentYaw = eulerAngles[2]

            val deltaYaw = currentYaw - initYaw
            transformationRotation.setFromEulerAngles(currentRoll, currentPitch, deltaYaw)

            // convert to ENU coordinates
            Quaternion.product(transformationRotation, conversionRotation, transformationRotation)
        } else {
            // convert to ENU coordinates
            Quaternion.product(currentAttitude, conversionRotation, transformationRotation)
        }

        result.rotation.fromRotation(transformationRotation)

        // current attitude contains body to local frame attitude
        // ecef frame contains euclidean position coordinates rotated by body rotation respect ECEF.
        // we need to transform position coordinates respect ECEF to euclidean coordinates respect
        // local position
        ecefDiffPosition.setInhomogeneousCoordinates(
            currentEcefFrame.x - initialEcefFrame.x,
            currentEcefFrame.y - initialEcefFrame.y,
            currentEcefFrame.z - initialEcefFrame.z
        )

        initialEcefFrame.coordinateTransformation.asRotation(startEcefRotation)
        startEcefRotation.inverseRotation(inverseEcefRotation)

        inverseEcefRotation.rotate(ecefDiffPosition, localDiffPosition)

        // convert from NED to ENU
        conversionRotation.rotate(localDiffPosition, localDiffPosition)

        result.setTranslation(localDiffPosition)
    }

    private companion object {

        /**
         * Number of rows in a matrix representation of triads.
         */
        const val ROWS = 3
    }

    /**
     * Interface to notify when a new pose is available.
     */
    fun interface OnPoseAvailableListener :
        AbsolutePoseEstimator.OnPoseAvailableListener<LocalPoseEstimator>
}