package com.irurueta.android.navigation.inertial.processors.attitude.kalman

import android.hardware.SensorManager
import com.irurueta.algebra.AlgebraException
import com.irurueta.algebra.Matrix
import com.irurueta.algebra.Utils
import com.irurueta.navigation.inertial.calibration.AccelerationTriad
import kotlin.math.abs

/**
 * Estimates external acceleration covariance matrix using Sabatini's method.
 *
 * @property accelerometerNoiseStandardDeviation standard deviation for accelerometer noise
 * expressed in m/s^2.
 */
class SabatiniExternalAccelerationCovarianceMatrixEstimator(
    accelerometerNoiseStandardDeviation: Double = DEFAULT_ACCELEROMETER_NOISE_STANDARD_DEVIATION
) : ExternalAccelerationCovarianceMatrixEstimator(accelerometerNoiseStandardDeviation) {
    // za
    /**
     * Body accelerometer measurement in local NED coordinates obtained from accelerometer sensor.
     *
     * @throws IllegalArgumentException if provided matrix is not 3x1.
     */
    var bodyAccelerationMatrix = Matrix(AccelerationTriad.COMPONENTS, 1)
        @Throws(IllegalArgumentException::class)
        set(value) {
            require(value.rows == AccelerationTriad.COMPONENTS)
            require(value.columns == 1)
            field.copyFrom(value)
        }

    /**
     * Gets or sets gravity expressed in m/s^2.
     *
     * @throws IllegalArgumentException if provided value is negative.
     */
    var gravity: Double = EARTH_GRAVITY
        @Throws(IllegalArgumentException::class)
        set(value) {
            require(value >= 0.0)
            field = value
        }

    /**
     * Threshold to determine whether external acceleration is detected or not.
     *
     * @throws IllegalArgumentException if provided value is negative.
     */
    var epsilon: Double = DEFAULT_EPSILON
        @Throws(IllegalArgumentException::class)
        set(value) {
            require(value >= 0.0)
            field = value
        }

    /**
     * Scale factor to be applied to external acceleration covariance matrix when external
     * acceleration is detected.
     */
    var s: Double = DEFAULT_S

    /**
     * Computes estimation of external acceleration covariance matrix.
     *
     * @throws com.irurueta.algebra.AlgebraException if there are numerical errors or instabilities.
     */
    @Throws(AlgebraException::class)
    override fun estimate() {
        computePartialAccelerometerResidualCovarianceMatrix()

        if (abs(Utils.normF(bodyAccelerationMatrix) - gravity) < epsilon) {
            qab.initialize(0.0)
        } else {
            Matrix.identity(qab)
            qab.multiplyByScalar(s)
        }
    }

    companion object {
        /**
         * Average gravity at Earth's sea level expressed in m/s^2.
         */
        const val EARTH_GRAVITY = SensorManager.GRAVITY_EARTH.toDouble()

        /**
         * Default error respect to gravity to consider that external acceleration exists expressed
         * in m/s^2.
         */
        const val DEFAULT_EPSILON: Double = 0.25

        /**
         * Default scale factor to be applied to external acceleration covariance matrix when
         * external acceleration is detected.
         */
        const val DEFAULT_S: Double = 10.0
    }
}