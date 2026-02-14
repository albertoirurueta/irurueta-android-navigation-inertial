package com.irurueta.android.navigation.inertial.old.processors.attitude

import android.location.Location
import com.irurueta.algebra.ArrayUtils
import com.irurueta.algebra.Matrix
import com.irurueta.algebra.Utils
import com.irurueta.geometry.Quaternion
import com.irurueta.navigation.inertial.estimators.NEDGravityEstimator
import kotlin.math.atan2

/**
 * Estimates leveling of device (roll and pitch angle) by using estimated gravity vector.
 * This processor does not estimate attitude yaw angle, as either a magnetometer or gyroscope would
 * be needed.
 * This estimator is more accurate than [com.irurueta.android.navigation.inertial.old.estimators.attitude.LevelingEstimator] since it takes into account device
 * location (which requires location permission), and at the expense of higher CPU load.
 *
 * @property location device location.
 * @property processorListener listener to notify new leveled attitudes.
 */
class AccurateLevelingProcessor(
    var location: Location,
    processorListener: OnProcessedListener? = null
) : BaseLevelingProcessor(processorListener) {

    /**
     * Processes provided gravity components estimated using a [BaseGravityProcessor].
     */
    override fun process(gx: Double, gy: Double, gz: Double) {
        computeLevelingAttitude(
            Math.toRadians(location.latitude),
            location.altitude,
            gx,
            gy,
            gz,
            attitude
        )

        this.gx = gx
        this.gy = gy
        this.gz = gz

        processorListener?.onProcessed(this, attitude)
    }

    companion object {
        /**
         * Computes an attitude containing device leveling (roll and pitch) based on current
         * location (latitude and height) and sensed specific force containing gravity components.
         *
         * @param latitude current device latitude expressed in radians.
         * @param height current device height above mean sea level expressed in meters (m).
         * @param gx x-coordinate of sensed specific force containing gravity component.
         * @param gy y-coordinate of sensed specific force containing gravity component.
         * @param gz z-coordinate of sensed specific force containing gravity component.
         * @param result quaternion where attitude containing estimated leveling (roll and pitch
         * angles) will be stored.
         */
        fun computeLevelingAttitude(
            latitude: Double,
            height: Double,
            gx: Double,
            gy: Double,
            gz: Double,
            result: Quaternion
        ) {
            // get normalized vector from measured specific force, which
            // mainly contains sensed gravity in the local navigation frame
            // when device is static (Coriolis force is neglected in this
            // implementation).

            // obtain normalized specific force in local navigation coordinates
            val normF = doubleArrayOf(gx, gy, gz)
            ArrayUtils.normalize(normF)

            // obtain gravity in NED coordinates (locally equivalent to
            // the one in local navigation frame).
            // Because Earth is not fully spherical, normalized vector won't
            // be (0, 0, 1), because there will always be a small north
            // gravity component.
            val nedGravity = NEDGravityEstimator.estimateGravityAndReturnNew(latitude, height)

            val normG = nedGravity.asArray()

            // ensure that down coordinate points towards Earth center, just
            // like sensed specific force

            // ensure that down coordinate points towards Earth center, just
            // like sensed specific force
            ArrayUtils.multiplyByScalar(normG, -1.0, normG)

            ArrayUtils.normalize(normG)

            // compute angle between both normalized vectors using dot product
            // cos(alpha) = normF' * normG
            val cosAlpha = ArrayUtils.dotProduct(normF, normG)

            // compute vector perpendicular to both normF and normG which will
            // be the rotation axis
            val skew = Utils.skewMatrix(normG)
            val tmp1 = Matrix.newFromArray(normF)
            val tmp2 = skew.multiplyAndReturnNew(tmp1)

            val sinAlpha = Utils.normF(tmp2)

            val axis = tmp2.toArray()
            ArrayUtils.normalize(axis)

            val alpha = atan2(sinAlpha, cosAlpha)

            result.setFromAxisAndRotation(axis, -alpha)
            result.normalize()
        }
    }
}