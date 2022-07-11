package com.irurueta.android.navigation.inertial.estimators

import com.irurueta.algebra.Matrix
import com.irurueta.algebra.Utils
import com.irurueta.geometry.Quaternion
import com.irurueta.navigation.inertial.calibration.AngularSpeedTriad

/**
 * Integrates steps of attitude rotations using a Runge-Kutta integration algorithm.
 */
object AttitudeIntegrator {

    /**
     * Performs a RK4 Runge-Kutta integration step.
     *
     * @param initialAttitude initial attitude.
     * @param initialWx initial x-coordinate rotation velocity at time t0 expressed in radians per
     * second (rad/s).
     * @param initialWy initial y-coordinate rotation velocity at time t0 expressed in radians per
     * second (rad/s).
     * @param initialWz initial z-coordinate rotation velocity at time t0 expressed in radians per
     * second (rad/s).
     * @param currentWx final x-coordinate rotation velocity at time t1 expressed in radians per
     * second (rad/s).
     * @param currentWy final y-coordinate rotation velocity at time t1 expressed in radians per
     * second (rad/s).
     * @param currentWz final z-coordinate rotation velocity at time t1 expressed in radians per
     * second (rad/s).
     * @param dt time step expressed in seconds (t1 - t0).
     */
    fun integrationStep(
        initialAttitude: Quaternion,
        initialWx: Double,
        initialWy: Double,
        initialWz: Double,
        currentWx: Double,
        currentWy: Double,
        currentWz: Double,
        dt: Double,
        result: Quaternion
    ) {

        val quat = Matrix(Quaternion.N_PARAMS, 1)
        initialAttitude.values(quat.buffer)

        val omega0 = Matrix(AngularSpeedTriad.COMPONENTS, 1)
        omega0.setElementAtIndex(0, initialWx)
        omega0.setElementAtIndex(1, initialWy)
        omega0.setElementAtIndex(2, initialWz)

        val omega1 = Matrix(AngularSpeedTriad.COMPONENTS, 1)
        omega1.setElementAtIndex(0, currentWx)
        omega1.setElementAtIndex(1, currentWy)
        omega1.setElementAtIndex(2, currentWz)

        val quatResult = Matrix(Quaternion.N_PARAMS, 1)
        quatIntegrationStepRK4(quat, omega0, omega1, dt, quatResult)
        result.values = quatResult.buffer
    }

    /**
     * Performs a RK4 Runge-Kutta integration step.
     *
     * @param quat the input 4D vector representing the initial rotation as quaternion values.
     * Must be 4x1.
     * @param omega0 initial rotation velocity time t0 expressed in radians per second (rad/s).
     * Must be 3x1.
     * @param omega1 final rotation velocity time t1 expressed in radians per scond (rad/s). Must be
     * 3x1.
     * @param dt = time step expressed in seconds (t1 - t0).
     * @param result resulting final rotation. Must be 4x1.
     */
    private fun quatIntegrationStepRK4(
        quat: Matrix,
        omega0: Matrix,
        omega1: Matrix,
        dt: Double,
        result: Matrix
    ) {
        val omega01 = omega0.addAndReturnNew(omega1)
        omega01.multiplyByScalar(0.5)

        val k1 = Matrix(Quaternion.N_PARAMS, 1)
        val k2 = Matrix(Quaternion.N_PARAMS, 1)
        val k3 = Matrix(Quaternion.N_PARAMS, 1)
        val k4 = Matrix(Quaternion.N_PARAMS, 1)
        val tmpQ = Matrix(Quaternion.N_PARAMS, 1)

        val omegaSkew = Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS)

        // First Runge-Kutta coefficient
        computeOmegaSkew(omega0, omegaSkew)
        omegaSkew.multiply(quat, k1)
        k1.multiplyByScalar(0.5)

        // Second Runge-Kutta coefficient
        tmpQ.copyFrom(k1)
        tmpQ.multiplyByScalar(0.5 * dt)
        tmpQ.add(quat)
        computeOmegaSkew(omega01, omegaSkew)
        omegaSkew.multiply(tmpQ, k2)
        k2.multiplyByScalar(0.5)

        // Third Runge-Kutta coefficient (same omega skew as second coeff.)
        tmpQ.copyFrom(k2)
        tmpQ.multiplyByScalar(0.5 * dt)
        tmpQ.add(quat)
        omegaSkew.multiply(tmpQ, k3)
        k3.multiplyByScalar(0.5)

        // Forth Runge-Kutta coefficient
        tmpQ.copyFrom(k3)
        tmpQ.multiplyByScalar(dt)
        tmpQ.add(quat)
        computeOmegaSkew(omega1, omegaSkew)
        omegaSkew.multiply(tmpQ, k4)
        k4.multiplyByScalar(0.5)

        val mult1 = 1.0 / 6.0
        val mult2 = 1.0 / 3.0

        // result = quat + dt * (mult1 * k1 + mult2 * k2 + mult2 * k3 + mult1 * k4)
        k1.multiplyByScalar(mult1)
        k2.multiplyByScalar(mult2)
        k3.multiplyByScalar(mult2)
        k4.multiplyByScalar(mult1)

        result.copyFrom(k1)
        result.add(k2)
        result.add(k3)
        result.add(k4)
        result.multiplyByScalar(dt)
        result.add(quat)

        normalizeQuaternion(result)
    }

    /**
     * Gets skew symmetric matrix representation of angular speed.
     *
     * @param omega matrix containing angular speed components. Must be 3x1.
     * @param result computed skew symmetric matrix representation. Must be 4x4.
     */
    private fun computeOmegaSkew(omega: Matrix, result: Matrix) {
        val omega0 = omega.getElementAtIndex(0)
        val omega1 = omega.getElementAtIndex(1)
        val omega2 = omega.getElementAtIndex(2)

        result.setElementAtIndex(0, 0.0)
        result.setElementAtIndex(1, omega0)
        result.setElementAtIndex(2, omega1)
        result.setElementAtIndex(3, omega2)

        result.setElementAtIndex(4, -omega0)
        result.setElementAtIndex(5, 0.0)
        result.setElementAtIndex(6, -omega2)
        result.setElementAtIndex(7, omega1)

        result.setElementAtIndex(8, -omega1)
        result.setElementAtIndex(9, omega2)
        result.setElementAtIndex(10, 0.0)
        result.setElementAtIndex(11, -omega0)

        result.setElementAtIndex(12, -omega2)
        result.setElementAtIndex(13, -omega1)
        result.setElementAtIndex(14, omega0)
        result.setElementAtIndex(15, 0.0)
    }

    /**
     * Normalize an input matrix corresponding to a quaternion into a unit
     * vector.
     *
     * @param quaternion matrix containing quaternion values to be normalized.
     */
    private fun normalizeQuaternion(quaternion: Matrix) {
        val norm = Utils.normF(quaternion)
        quaternion.multiplyByScalar(1.0 / norm)
    }
}