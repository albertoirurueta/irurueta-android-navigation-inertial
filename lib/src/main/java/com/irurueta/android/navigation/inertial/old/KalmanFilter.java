package com.irurueta.android.navigation.inertial.old;

import com.irurueta.algebra.AlgebraException;
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.Utils;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.numerical.signal.processing.SignalProcessingException;

import java.io.Serializable;

/**
 * Implementation of a Kalman filter.
 * This class contains the state of a Kalman kilter.
 * The state of a Kalman filter is updated by
 * <code>predict</code> and <code>correct</code> functions.
 * <p>
 * The source code, notation and formulae below are borrowed from the JKalman
 * tutorial <a href="http://www.cs.unc.edu/~welch/kalman/">[Welch95]</a>:
 * <pre>
 * {@code
 * x<sub>k</sub>=A*x<sub>k-1</sub>+B*u<sub>k</sub>+w<sub>k</sub>
 * z<sub>k</sub>=Hx<sub>k</sub>+v<sub>k</sub>,
 * }
 * </pre>
 * <p>where:
 * <pre>
 * {@code x<sub>k</sub> (x<sub>k-1</sub>)} - state of the system at the moment k (k-1)
 * {@code z<sub>k</sub>} - measurement of the system state at the moment k
 * {@code u<sub>k</sub>} - external control applied at the moment k
 * {@code w<sub>k</sub>} and {@code v<sub>k</sub>} are normally-distributed process and
 * measurement noise, respectively:
 * p(w) ~ N(0,Q)
 * p(v) ~ N(0,R),
 * that is,
 * Q - process noise covariance matrix, constant or variable,
 * R - measurement noise covariance matrix, constant or variable
 * </pre>
 * <p>
 * In case of standard Kalman filter, all the matrices: A, B, H, Q and R
 * are initialized once after Kalman structure is allocated via constructor.
 * However, the same structure and the same functions may be used to simulate
 * extended Kalman filter by linearizing extended Kalman filter equation in the
 * current system state neighborhood, in this case A, B, H (and, probably,
 * Q and R) should be updated on every step.
 */
public class KalmanFilter implements Serializable {

    /**
     * Independent process noise variance assumed when no process noise
     * covariance matrix is provided.
     * The lower the process variance the smoother the estimated state will
     * typically be.
     */
    public static final double DEFAULT_PROCESS_NOISE_VARIANCE = 1e-6;

    /**
     * Independent measurement noise variance assumed when no measurement noise
     * covariance matrix is provided.
     */
    public static final double DEFAULT_MEASUREMENT_NOISE_VARIANCE = 1e-1;

    /**
     * Number of measurement vector dimensions (measure parameters).
     */
    private int mp;

    /**
     * Number of state vector dimensions (dynamic parameters).
     */
    private final int dp;

    /**
     * Number of control vector dimensions (control parameters).
     */
    private final int cp;

    /**
     * Predicted state (x'(k)): x(k)=A*x(k-1)+B*u(k)
     */
    private Matrix statePre;

    /**
     * Corrected state (x(k)): x(k)=x'(k)+K(k)*(z(k)-H*x'(k))
     */
    private Matrix statePost;

    /**
     * State transition matrix (A).
     */
    private Matrix transitionMatrix;

    /**
     * Control matrix (B) (it is not used if there is no control).
     */
    private Matrix controlMatrix;

    /**
     * Measurement matrix (H).
     */
    private Matrix measurementMatrix;

    /**
     * Process noise covariance matrix (Q).
     */
    private Matrix processNoiseCov;

    /**
     * Measurement noise covariance matrix (R).
     */
    private Matrix measurementNoiseCov;

    /**
     * Priori error estimate covariance matrix (P'(k)): P'(k)=A*P(k-1)*At + Q)
     */
    private Matrix errorCovPre;

    /**
     * Kalman gain matrix (K(k)): K(k)=P'(k)*Ht*inv(H*P'(k)*Ht+R)
     */
    private Matrix gain;

    /**
     * Posteriori error estimate covariance matrix (P(k)): P(k)=(I-K(k)*H)*P'(k)
     */
    private Matrix errorCovPost;

    // temporary matrices to be reused to avoid unnecessary re-allocations

    /**
     * Temporary matrix 1.
     */
    private final Matrix temp1;

    /**
     * Temporary matrix 2.
     */
    private Matrix temp2;

    /**
     * Temporary matrix 3.
     */
    private Matrix temp3;

    /**
     * Temporary matrix 4.
     */
    private Matrix temp4;

    /**
     * Temporary matrix 5.
     */
    private Matrix temp5;

    /**
     * Temporary matrix 6.
     */
    private Matrix temp6;

    /**
     * Temporary matrix 7.
     */
    private final Matrix temp7;

    /**
     * Temporary matrix 8.
     */
    private Matrix temp8;

    /**
     * Allocates a Kalman filter and all its matrices and initializes them.
     *
     * @param dynamParams   number of dynamic parameters (state vector dimensions).
     * @param measureParams number of measurement parameters (measurement vector
     *                      dimensions).
     * @param controlParams number of control parameters (control vector.
     *                      dimensions). If zero, no control parameters are used. If less than zero,
     *                      it is assumed that this is equal to the number of dynamic parameters.
     * @throws IllegalArgumentException  if either the number of dynamic or
     *                                   measurement parameters is zero or negative.
     * @throws SignalProcessingException if something else fails.
     */
    public KalmanFilter(final int dynamParams, final int measureParams, int controlParams)
            throws SignalProcessingException {

        if (dynamParams <= 0 || measureParams <= 0) {
            throw new IllegalArgumentException(
                    "Kalman filter: Illegal dimensions");
        }

        if (controlParams < 0) {
            controlParams = dynamParams;
        }

        // init
        dp = dynamParams;
        mp = measureParams;
        cp = controlParams;

        try {
            statePre = new Matrix(dp, 1);

            // following variables must be initialized properly in advance
            statePost = new Matrix(dp, 1);
            transitionMatrix = Matrix.identity(dp, dp);

            processNoiseCov = Matrix.identity(dp, dp);
            processNoiseCov.multiplyByScalar(DEFAULT_PROCESS_NOISE_VARIANCE);

            measurementMatrix = Matrix.identity(mp, dp);
            measurementNoiseCov = Matrix.identity(mp, mp);
            measurementNoiseCov.multiplyByScalar(
                    DEFAULT_MEASUREMENT_NOISE_VARIANCE);

            errorCovPre = new Matrix(dp, dp);
            errorCovPost = Matrix.identity(dp, dp);

            gain = new Matrix(dp, mp);

            if (cp > 0) {
                controlMatrix = new Matrix(dp, cp);
            } else {
                // no control parameters
                controlMatrix = null;
            }

            temp1 = new Matrix(dp, dp);
            temp2 = new Matrix(mp, dp);
            temp3 = new Matrix(mp, mp);
            temp4 = new Matrix(mp, dp);
            temp5 = new Matrix(mp, 1);

            if (cp > 0) {
                temp6 = new Matrix(dp, 1);
            }

            temp7 = new Matrix(dp, dp);
            temp8 = new Matrix(dp, mp);
        } catch (final AlgebraException ex) {
            throw new SignalProcessingException(ex);
        }
    }

    /**
     * Constructor in case of no control parameters.
     *
     * @param dynamParams   number of dynamic parameters (state vector dimensions).
     * @param measureParams number of measurement parameters (measurement vector
     *                      dimensions).
     * @throws IllegalArgumentException  if either the number of dynamic or
     *                                   measurement parameters is zero or negative.
     * @throws SignalProcessingException if something else fails.
     */
    public KalmanFilter(final int dynamParams, final int measureParams)
            throws SignalProcessingException {
        this(dynamParams, measureParams, 0);
    }

    /**
     * Estimates subsequent model state without control parameters.
     *
     * @return estimated state.
     * @throws SignalProcessingException if something fails.
     * @see #predict(Matrix)
     */
    public Matrix predict() throws SignalProcessingException {
        return predict(null);
    }

    /**
     * Estimates subsequent model state.
     * The function estimates the subsequent stochastic model state by its
     * current state and stores it at <code>statePre</code>:
     * <pre>
     * {@code
     * x'<sub>k</sub>=A*x<sub>k</sub>+B*u<sub>k</sub>
     * P'<sub>k</sub>=A*P<sub>k-1</sub>*A<sup>T</sup> + Q,
     * where
     * x'<sub>k</sub> is predicted state (statePre),
     * x<sub>k-1</sub> is corrected state on the previous step (statePost)
     *     (should be initialized somehow in the beginning, zero vector by
     * default),
     * u<sub>k</sub> is external control (<code>control</code> parameter),
     * P'<sub>k</sub> is prior error covariance matrix (error_cov_pre)
     * P<sub>k-1</sub> is posteriori error covariance matrix on the previous
     * step (error_cov_post)
     *     (should be initialized somehow in the beginning, identity matrix by
     * default),
     * }
     * </pre>
     *
     * @param control control vector (u<sub>k</sub>), should be null if there is
     *                no external control (<code>controlParams</code>=0). If provided and
     *                filter uses control parameters, it must be a 1 column matrix having
     *                cp rows (where cp = number of control parameters), otherwise a
     *                SignalProcessingException will be raised.
     * @return estimated state as a 1 column matrix having dp rows (where dp =
     * number of dynamic parameters).
     * @throws SignalProcessingException if something fails.
     */
    public Matrix predict(final Matrix control) throws SignalProcessingException {
        try {
            // (1) Project the state ahead
            // update the state: x'(k) = A*x(k)
            transitionMatrix.multiply(statePost, statePre);
            if (control != null && cp > 0) {
                // x'(k) = x'(k) + B*u(k)
                controlMatrix.multiply(control, temp6);
                statePre.add(temp6);
            }

            // (2) Project the error covariance ahead
            // update error covariance matrices: temp1 = A * P(k)
            transitionMatrix.multiply(errorCovPost, temp1);
            // P'(k) = temp1 * At + Q
            transitionMatrix.transpose(temp7);
            temp1.multiply(temp7);
            temp1.add(processNoiseCov);
            errorCovPre.copyFrom(temp1);

            return statePre;
        } catch (final AlgebraException e) {
            throw new SignalProcessingException(e);
        }
    }

    /**
     * Adjusts model state.
     * This method adjusts stochastic model state on the basis of the given
     * measurement of the model state:
     * <pre>
     * {@code
     * K<sub>k</sub>=P'<sub>k</sub>*H<sup>T</sup>*(H*P'<sub>k</sub>*H<sup>T</sup>+R)<sup>-1</sup>
     * x<sub>k</sub>=x'<sub>k</sub>+K<sub>k</sub>*(z<sub>k</sub>-H*x'<sub>k</sub>)
     * P<sub>k</sub>=(I-K<sub>k</sub>*H)*P'<sub>k</sub>
     * where
     * z<sub>k</sub> - given measurement (<code>measurement</code> parameter)
     * K<sub>k</sub> - Kalman "gain" matrix.
     * }
     * </pre>
     * <p>
     * The function stores adjusted state at <code>statePost</code> and returns
     * it on output.
     *
     * @param measurement matrix containing the measurement vector. Matrix must
     *                    have 1 column and mp rows (mp = measurement parameters).
     * @return adjusted model state.
     * @throws SignalProcessingException if something fails.
     */
    public Matrix correct(final Matrix measurement) throws SignalProcessingException {
        try {
            // (1) compute the Kalman gain
            // temp2 = H*P'(k)
            measurementMatrix.multiply(errorCovPre, temp2);

            // temp3 = temp2*Ht + R
            measurementMatrix.transpose(temp8);
            temp2.multiply(temp8, temp3);
            temp3.add(measurementNoiseCov);

            // temp4 = inv(temp3)*temp2 = Kt(k)
            // which is also equivalent to:
            // temp4 = temp3.svd().getU().times(temp2)

            // However, we can also solve a liner system of equations more efficiently as since:
            // temp2 = temp3 * Kt(k) = temp3 * temp4
            Utils.solve(temp3, temp2, temp4);

            // K(k)
            temp4.transpose();
            gain.copyFrom(temp4);

            // (2) Update estimate with measurement z(k)
            //temp5 = z(k) - H*x'(k)
            measurementMatrix.multiply(statePre, temp5);
            temp5.multiplyByScalar(-1.0);
            temp5.add(measurement);

            // x(k) = x'(k) + K(k)*temp5
            gain.multiply(temp5, statePost);
            statePost.add(statePre);

            // (3) Update the error covariance
            // P(x) = P'(k) - K(k)*temp2
            gain.multiply(temp2, errorCovPost);
            errorCovPost.multiplyByScalar(-1.0);
            errorCovPost.add(errorCovPre);

            return statePost;
        } catch (final AlgebraException e) {
            throw new SignalProcessingException(e);
        }
    }

    /**
     * Obtains the number of measurement vector dimensions (measure parameters).
     *
     * @return number of measurement vector dimensions (measure parameters)
     */
    public int getMeasureParameters() {
        return mp;
    }

    /**
     * Sets the number of measurement vector dimensions (measure parameters).
     *
     * @param measureParameters number of measurement vector dimensions (measure
     *                          parameters).
     *                          NOTE: when resetting number of measure parameters, the measurement noise
     *                          covariance matrix and the measurement matrix get reset to their default
     *                          values having the required new size. Please, make sure those matrices
     *                          are reset to their proper values after calling this method.
     * @throws IllegalArgumentException  if provided value is zero or negative.
     * @throws SignalProcessingException if something else fails
     */
    public void setMeasureParameters(final int measureParameters)
            throws SignalProcessingException {
        if (measureParameters <= 0) {
            throw new IllegalArgumentException("");
        }
        mp = measureParameters;

        try {
            measurementMatrix = Matrix.identity(mp, dp);
            measurementNoiseCov = Matrix.identity(mp, mp);
            measurementNoiseCov.multiplyByScalar(
                    DEFAULT_MEASUREMENT_NOISE_VARIANCE);

            gain = new Matrix(dp, mp);

            temp2 = new Matrix(mp, dp);
            temp3 = new Matrix(mp, mp);
            temp4 = new Matrix(mp, dp);
            temp5 = new Matrix(mp, 1);

            temp8 = new Matrix(dp, mp);
        } catch (final WrongSizeException e) {
            throw new SignalProcessingException(e);
        }
    }

    /**
     * Obtains the number of state vector dimensions (dynamic parameters).
     *
     * @return number of state vector dimensions (dynamic parameters)
     */
    public int getDynamicParameters() {
        return dp;
    }

    /**
     * Obtains the number of control vector dimensions (control parameters).
     *
     * @return number of control vector dimensions (control parameters)
     */
    public int getControlParameters() {
        return cp;
    }

    /**
     * Obtains predicted state (x'(k)): x(k)=A*x(k-1)+B*u(k).
     * It is a column matrix having 1 column and dp rows, where dp is the
     * number of dynamic parameters
     *
     * @return predicted state
     */
    public Matrix getStatePre() {
        return statePre;
    }

    /**
     * Sets predicted state (x'(k)): x(k)=A*x(k-1)+B*u(k).
     * Provided matrix must have 1 column and dp rows, where dp is the number
     * of dynamic parameters set for this Kalman filter instance.
     * This setter method can be used for initial setup purposes.
     *
     * @param statePre new predicted state.
     * @throws IllegalArgumentException if provided matrix does not have 1
     *                                  column and dp rows
     */
    public void setStatePre(final Matrix statePre) {
        if (statePre.getColumns() != 1 || statePre.getRows() != dp) {
            throw new IllegalArgumentException();
        }
        this.statePre = statePre;
    }

    /**
     * Obtains corrected state (x(k)): x(k)=x'(k)+K(k)*(z(k)-H*x'(k)).
     * It is a column matrix having 1 column and dp rows, where dp is the
     * number of dynamic parameters
     *
     * @return corrected state
     */
    public Matrix getStatePost() {
        return statePost;
    }

    /**
     * Sets corrected state (x(k)): x(k)=x'(k)+K(k)*(z(k)-H*x'(k)).
     * Provided matrix must have 1 column and dp rows, where dp is the number
     * of dynamic parameters set for this Kalman filter instance.
     * This setter method can be used for initial setup purposes.
     *
     * @param statePost new corrected state
     * @throws IllegalArgumentException if provided matrix does not have 1
     *                                  column and dp rows
     */
    public void setStatePost(final Matrix statePost) {
        if (statePost.getColumns() != 1 || statePost.getRows() != dp) {
            throw new IllegalArgumentException();
        }
        this.statePost = statePost;
    }

    /**
     * Obtains the state transition matrix (A).
     * It is a square matrix having dp rows and columns, where dp is equal to
     * the number of dynamic parameters.
     * This matrix defines how the system transitions to a new state for a given
     * previous state. It is used for prediction purposes
     *
     * @return state transition matrix
     */
    public Matrix getTransitionMatrix() {
        return transitionMatrix;
    }

    /**
     * Sets the state transition matrix (A).
     * It must be a square matrix having dp rows and columns, where dp is equal
     * to the number of dynamic parameters set for this instance.
     * This matrix defines how the system transitions to a new state for a given
     * previous state. It is used for prediction purposes.
     * This setter method can be used for initial setup purposes.
     *
     * @param transitionMatrix new state transition matrix
     * @throws IllegalArgumentException if provided matrix does not have dp rows
     *                                  and columns
     */
    public void setTransitionMatrix(final Matrix transitionMatrix) {
        if (transitionMatrix.getRows() != dp ||
                transitionMatrix.getColumns() != dp) {
            throw new IllegalArgumentException();
        }
        this.transitionMatrix = transitionMatrix;
    }

    /**
     * Obtains the control matrix (B) (it is not used if there is no control).
     * It's a matrix having dp rows and cp columns, where dp is the number of
     * dynamic parameters and cp is the number of control parameters.
     *
     * @return control matrix
     */
    public Matrix getControlMatrix() {
        return controlMatrix;
    }

    /**
     * Sets the control matrix (B) (it is not used if there is no control).
     * Provided matrix must have dp rows and cp columns, where dp is the number
     * of dynamic parameters and cp is the number of control parameters set for
     * this Kalman filter instance.
     * This setter method can be used for initial setup purposes.
     *
     * @param controlMatrix new control matrix to be set, or null if no control
     *                      parameters are set
     * @throws IllegalArgumentException if provided matrix does not have dp
     *                                  rows and cp columns
     */
    public void setControlMatrix(final Matrix controlMatrix) {
        if (cp > 0) {
            if (controlMatrix == null || (controlMatrix.getRows() != dp ||
                    controlMatrix.getColumns() != cp)) {
                throw new IllegalArgumentException();
            }
        } else {
            // control matrix cannot be set
            throw new IllegalArgumentException();
        }
        this.controlMatrix = controlMatrix;
    }

    /**
     * Obtains measurement matrix (H).
     * It's a matrix having mp rows and dp columns, where mp is the number
     * of measurement parameters and dp is the number of dynamic parameters of
     * the system state.
     * This matrix relates obtained measures to the actual system state when a
     * given model is known in advance. If no model is known and measures
     * directly indicate the system state, then this matrix must be the
     * identity.
     *
     * @return measurement matrix
     */
    public Matrix getMeasurementMatrix() {
        return measurementMatrix;
    }

    /**
     * Sets measurement matrix (H).
     * Provided matrix must have mp rows and dp columns, where mp is the number
     * of measurement parameters and dp is the number of dynamic parameters of
     * the system state.
     * This matrix relates obtained measures to the actual system state when a
     * given model is known in advance. If no model is known and measures
     * directly indicate the system state, then this matrix must be the
     * identity.
     * This setter method can be used for initial setup purposes.
     *
     * @param measurementMatrix measurement matrix
     * @throws IllegalArgumentException if provided matrix does not have mp rows
     *                                  and dp columns.
     */
    public void setMeasurementMatrix(final Matrix measurementMatrix) {
        if (measurementMatrix.getRows() != mp ||
                measurementMatrix.getColumns() != dp) {
            throw new IllegalArgumentException();
        }
        this.measurementMatrix = measurementMatrix;
    }

    /**
     * Obtains the process noise covariance matrix (Q).
     * This is a covariance matrix indicating the correlations of the amount of
     * error in the system state.
     * It is a square symmetric matrix having dp rows and columns, where dp is
     * the number of dynamic parameters containing the system state.
     *
     * @return the process noise covariance matrix
     */
    public Matrix getProcessNoiseCov() {
        return processNoiseCov;
    }

    /**
     * Sets the process noise covariance matrix (Q).
     * This is a covariance matrix indicating the correlations of the amount of
     * error in the system state.
     * It must be provided a square symmetric matrix having dp rows and columns,
     * where dp is the number of dynamic parameters containing the system state
     * for this instance of a Kalman filter.
     * This setter method can be used for initial setup purposes, however
     * typically the process noise is difficult to determine. This matrix is
     * generally constructed intuitively so that un-modelled dynamics and
     * parameter uncertainties are modeled as process noise generally. If
     * the process noise is unknown, just leave the default value or provide
     * a diagonal matrix with the desired level of variance Q, where a low Q
     * variance indicates confidence that any unknown noise terms and/or
     * modelling errors are small to negligible, and higher Q allows the tracker
     * to follow the state despite unknown noise and/or model errors.
     *
     * @param processNoiseCov process noise covariance matrix
     * @throws IllegalArgumentException if provided matrix does not have dp
     *                                  rows and columns or it is not symmetric
     */
    public void setProcessNoiseCov(final Matrix processNoiseCov) {
        if (processNoiseCov.getRows() != dp ||
                processNoiseCov.getColumns() != dp ||
                !Utils.isSymmetric(processNoiseCov)) {
            throw new IllegalArgumentException();
        }

        this.processNoiseCov = processNoiseCov;
    }

    /**
     * Obtains the measurement noise covariance matrix (R).
     * This is a covariance matrix indicating the correlations of the amount of
     * error in the measures taken from the system.
     * It is a square symmetric matrix having mp rows and columns, where mp is
     * the number of measurement parameters.
     * Typically this matrix can be easily obtained by processing the
     * measurements while the output of the system is held constant. In this
     * case, only noise remains in the data after its mean is removed.
     * The covariance can be calculated easily from the remaining portion of the
     * data.
     *
     * @return the measurement noise covariance matrix
     */
    public Matrix getMeasurementNoiseCov() {
        return measurementNoiseCov;
    }

    /**
     * Sets the measurement noise covariance matrix (R).
     * This is a covariance matrix indicating the correlations of the amount of
     * error in the measures taken from the system.
     * Provided matrix must be a square symmetric matrix having mp rows and
     * columns, where mp is the number of measurement parameters.
     * Typically this matrix can be easily obtained by processing the
     * measurements while the output of the system is held constant. In this
     * case, only noise remains in the data after its mean is removed.
     * The covariance can be calculated easily from the remaining portion of the
     * data.
     * This setter method can be used for initial setup purposes.
     *
     * @param measurementNoiseCov new measurement noise covariance matrix
     * @throws IllegalArgumentException if provided matrix does not have mp
     *                                  rows and columns or it is not symmetric
     */
    public void setMeasurementNoiseCov(final Matrix measurementNoiseCov) {
        if (measurementNoiseCov.getRows() != mp ||
                measurementNoiseCov.getColumns() != mp ||
                !Utils.isSymmetric(measurementNoiseCov)) {
            throw new IllegalArgumentException();
        }

        this.measurementNoiseCov = measurementNoiseCov;
    }

    /**
     * Obtains the priori error estimate covariance matrix
     * (P'(k)): P'(k)=A*P(k-1)*At + Q).
     * It is a square symmetric matrix having dp rows and columns, where dp
     * is the number of dynamic parameters of the system state
     *
     * @return the priori error estimate covariance matrix
     */
    public Matrix getErrorCovPre() {
        return errorCovPre;
    }

    /**
     * Sets the priori error estimate covariance matrix
     * (P'(k)): P'(k)=A*P(k-1)*At + Q).
     * Provided matrix must be square and symmetric having dp rows and columns,
     * where dp is the number of the dynamic parameters of the system state set
     * for this Kalman filter instance.
     * This setter method can be used for initial setup purposes, however this
     * value will rarely need to be set, and instead the getter method will be
     * used to obtain the error of the predicted system state once the filter
     * converges
     *
     * @param errorCovPre new priori error estimate covariance matrix
     * @throws IllegalArgumentException if provided matrix does not have dp rows
     *                                  and columns or it is not symmetric
     */
    public void setErrorCovPre(final Matrix errorCovPre) {
        if (errorCovPre.getRows() != dp || errorCovPre.getColumns() != dp ||
                !Utils.isSymmetric(errorCovPre)) {
            throw new IllegalArgumentException();
        }
        this.errorCovPre = errorCovPre;
    }

    /**
     * Obtains the Kalman gain matrix (K(k)): K(k)=P'(k)*Ht*inv(H*P'(k)*Ht+R).
     * This matrix is used to correct the predicted state, if the gain values
     * are small then the filter is accurately tracking the system state and the
     * prediction error remains small too.
     * The gain matrix has dp rows and mp columns, where dp is the number of
     * dynamic parameters and mp is the number of measure parameters.
     *
     * @return the Kalman gain matrix
     */
    public Matrix getGain() {
        return gain;
    }

    /**
     * Sets the Kalman gain matrix (K(k)): K(k)=P'(k)*Ht*inv(H*P'(k)*Ht+R).
     * This matrix is used to correct the predicted state, if the gain values
     * are small then the filter is accurately tracking the system state and the
     * prediction error remains small too.
     * The gain matrix must have dp rows and mp columns, where dp is the number
     * of dynamic parameters and mp is the number of measure parameters set for
     * this Kalman filter instance.
     * This setter method can be used for initial setup purposes, however this
     * matrix rarely needs to be set, and instead it is better to let the filter
     * converge to the actual system state.
     *
     * @param gain new gain matrix
     * @throws IllegalArgumentException if provided matrix does not have dp rows
     *                                  and mp columns
     */
    public void setGain(final Matrix gain) {
        if (gain.getRows() != dp || gain.getColumns() != mp) {
            throw new IllegalArgumentException("Wrong matrix size");
        }
        this.gain = gain;
    }

    /**
     * Obtains the posteriori error estimate covariance matrix
     * (P(k)): P(k)=(I-K(k)*H)*P'(k).
     * It is a square symmetric matrix having dp rows and columns, where dp
     * is the number of dynamic parameters of the system state
     *
     * @return the priori error estimate covariance matrix
     */
    public Matrix getErrorCovPost() {
        return errorCovPost;
    }

    /**
     * Sets the posteriori error estimate covariance matrix
     * (P(k)): P(k)=(I-K(k)*H)*P'(k).
     * Provided matrix must be square and symmetric having dp rows and columns,
     * where dp is the number of the dynamic parameters of the system state set
     * for this Kalman filter instance.
     * This setter method can be used for initial setup purposes, however this
     * value will rarely need to be set, and instead the getter method will be
     * used to obtain the error of the posteriori system state once the filter
     * converges
     *
     * @param errorCovPost new posteriori error estimate covariance matrix
     * @throws IllegalArgumentException if provided matrix does not have dp rows
     *                                  and columns or it is not symmetric
     */
    public void setErrorCovPost(final Matrix errorCovPost) {
        if (errorCovPost.getRows() != dp || errorCovPost.getColumns() != dp ||
                !Utils.isSymmetric(errorCovPost)) {
            throw new IllegalArgumentException();
        }
        this.errorCovPost = errorCovPost;
    }
}