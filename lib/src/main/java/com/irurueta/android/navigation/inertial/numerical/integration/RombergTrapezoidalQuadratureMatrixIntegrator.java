/*
 * Copyright (C) 2023 Alberto Irurueta Carro (alberto@irurueta.com)
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
package com.irurueta.android.navigation.inertial.numerical.integration;

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.android.navigation.inertial.numerical.interpolation.InterpolationException;
import com.irurueta.android.navigation.inertial.numerical.interpolation.PolynomialInterpolator;
import com.irurueta.numerical.EvaluationException;

/**
 * Computes matrix function integration by using Romberg integration.
 */
public class RombergTrapezoidalQuadratureMatrixIntegrator
        extends RombergMatrixIntegrator<TrapezoidalMatrixQuadrature> {

    /**
     * Default accuracy.
     */
    public static final double EPS = 1e-10;

    /**
     * Maximum number of allowed steps.
     */
    private static final int JMAX = 20;

    /**
     * Maximum number of allowed steps + 1.
     */
    private static final int JMAXP = JMAX + 1;

    /**
     * Minimum required number of steps.
     */
    private static final int K = 5;

    /**
     * Successive trapezoidal approximations.
     */
    private final Matrix[] s = new Matrix[JMAX];

    /**
     * Successive trapezoidal step sizes.
     */
    private final double[] h = new double[JMAXP];

    /**
     * Constructor.
     *
     * @param a        Lower limit of integration.
     * @param b        Upper limit of integration.
     * @param listener listener to evaluate a single dimension matrix (multivariate) function at
     *                 required points.
     * @param eps      required accuracy.
     * @throws WrongSizeException if size notified by provided listener is invalid.
     */
    public RombergTrapezoidalQuadratureMatrixIntegrator(
            final double a, final double b,
            final MatrixSingleDimensionFunctionEvaluatorListener listener,
            final double eps) throws WrongSizeException {
        super(new TrapezoidalMatrixQuadrature(a, b, listener), eps);
    }

    /**
     * Constructor with default accuracy.
     *
     * @param a        Lower limit of integration.
     * @param b        Upper limit of integration.
     * @param listener listener to evaluate a single dimension matrix (multivariate) function at
     *                 required points.
     * @throws WrongSizeException if size notified by provided listener is invalid.
     */
    public RombergTrapezoidalQuadratureMatrixIntegrator(
            final double a, final double b,
            final MatrixSingleDimensionFunctionEvaluatorListener listener)
            throws WrongSizeException {
        this(a, b, listener, EPS);
    }

    /**
     * Integrates function between provided lower and upper limits.
     *
     * @param result instance where result of integration will be stored.
     * @throws IntegrationException if integration fails for numerical reasons.
     */
    @Override
    public void integrate(final Matrix result) throws IntegrationException {
        try {
            final int rows = q.getRows();
            final int columns = q.getColumns();
            final int elems = rows * columns;
            for (int i = 0; i < JMAX; i++) {
                s[i] = new Matrix(rows, columns);
            }

            final PolynomialInterpolator[] interpolators = new PolynomialInterpolator[elems];
            final double[][] sInterp = new double[elems][JMAX];
            for (int i = 0; i < elems; i++) {
                sInterp[i] = new double[JMAX];
                interpolators[i] = new PolynomialInterpolator(h, sInterp[i], K, false);
            }

            h[0] = 1.0;
            for (int j = 1; j <= JMAX; j++) {
                q.next(s[j - 1]);
                // update sInterp
                for (int i = 0; i < elems; i++) {
                    sInterp[i][j - 1] = s[j - 1].getElementAtIndex(i);
                }
                if (j >= K) {
                    boolean finished = true;
                    for (int i = 0; i < elems; i++) {
                        final double ss = interpolators[i].rawinterp(j - K, 0.0);
                        if (Double.isNaN(ss)) {
                            throw new IntegrationException("NaN was found");
                        }
                        result.setElementAtIndex(i, ss);
                        if (Math.abs(interpolators[i].getDy()) > eps * Math.abs(ss)) {
                            finished = false;
                        }
                    }

                    if (finished) {
                        return;
                    }
                }
                h[j] = 0.25 * h[j - 1];
            }
        } catch (final EvaluationException | InterpolationException | WrongSizeException e) {
            throw new IntegrationException(e);
        }

        // Too many steps
        throw new IntegrationException();
    }

    /**
     * Gets type of quadrature.
     *
     * @return type of quadrature.
     */
    @Override
    public QuadratureType getQuadratureType() {
        return QuadratureType.TRAPEZOIDAL;
    }
}
