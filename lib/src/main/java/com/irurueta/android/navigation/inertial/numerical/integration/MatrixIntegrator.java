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

/**
 * Integrates single dimension matrix (multivariate) functions over a specified interval.
 */
public abstract class MatrixIntegrator {

    /**
     * Default integrator type. Picks the safest option when no prior knowledge about the integrand
     * is known.
     */
    public static final IntegratorType DEFAULT_INTEGRATOR_TYPE = IntegratorType.ROMBERG;

    /**
     * Default quadrature type. Picks the safest option when no prior knowledge about the integrand
     * is known.
     */
    public static QuadratureType DEFAULT_QUADRATURE_TYPE = QuadratureType.TRAPEZOIDAL;

    /**
     * Gets type of integrator.
     *
     * @return type of integrator.
     */
    public abstract IntegratorType getIntegratorType();

    /**
     * Gets type of quadrature.
     *
     * @return type of quadrature.
     */
    public abstract QuadratureType getQuadratureType();

    /**
     * Integrates function between provided lower and upper limits.
     *
     * @param result instance where result of integration will be stored.
     * @throws IntegrationException if integration fails for numerical reasons.
     */
    public abstract void integrate(final Matrix result) throws IntegrationException;

    /**
     * Creates an integrator using provided integrator and quadrature types.
     * It must be noticed that upper limit of integration is ignored when using exponential mid
     * point quadrature type for Romberg's integration method.
     *
     * @param a              Lower limit of integration.
     * @param b              Upper limit of integration.
     * @param listener       listener to evaluate a single dimension function at required points.
     * @param eps            required accuracy.
     * @param integratorType integrator type.
     * @param quadratureType quadrature type.
     * @return created integrator.
     * @throws IllegalArgumentException if provided quadrature type is not supported for required
     *                                  integrator type.
     * @throws WrongSizeException       if size notified by provided listener is invalid.
     */
    public static MatrixIntegrator create(
            final double a, final double b,
            final MatrixSingleDimensionFunctionEvaluatorListener listener, final double eps,
            final IntegratorType integratorType,
            final QuadratureType quadratureType) throws WrongSizeException {
        return switch (integratorType) {
            case ROMBERG -> RombergMatrixIntegrator.create(a, b, listener, eps, quadratureType);
            case SIMPSON -> SimpsonMatrixIntegrator.create(a, b, listener, eps, quadratureType);
            default -> QuadratureMatrixIntegrator.create(a, b, listener, eps, quadratureType);
        };
    }

    /**
     * Creates an integrator using provided integrator and quadrature types with default accuracy.
     * It must be noticed that upper limit of integration is ignored when using exponential mid
     * point quadrature type for Romberg's integration method.
     *
     * @param a              Lower limit of integration.
     * @param b              Upper limit of integration.
     * @param listener       listener to evaluate a single dimension function at required points.
     * @param integratorType integrator type.
     * @param quadratureType quadrature type.
     * @return created integrator.
     * @throws IllegalArgumentException if provided quadrature type is not supported for required
     *                                  integrator type.
     * @throws WrongSizeException       if size notified by provided listener is invalid.
     */
    public static MatrixIntegrator create(
            final double a, final double b,
            final MatrixSingleDimensionFunctionEvaluatorListener listener,
            final IntegratorType integratorType,
            final QuadratureType quadratureType) throws WrongSizeException {
        return switch (integratorType) {
            case ROMBERG -> RombergMatrixIntegrator.create(a, b, listener, quadratureType);
            case SIMPSON -> SimpsonMatrixIntegrator.create(a, b, listener, quadratureType);
            default -> QuadratureMatrixIntegrator.create(a, b, listener, quadratureType);
        };
    }

    /**
     * Creates an integrator using provided integrator type and using default quadrature type.
     * It must be noticed that upper limit of integration is ignored when using exponential mid
     * point quadrature type for Romberg's integration method.
     *
     * @param a              Lower limit of integration.
     * @param b              Upper limit of integration.
     * @param listener       listener to evaluate a single dimension function at required points.
     * @param eps            required accuracy.
     * @param integratorType integrator type.
     * @return created integrator.
     * @throws WrongSizeException if size notified by provided listener is invalid.
     */
    public static MatrixIntegrator create(
            final double a, final double b,
            final MatrixSingleDimensionFunctionEvaluatorListener listener, final double eps,
            final IntegratorType integratorType) throws WrongSizeException {
        return switch (integratorType) {
            case ROMBERG -> RombergMatrixIntegrator.create(a, b, listener, eps);
            case SIMPSON -> SimpsonMatrixIntegrator.create(a, b, listener, eps);
            default -> QuadratureMatrixIntegrator.create(a, b, listener, eps);
        };
    }

    /**
     * Creates an integrator using provided integrator type with default accuracy and using default
     * quadrature type.
     * It must be noticed that upper limit of integration is ignored when using exponential mid
     * point quadrature type for Romberg's integration method.
     *
     * @param a              Lower limit of integration.
     * @param b              Upper limit of integration.
     * @param listener       listener to evaluate a single dimension function at required points.
     * @param integratorType integrator type.
     * @return created integrator.
     * @throws WrongSizeException if size notified by provided listener is invalid.
     */
    public static MatrixIntegrator create(
            final double a, final double b,
            final MatrixSingleDimensionFunctionEvaluatorListener listener,
            final IntegratorType integratorType) throws WrongSizeException {
        return switch (integratorType) {
            case ROMBERG -> RombergMatrixIntegrator.create(a, b, listener);
            case SIMPSON -> SimpsonMatrixIntegrator.create(a, b, listener);
            default -> QuadratureMatrixIntegrator.create(a, b, listener);
        };
    }

    /**
     * Creates an integrator using default integrator and quadrature types.
     * It must be noticed that upper limit of integration is ignored when using exponential mid
     * point quadrature type for Romberg's integration method.
     *
     * @param a        Lower limit of integration.
     * @param b        Upper limit of integration.
     * @param listener listener to evaluate a single dimension function at required points.
     * @param eps      required accuracy.
     * @return created integrator.
     * @throws WrongSizeException if size notified by provided listener is invalid.
     */
    public static MatrixIntegrator create(
            final double a, final double b,
            final MatrixSingleDimensionFunctionEvaluatorListener listener, final double eps)
            throws WrongSizeException {
        return create(a, b, listener, eps, DEFAULT_INTEGRATOR_TYPE);
    }

    /**
     * Creates an integrator using default integrator and quadrature types with default accuracy.
     * It must be noticed that upper limit of integration is ignored when using exponential mid
     * point quadrature type for Romberg's integration method.
     *
     * @param a        Lower limit of integration.
     * @param b        Upper limit of integration.
     * @param listener listener to evaluate a single dimension function at required points.
     * @return created integrator.
     * @throws WrongSizeException if size notified by provided listener is invalid.
     */
    public static MatrixIntegrator create(
            final double a, final double b,
            final MatrixSingleDimensionFunctionEvaluatorListener listener)
            throws WrongSizeException {
        return create(a, b, listener, DEFAULT_INTEGRATOR_TYPE);
    }
}
