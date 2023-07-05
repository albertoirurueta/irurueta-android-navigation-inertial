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
package com.irurueta.android.navigation.inertial.numerical;

import com.irurueta.algebra.AlgebraException;
import com.irurueta.algebra.LUDecomposer;
import com.irurueta.algebra.Matrix;
import com.irurueta.numerical.NumericalException;

/**
 * Estimates the Padé approximant rational function by using a number of coefficients
 * of a Taylor series.
 * Padé approximants can yield more accurate solutions than Taylor series in certain situations.
 */
public class PadeApproximantEstimator {

   /**
    * Number of times to iteratively improve LU decomposition solution by default.
    */
   private static final int DEFAULT_IMPROVEMENT_TIMES = 4;

   /**
    * Number of times to iteratively improve LU decomposition.
    */
   private final int improveTimes;

   /**
    * Computes LU decomposition to find denominator coefficients.
    */
   private final LUDecomposer luDecomposer = new LUDecomposer();

   /**
    * Number of coefficients being processed based on provided Taylor power series ones.
    */
   private int n;

   /**
    * Contains matrix to solve Padé coefficients.
    */
   private Matrix q;

   /**
    * Contains intermediate solution for denominator coefficients.
    */
   private Matrix x;

   /**
    * Contains intermediate solution for numerator coefficients.
    */
   private Matrix y;

   /**
    * Contains product of a an x matrices to iteratively improve LU solution.
    */
   private Matrix ax;

   /**
    * Contains residual to iteratively improve LU solution.
    */
   private Matrix residual;

   /**
    * Improved LU solution in one iteration.
    */
   private Matrix improvedX;

   /**
    * Default constructor.
    * Uses default number of times to iteratively improve solution.
    */
   public PadeApproximantEstimator() {
      this(DEFAULT_IMPROVEMENT_TIMES);
   }

   /**
    * Constructor.
    * @param improveTimes number of times to iteratively improve solution.
    * @throws IllegalArgumentException if provided number of times is negative.
    */
   public PadeApproximantEstimator(final int improveTimes) {
      if (improveTimes < 0) {
         throw new IllegalArgumentException("Times must be zero or greater");
      }

      this.improveTimes = improveTimes;
   }

   /**
    * Estimates Padé coefficients for provided Taylor power series ones.
    *
    * @param taylorCoefficients Taylor series coefficients.
    * @return Result containing Padé approximant numerator and denominator coefficients.
    * @throws NumericalException if a numerical error occurs.
    * @throws IllegalArgumentException if provided number of Taylor series coefficients is less
    * than 3.
    */
   public Result estimatePadeCoefficients(final double[] taylorCoefficients)
           throws NumericalException {
      final int n = (taylorCoefficients.length - 1) / 2;
      final double[] num = new double[n + 1];
      final double[] denom = new double[n + 1];
      estimatePadeCoefficients(taylorCoefficients, n, num, denom);
      return new Result(num, denom);
   }

   /**
    * Estimates Padé coefficients for provided Taylor power series ones.
    *
    * @param taylorCoefficients Taylor series coefficients.
    * @param numeratorResult numerator coefficients of Padé approximant
    *                        (must be (taylorCoefficients.length - 1) / 2).
    * @param denominatorResult denominator coefficients of Padé approximant
    *                          (must be (taylorCoefficients.length - 1) / 2)..
    * @throws NumericalException if a numerical error occurs.
    * @throws IllegalArgumentException if provided number of Taylor series coefficients is less
    * than 3 or if provided numerator or denominator result coefficients have an invalid size..
    */
   public void estimatePadeCoefficients(
           final double[] taylorCoefficients, final double[] numeratorResult,
           final double[] denominatorResult) throws NumericalException {
      final int n = (taylorCoefficients.length - 1) / 2;
      estimatePadeCoefficients(taylorCoefficients, n, numeratorResult, denominatorResult);
   }

   /**
    * Estimates Padé coefficients for provided Taylor power series ones.
    *
    * @param taylorCoefficients Taylor series coefficients.
    * @param n Number of padé coefficients to generate.
    * @param numeratorResult numerator coefficients of Padé approximant
    *                        (must be (taylorCoefficients.length - 1) / 2).
    * @param denominatorResult denominator coefficients of Padé approximant
    *                          (must be (taylorCoefficients.length - 1) / 2)..
    * @throws NumericalException if a numerical error occurs.
    * @throws IllegalArgumentException if provided number of Taylor series coefficients is less
    * than 3 or if provided numerator or denominator result coefficients have an invalid size..
    */
   private void estimatePadeCoefficients(
           final double[] taylorCoefficients, final int n, final double[] numeratorResult,
           final double[] denominatorResult) throws NumericalException {
      if (taylorCoefficients.length < 3) {
         throw new IllegalArgumentException(
                 "Length of Tayloar series coefficients must be at least 3");
      }

      try {
         // Based on Numerical Recipes section 5.12 Padé Approximants page 245.
         final int nPlusOne = n +1;
         if (numeratorResult.length != nPlusOne || denominatorResult.length != nPlusOne) {
            throw new IllegalArgumentException("Wrong numerator or denominator array length");
         }

         if (this.n != n) {
            initialize(n);
         }
         int j, k;
         double sum;

         for (j = 0; j < n; j++) {
            // set up matrix for solving
            y.setElementAtIndex(j, taylorCoefficients[n + j + 1]);
            for (k = 0; k < n; k++) {
               q.setElementAt(j, k, taylorCoefficients[j - k + n]);
            }
         }

         luDecomposer.setInputMatrix(q);
         luDecomposer.decompose();
         luDecomposer.solve(y, x);

         for (j = 0; j < improveTimes; j++) {
            improveLuSolve(q, y, x, improvedX);
            x.copyFrom(improvedX);
         }

         for (k = 0; k < n; k++) {
            for (sum = taylorCoefficients[k + 1], j = 0; j <= k; j++) {
               sum -= x.getElementAtIndex(j) * taylorCoefficients[k - j];
            }
            y.setElementAtIndex(k, sum);
         }
         numeratorResult[0] = taylorCoefficients[0];
         denominatorResult[0] = 1.0;
         for (j = 0; j < n; j++) {
            numeratorResult[j + 1] = y.getElementAtIndex(j);
            denominatorResult[j + 1] = -x.getElementAtIndex(j);
         }

      } catch (final AlgebraException ex) {
         throw new NumericalException(ex);
      }
   }

   /**
    * One step to iteratively improve LU solve solution.
    *
    * @param a a matrix of a linear system of equations to be solved.
    * @param b b matrix of a linear system of equations to be solved.
    * @param x x matrix containing initial solution of linear system of equations to be improved.
    * @param result matrix where result will be stored.
    * @throws AlgebraException if a numerical error occurs.
    */
   private void improveLuSolve(final Matrix a, final Matrix b, final Matrix x, final Matrix result)
           throws AlgebraException {
      // Based on Numerical Recipes page 62
      // We need to solve iteratively: A * deltaX  = A * (x + deltaX) - b
      // deltaX is the residual error between initially estimated x and the true x
      // Hence:
      // result = x - deltaX
      // Where result will be closer to the true x

      a.multiply(x, ax);
      ax.subtract(b, residual);

      luDecomposer.solve(residual, result);

      result.multiplyByScalar(-1.0);
      result.add(x);
   }

   /**
    * Initializes required matrices.
    *
    * @param n length of required number of Padé coefficients for provided Taylor series ones.
    * @throws AlgebraException if a numerical error occurs.
    */
   private void initialize(final int n) throws AlgebraException {
      q = new Matrix(n, n);
      x = new Matrix(n, 1);
      y = new Matrix(n, 1);

      if (improveTimes > 0) {
         ax = new Matrix(n, 1);
         residual = new Matrix(n, 1);
         improvedX = new Matrix(n, 1);
      }

      this.n = n;
   }

   /**
    * Contains result of Padé approximant.
    */
   public static class Result {

      /**
       * Numerator coefficients.
       */
      private final double[] mNumerators;

      /**
       * Denominator coefficients.
       */
      private final double[] mDenominators;

      /**
       * Constructor.
       *
       * @param numerators numerator coefficients.
       * @param denominators denominator coefficients.
       */
      public Result(final double[] numerators, final double[] denominators) {
         mNumerators = numerators;
         mDenominators = denominators;
      }

      /**
       * Gets numerator coefficients.
       *
       * @return numerator coefficients.
       */
      public double[] getNumerators() {
         return mNumerators;
      }

      /**
       * Gets denominator coefficients.
       *
       * @return denominator coefficients.
       */
      public double[] getDenominators() {
         return mDenominators;
      }
   }
}
