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

import com.irurueta.numerical.EvaluationException;
import com.irurueta.numerical.SingleDimensionFunctionEvaluatorListener;

/**
 * Implementation of quadrature using trapezoidal algorithm.
 * This implementation is suitable for non-improper integrands, which consist
 * of functions with not known singularities that can be evaluated on all the integration
 * interval, which must be finite.
 */
public class TrapezoidalQuadrature extends Quadrature {

   /**
    * Lower limit of integration.
    */
   private final double a;

   /**
    * Upper limit of integration.
    */
   private final double b;

   /**
    * Current value of integral.
    */
   private double s;

   /**
    * Listener to evaluate single dimension functions at required points.
    */
   private final SingleDimensionFunctionEvaluatorListener listener;

   /**
    * Constructor.
    *
    * @param a Lower limit of integration.
    * @param b Upper limit of integration.
    * @param listener listener to evaluate a single dimension function at required points.
    */
   public TrapezoidalQuadrature(
           final double a, final double b,
           final SingleDimensionFunctionEvaluatorListener listener) {
      this.n = 0;
      this.a = a;
      this.b = b;
      this.s = 0;
      this.listener = listener;
   }

   /**
    * Gets lower limit of integration.
    *
    * @return lower limit of integration.
    */
   public double getA() {
      return a;
   }

   /**
    * Gets upper limit of integration.
    *
    * @return upper limit of integration.
    */
   public double getB() {
      return b;
   }

   /**
    * Gets current value of integral.
    * @return current value of integral.
    */
   public double getS() {
      return s;
   }

   /**
    * Returns the value of the integral at the nth stage of refinement.
    *
    * @return the value of the integral at the nth stage of refinement.
    * @throws EvaluationException Raised if something failed during the evaluation.
    */
   @Override
   public double next() throws EvaluationException {
      double x;
      double tnm;
      double sum;
      double del;
      int it;
      int j;
      n++;
      if (n == 1) {
         return (s = 0.5 * (b - a) * (listener.evaluate(a) + listener.evaluate(b)));
      } else {
         for (it = 1, j = 1; j < n - 1; j++) {
            it <<= 1;
         }
         tnm = it;
         // This is the spacing of the points to be added
         del = (b - a) / tnm;
         x = a + 0.5 * del;
         for (sum = 0.0, j = 0; j < it; j++, x += del) {
            sum += listener.evaluate(x);
         }
         // This replaces s by its refined value
         s = 0.5 * (s + (b - a) * sum / tnm);
         return s;
      }
   }

   /**
    * Gets type of quadrature.
    *
    * @return type of quadrature.
    */
   @Override
   public QuadratureType getType() {
      return QuadratureType.TRAPEZOIDAL;
   }
}
