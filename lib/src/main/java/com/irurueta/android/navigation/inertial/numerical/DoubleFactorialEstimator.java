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

/**
 * Estimates factorial values as double precision floating point values.
 */
public class DoubleFactorialEstimator {

   /**
    * Cache size for factorial values that can be represented without overflowing with double
    * precision.
    * Values up to 170! are representable, 171! overflows.
    */
   public static final int CACHE_SIZE = 171;

   /**
    * Cache of values.
    */
   private final double[] cache;

   /**
    * Constructor with default cache size.
    */
   public DoubleFactorialEstimator() {
      this(CACHE_SIZE);
   }

   /**
    * Constructor.
    *
    * @param cacheSize size of cache.
    * @throws IllegalArgumentException if provided value is less than 1 or exceeds 171.
    */
   public DoubleFactorialEstimator(final int cacheSize)  {
      if (cacheSize < 1) {
         throw new IllegalArgumentException("Cache size must be at least 1");
      }
      if (cacheSize > CACHE_SIZE) {
         throw new IllegalArgumentException("Maximum allowed cache size is 171");
      }

      // initialize cache
      cache = new double[cacheSize];
      cache[0] = 1.0;
      for (int i = 1; i < cacheSize; i++) {
         cache[i] = i * cache[i - 1];
      }
   }

   /**
    * Gets current cache size.
    *
    * @return cache size.
    */
   public int getCacheSize() {
      return cache.length;
   }

   /**
    * Gets factorial of provided value represented with double precision.
    *
    * @param value value to compute factorial for.
    * @return computed factorial value.
    *
    * @throws IllegalArgumentException if provided value exceeds cache size.
    */
   public double factorial(final int value) {
      if (value < 0) {
         throw new IllegalArgumentException("Minimum supported value is 0");
      }
      if (value >= cache.length) {
         throw new IllegalArgumentException("Maximum supported value is " + (cache.length - 1));
      }

      return cache[value];
   }
}
