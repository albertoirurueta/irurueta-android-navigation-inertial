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

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import com.irurueta.algebra.ArrayUtils;
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.Utils;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.sorting.Sorter;
import com.irurueta.statistics.UniformRandomizer;

import org.junit.Test;

import java.util.Random;

public class ExponentialMatrixEstimatorTest {

    private static final int SIZE = 4;

    private static final double MIN_VALUE = 0.1;

    private static final double MAX_VALUE = 1.0;

    private static final double ABSOLUTE_ERROR = 1e-12;

    private static final double LARGE_ABSOLUTE_ERROR = 1e-5;

    @Test(expected = IllegalArgumentException.class)
    public void exponential_whenNonSquareMatrix_throwsIllegalArgumentException() throws Exception {
        final Matrix a = new Matrix(2, 1);
        final ExponentialMatrixEstimator estimator = new ExponentialMatrixEstimator();
        estimator.exponential(a);
    }

    @Test(expected = IllegalArgumentException.class)
    public void exponential_whenResultHasInvalidRows_throwsIllegalArgumentException()
            throws Exception {
        final Matrix a = new Matrix(2, 2);
        final Matrix result = new Matrix(3, 3);
        final ExponentialMatrixEstimator estimator = new ExponentialMatrixEstimator();
        estimator.exponential(a, result);
    }

    @Test(expected = IllegalArgumentException.class)
    public void exponential_whenResultHasInvalidColumns_throwsIllegalArgumentException()
            throws Exception {
        final Matrix a = new Matrix(2, 2);
        final Matrix result = new Matrix(2, 3);
        final ExponentialMatrixEstimator estimator = new ExponentialMatrixEstimator();
        estimator.exponential(a, result);
    }

    @Test(expected = IllegalArgumentException.class)
    public void exponential_whenInvalidTolerance_throwsIllegalArgumentException() throws Exception {
        final Matrix a = new Matrix(2, 2);
        final ExponentialMatrixEstimator estimator = new ExponentialMatrixEstimator();
        estimator.exponential(a, -1.0);
    }

    @Test
    public void exponential_whenExample1_returnsExpectedValue() throws Exception {
        // This example is based on:
        // https://math.libretexts.org/Bookshelves/Linear_Algebra/Matrix_Analysis_(Cox)/10%3A_The_Matrix_Exponential/10.05%3A_The_Matrix_Exponential_via_Eigenvalues_and_Eigenvectors

        final Matrix a = new Matrix(2, 2);
        a.setElementAtIndex(0, 1.0);
        a.setElementAtIndex(1, 0.0);
        a.setElementAtIndex(2, 0.0);
        a.setElementAtIndex(3, 2.0);

        final Matrix expected = new Matrix(2, 2);
        expected.setElementAtIndex(0, Math.exp(1.0));
        expected.setElementAtIndex(1, 0.0);
        expected.setElementAtIndex(2, 0.0);
        expected.setElementAtIndex(3, Math.exp(2.0));

        final ExponentialMatrixEstimator estimator = new ExponentialMatrixEstimator();
        final Matrix result1 = estimator.exponential(a);
        final Matrix result2 = new Matrix(2, 2);
        estimator.exponential(a, result2);
        final Matrix result3 = estimator.exponential(a, LARGE_ABSOLUTE_ERROR);

        assertEquals(result1, result2);
        assertTrue(expected.equals(result1, ABSOLUTE_ERROR));
        assertTrue(expected.equals(result3, LARGE_ABSOLUTE_ERROR));
    }

    @Test
    public void exponential_whenExample2_returnsExpectedValue() throws Exception {
        // This example is based on:
        // https://math.libretexts.org/Bookshelves/Linear_Algebra/Matrix_Analysis_(Cox)/10%3A_The_Matrix_Exponential/10.05%3A_The_Matrix_Exponential_via_Eigenvalues_and_Eigenvectors

        final UniformRandomizer randomizer = new UniformRandomizer();
        final double t = randomizer.nextDouble(-Math.PI, Math.PI);

        final Matrix a = new Matrix(2, 2);
        a.setElementAtIndex(0, 0.0);
        a.setElementAtIndex(1, -1.0);
        a.setElementAtIndex(2, 1.0);
        a.setElementAtIndex(3, 0.0);
        a.multiplyByScalar(t);

        final Matrix expected = new Matrix(2, 2);
        expected.setElementAtIndex(0, Math.cos(t));
        expected.setElementAtIndex(1, -Math.sin(t));
        expected.setElementAtIndex(2, Math.sin(t));
        expected.setElementAtIndex(3, Math.cos(t));

        final ExponentialMatrixEstimator estimator = new ExponentialMatrixEstimator();
        final Matrix result1 = estimator.exponential(a);
        final Matrix result2 = new Matrix(2, 2);
        estimator.exponential(a, result2);
        final Matrix result3 = estimator.exponential(a, LARGE_ABSOLUTE_ERROR);

        assertEquals(result1, result2);
        assertTrue(expected.equals(result1, ABSOLUTE_ERROR));
        assertTrue(expected.equals(result3, 2.0 * LARGE_ABSOLUTE_ERROR));
    }

    @Test
    public void exponential_whenExample3_returnsExpectedValue() throws Exception {
        // This example is based on:
        // https://sites.millersville.edu/bikenaga/linear-algebra/matrix-exponential/matrix-exponential.html

        final UniformRandomizer randomizer = new UniformRandomizer();
        final double t = randomizer.nextDouble(-1.0, 1.0);

        final Matrix a = new Matrix(2, 2);
        a.setElementAtIndex(0, 1.0);
        a.setElementAtIndex(1, 0.0);
        a.setElementAtIndex(2, 2.0);
        a.setElementAtIndex(3, 1.0);
        a.multiplyByScalar(t);

        final Matrix expected = new Matrix(2, 2);
        expected.setElementAtIndex(0, Math.exp(t));
        expected.setElementAtIndex(1, 0.0);
        expected.setElementAtIndex(2, 2.0 * t * Math.exp(t));
        expected.setElementAtIndex(3, Math.exp(t));

        final ExponentialMatrixEstimator estimator = new ExponentialMatrixEstimator();
        final Matrix result1 = estimator.exponential(a);
        final Matrix result2 = new Matrix(2, 2);
        estimator.exponential(a, result2);
        final Matrix result3 = estimator.exponential(a, LARGE_ABSOLUTE_ERROR);

        assertEquals(result1, result2);
        assertTrue(expected.equals(result1, ABSOLUTE_ERROR));
        assertTrue(expected.equals(result3, LARGE_ABSOLUTE_ERROR));
    }

    @Test
    public void exponential_whenExample4_returnsExpectedValue() throws Exception {
        // This example is based on:
        // https://sites.millersville.edu/bikenaga/linear-algebra/matrix-exponential/matrix-exponential.html

        final UniformRandomizer randomizer = new UniformRandomizer();
        final double t = randomizer.nextDouble(-1.0, 1.0);

        final Matrix a = new Matrix(2, 2);
        a.setElementAtIndex(0, 3.0);
        a.setElementAtIndex(1, 1.0);
        a.setElementAtIndex(2, -10.0);
        a.setElementAtIndex(3, -4.0);
        a.multiplyByScalar(t);

        final Matrix expected = new Matrix(2, 2);
        expected.setElementAtIndex(0, 5.0 / 3.0 * Math.exp(t) - 2.0 / 3.0 * Math.exp(-2.0 * t));
        expected.setElementAtIndex(1, 1.0 / 3.0 * Math.exp(t) - 1.0 / 3.0 * Math.exp(-2.0 * t));
        expected.setElementAtIndex(2, -10.0 / 3.0 * Math.exp(t) + 10.0 / 3.0 * Math.exp(-2.0 * t));
        expected.setElementAtIndex(3, -2.0 / 3.0 * Math.exp(t) + 5.0 / 3.0 * Math.exp(-2.0 * t));

        final ExponentialMatrixEstimator estimator = new ExponentialMatrixEstimator();
        final Matrix result1 = estimator.exponential(a);
        final Matrix result2 = new Matrix(2, 2);
        estimator.exponential(a, result2);
        final Matrix result3 = estimator.exponential(a, LARGE_ABSOLUTE_ERROR);

        assertEquals(result1, result2);
        assertTrue(expected.equals(result1, ABSOLUTE_ERROR));
        assertTrue(expected.equals(result3, LARGE_ABSOLUTE_ERROR));
    }

    @Test
    public void exponential_whenExample5_returnsExpectedValue() throws Exception {
        // This example is based on:
        // https://sites.millersville.edu/bikenaga/linear-algebra/matrix-exponential/matrix-exponential.html

        final UniformRandomizer randomizer = new UniformRandomizer();
        final double t = randomizer.nextDouble(-1.0, 1.0);
        final int size = randomizer.nextInt(1, SIZE + 1);

        final Matrix v = getOrthonormalMatrix(size);
        final Matrix invV = Utils.inverse(v);


        // create diagonal of singular values in descending order
        final double[] diag = new double[size];
        randomizer.fill(diag, MIN_VALUE, MAX_VALUE);
        Sorter.create().sort(diag);
        ArrayUtils.reverse(diag);

        // compute matrix as A = V * diag * V^-1
        final Matrix a = v.multiplyAndReturnNew(Matrix.diagonal(diag))
                .multiplyAndReturnNew(invV)
                .multiplyByScalarAndReturnNew(t);

        // expected exponential matrix should be:
        // exp(A) = V * diag(exp(lamba0) ... exp(lanbdaN)) * V^T
        final double[] expDiag = new double[size];
        for (int i = 0; i < size; i++) {
            expDiag[i] = Math.exp(diag[i] * t);
        }

        final Matrix expected = v.multiplyAndReturnNew(Matrix.diagonal(expDiag))
                .multiplyAndReturnNew(invV);

        final ExponentialMatrixEstimator estimator = new ExponentialMatrixEstimator();
        final Matrix result1 = estimator.exponential(a);
        final Matrix result2 = new Matrix(size, size);
        estimator.exponential(a, result2);
        final Matrix result3 = estimator.exponential(a, LARGE_ABSOLUTE_ERROR);

        assertEquals(result1, result2);
        assertTrue(expected.equals(result1, ABSOLUTE_ERROR));
        assertTrue(expected.equals(result3, size * LARGE_ABSOLUTE_ERROR));
    }

    private static Matrix getOrthonormalMatrix(final int rows) throws WrongSizeException {

        final Matrix m = Matrix.identity(rows, rows);
        final double value = 1.0 / Math.sqrt(rows);
        m.multiplyByScalar(value);

        // now scramble an amount of columns
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        int originColumn;
        int destinationColumn;
        final double[] column1 = new double[rows];
        final double[] column2 = new double[rows];

        for (int i = 0; i < rows; i++) {
            originColumn = randomizer.nextInt(0, rows);
            destinationColumn = randomizer.nextInt(0, rows);
            m.getSubmatrixAsArray(0, originColumn, rows - 1, originColumn,
                    column1);
            m.getSubmatrixAsArray(0, destinationColumn, rows - 1,
                    destinationColumn, column2);

            // now swap columns
            m.setSubmatrix(0, originColumn, rows - 1, originColumn, column2);
            m.setSubmatrix(0, destinationColumn, rows - 1, destinationColumn,
                    column1);
        }

        return m;
    }
}