package fr.radi3nt.physics.constraints.sle;

import fr.radi3nt.maths.Maths;
import fr.radi3nt.maths.components.arbitrary.VectorNf;
import fr.radi3nt.physics.math.ArbitraryMatrix;

public class ProjectedGaussSeidelSolver implements SleSolver {

    private static final float THRESHOLD = 1f;

    private final float threshold;
    private int iterations;

    public ProjectedGaussSeidelSolver(float threshold, int iterations) {
        this.threshold = threshold;
        this.iterations = iterations;
    }

    private float solveIteration(ArbitraryMatrix a, VectorNf b, VectorNf lambda, VectorNf min, VectorNf max) {
        float maxDifference = 0.0f;

        int height = lambda.size();
        for (int row = 0; row < height; row++) {
            float s0 = 0, s1 = 0;
            for (int j = 0; j < row; ++j) {
                s0 += a.get(j, row) * lambda.get(j);
            }

            for (int j = row + 1; j < height; ++j) {
                s1 += a.get(j, row) * lambda.get(j);
            }

            float limitMax = max.get(row);
            float limitMin = min.get(row);

            float kNextI =
                    (1 / a.get(row, row)) * (b.get(row) - s0 - s1);

            float x = Maths.clamp(kNextI, limitMin, limitMax);

            float kBefore = lambda.get(row);
            float minKBefore = Math.max(THRESHOLD, Math.abs(kBefore));

            float delta = Math.abs(x - kBefore) / minKBefore;
            maxDifference = Math.max(delta, maxDifference);

            lambda.set(row, x);
        }
        return maxDifference;
    }

    private float solveIteration(ArbitraryMatrix a, VectorNf b, VectorNf lambda) {
        float maxDifference = 0.0f;

        int height = lambda.size();
        for (int row = 0; row < height; row++) {
            float s0 = 0, s1 = 0;
            for (int j = 0; j < row; ++j) {
                s0 += a.get(j, row) * lambda.get(j);
            }

            for (int j = row + 1; j < height; ++j) {
                s1 += a.get(j, row) * lambda.get(j);
            }

            float k_next_i =
                    (1 / a.get(row, row)) * (b.get(row) - s0 - s1);

            float k_before = lambda.get(row);
            float delta = Math.abs(k_next_i - k_before);
            maxDifference = Math.max(delta, maxDifference);

            lambda.set(row, k_next_i);
        }
        return maxDifference;
    }

    public void setIterations(int iterations) {
        this.iterations = iterations;
    }

    @Override
    public VectorNf solve(VectorNf lambda, ArbitraryMatrix a, VectorNf b, VectorNf min, VectorNf max) {
        int size = b.size();
        if (lambda.size()!=size)
            throw new UnsupportedOperationException("Lambda size isn't right");

        int i = 0;
        for (; i < iterations; i++) {
            float maxDelta = solveIteration(a, b, lambda, min, max);
            if (maxDelta <= threshold) {
                break;
            }
        }
        return lambda;
    }
}
