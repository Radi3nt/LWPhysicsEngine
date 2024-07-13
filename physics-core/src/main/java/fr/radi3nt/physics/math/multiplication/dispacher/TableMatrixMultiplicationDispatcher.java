package fr.radi3nt.physics.math.multiplication.dispacher;

import fr.radi3nt.physics.math.ArbitraryMatrix;
import fr.radi3nt.physics.math.multiplication.MatrixMultiplicationAlgorithm;

import java.util.Collection;

public class TableMatrixMultiplicationDispatcher implements MatrixMultiplicationDispatcher {

    private final Collection<DispatchedMatrixMultiplicationAlgorithm> dispatchedMatrixMultiplicationAlgorithms;
    private final MatrixMultiplicationAlgorithm<ArbitraryMatrix, ArbitraryMatrix, ? extends ArbitraryMatrix> fallback;

    public TableMatrixMultiplicationDispatcher(Collection<DispatchedMatrixMultiplicationAlgorithm> dispatchedMatrixMultiplicationAlgorithms, MatrixMultiplicationAlgorithm<ArbitraryMatrix, ArbitraryMatrix, ? extends ArbitraryMatrix> fallback) {
        this.dispatchedMatrixMultiplicationAlgorithms = dispatchedMatrixMultiplicationAlgorithms;
        this.fallback = fallback;
    }

    @Override
    public ArbitraryMatrix multiply(ArbitraryMatrix a, ArbitraryMatrix b) {
        for (DispatchedMatrixMultiplicationAlgorithm dispatchedMatrixMultiplicationAlgorithm : dispatchedMatrixMultiplicationAlgorithms) {
            if (dispatchedMatrixMultiplicationAlgorithm.areConditionsMet(a, b))
                dispatchedMatrixMultiplicationAlgorithm.algorithm().multiply(a, b);
        }
        return fallback.multiply(a, b);
    }

    @Override
    public ArbitraryMatrix multiplySwitched(ArbitraryMatrix a, ArbitraryMatrix b) {
        for (DispatchedMatrixMultiplicationAlgorithm dispatchedMatrixMultiplicationAlgorithm : dispatchedMatrixMultiplicationAlgorithms) {
            if (dispatchedMatrixMultiplicationAlgorithm.areConditionsMet(a, b))
                dispatchedMatrixMultiplicationAlgorithm.algorithm().multiplySwitched(a, b);
        }
        return fallback.multiplySwitched(a, b);
    }

    @Override
    public ArbitraryMatrix multiplyWithATransposed(ArbitraryMatrix a, ArbitraryMatrix b) {
        for (DispatchedMatrixMultiplicationAlgorithm dispatchedMatrixMultiplicationAlgorithm : dispatchedMatrixMultiplicationAlgorithms) {
            if (dispatchedMatrixMultiplicationAlgorithm.areConditionsMet(a, b))
                dispatchedMatrixMultiplicationAlgorithm.algorithm().multiplyWithATransposed(a, b);
        }
        return fallback.multiplyWithATransposed(a, b);
    }

    @Override
    public ArbitraryMatrix multiplyWithBTransposedSwitched(ArbitraryMatrix a, ArbitraryMatrix b) {
        for (DispatchedMatrixMultiplicationAlgorithm dispatchedMatrixMultiplicationAlgorithm : dispatchedMatrixMultiplicationAlgorithms) {
            if (dispatchedMatrixMultiplicationAlgorithm.areConditionsMet(a, b))
                dispatchedMatrixMultiplicationAlgorithm.algorithm().multiplyWithBTransposedSwitched(a, b);
        }
        return fallback.multiplyWithBTransposedSwitched(a, b);
    }

    @Override
    public ArbitraryMatrix multiplyWithBTransposed(ArbitraryMatrix a, ArbitraryMatrix b) {
        for (DispatchedMatrixMultiplicationAlgorithm dispatchedMatrixMultiplicationAlgorithm : dispatchedMatrixMultiplicationAlgorithms) {
            if (dispatchedMatrixMultiplicationAlgorithm.areConditionsMet(a, b))
                dispatchedMatrixMultiplicationAlgorithm.algorithm().multiplyWithBTransposed(a, b);
        }
        return fallback.multiplyWithBTransposed(a, b);
    }

    @Override
    public ArbitraryMatrix multiplyWithATransposedSwitched(ArbitraryMatrix a, ArbitraryMatrix b) {
        for (DispatchedMatrixMultiplicationAlgorithm dispatchedMatrixMultiplicationAlgorithm : dispatchedMatrixMultiplicationAlgorithms) {
            if (dispatchedMatrixMultiplicationAlgorithm.areConditionsMet(a, b))
                dispatchedMatrixMultiplicationAlgorithm.algorithm().multiplyWithATransposedSwitched(a, b);
        }
        return fallback.multiplyWithATransposedSwitched(a, b);
    }
}
