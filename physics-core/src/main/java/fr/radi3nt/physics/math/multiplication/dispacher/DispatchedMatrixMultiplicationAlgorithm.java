package fr.radi3nt.physics.math.multiplication.dispacher;

import fr.radi3nt.physics.math.ArbitraryMatrix;
import fr.radi3nt.physics.math.multiplication.MatrixMultiplicationAlgorithm;

public interface DispatchedMatrixMultiplicationAlgorithm {

    boolean areConditionsMet(ArbitraryMatrix a, ArbitraryMatrix b);
    MatrixMultiplicationAlgorithm<ArbitraryMatrix, ArbitraryMatrix, ArbitraryMatrix> algorithm();

}
