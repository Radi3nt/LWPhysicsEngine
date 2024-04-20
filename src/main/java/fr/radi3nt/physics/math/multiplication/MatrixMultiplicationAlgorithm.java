package fr.radi3nt.physics.math.multiplication;

import fr.radi3nt.physics.math.ArbitraryMatrix;

public interface MatrixMultiplicationAlgorithm<A extends ArbitraryMatrix, B extends ArbitraryMatrix, R extends ArbitraryMatrix> {

    R multiply(A a, B b);
    R multiplySwitched(A a, B b);
    R multiplyWithATransposed(A a, B b);
    R multiplyWithBTransposedSwitched(A a, B b);
    R multiplyWithBTransposed(A a, B b);
    R multiplyWithATransposedSwitched(A a, B b);

}
