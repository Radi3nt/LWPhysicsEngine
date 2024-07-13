package fr.radi3nt.physics.math.multiplication.algorithm;

import fr.radi3nt.physics.math.ArbitraryMatrix;
import fr.radi3nt.physics.math.matrices.EditableArbitraryMatrix;
import fr.radi3nt.physics.math.matrices.array.FlatArrayArbitraryMatrix;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class NaiveMatrixMultiplicationAlgorithmTest {

    private final EditableArbitraryMatrix resultMultiple = new FlatArrayArbitraryMatrix(2, 3);
    private final EditableArbitraryMatrix resultOneRow = new FlatArrayArbitraryMatrix(3, 4);

    private NaiveMatrixMultiplicationAlgorithm algorithm;

    @BeforeEach
    void setUp() {
        algorithm = NaiveMatrixMultiplicationAlgorithm.INSTANCE;

        setupResultOneRow();
        setupResultMultiple();
    }

    private void setupResultMultiple() {
        resultMultiple.set(0, 0, 22);
        resultMultiple.set(1, 0, 28);

        resultMultiple.set(0, 1, 49);
        resultMultiple.set(1, 1, 64);

        resultMultiple.set(0, 2, 76);
        resultMultiple.set(1, 2, 100);
    }

    private void setupResultOneRow() {
        resultOneRow.set(0, 0, 1);
        resultOneRow.set(0, 1, 2);
        resultOneRow.set(0, 2, 3);
        resultOneRow.set(0, 3, 4);

        resultOneRow.set(1, 0, 2);
        resultOneRow.set(1, 1, 4);
        resultOneRow.set(1, 2, 6);
        resultOneRow.set(1, 3, 8);

        resultOneRow.set(2, 0, 3);
        resultOneRow.set(2, 1, 6);
        resultOneRow.set(2, 2, 9);
        resultOneRow.set(2, 3, 12);
    }

    @Test
    void multiplyOneRow() {
        EditableArbitraryMatrix a = new FlatArrayArbitraryMatrix(1, 4);
        a.set(0, 0, 1);
        a.set(0, 1, 2);
        a.set(0, 2, 3);
        a.set(0, 3, 4);
        EditableArbitraryMatrix b = new FlatArrayArbitraryMatrix(3, 1);
        b.set(0, 0, 1);
        b.set(1, 0, 2);
        b.set(2, 0, 3);
        Assertions.assertEquals(algorithm.multiply(a, b), resultOneRow);
    }

    @Test
    void multiplyMultiple() {
        EditableArbitraryMatrix a = new FlatArrayArbitraryMatrix(3, 3);
        a.set(0, 0, 1);
        a.set(1, 0, 2);
        a.set(2, 0, 3);

        a.set(0, 1, 4);
        a.set(1, 1, 5);
        a.set(2, 1, 6);

        a.set(0, 2, 7);
        a.set(1, 2, 8);
        a.set(2, 2, 9);

        EditableArbitraryMatrix b = new FlatArrayArbitraryMatrix(2, 3);
        b.set(0, 0, 1);
        b.set(1, 0, 2);

        b.set(0, 1, 3);
        b.set(1, 1, 4);

        b.set(0, 2, 5);
        b.set(1, 2, 6);
        ArbitraryMatrix result = algorithm.multiply(a, b);
        Assertions.assertEquals(result, resultMultiple);
    }

    @Test
    void multiplySwitchedOneRow() {
        EditableArbitraryMatrix a = new FlatArrayArbitraryMatrix(1, 4);
        a.set(0, 0, 1);
        a.set(0, 1, 2);
        a.set(0, 2, 3);
        a.set(0, 3, 4);
        EditableArbitraryMatrix b = new FlatArrayArbitraryMatrix(3, 1);
        b.set(0, 0, 1);
        b.set(1, 0, 2);
        b.set(2, 0, 3);
        Assertions.assertEquals(algorithm.multiplySwitched(b, a), resultOneRow);
    }

    @Test
    void multiplySwitchedMultiple() {
        EditableArbitraryMatrix a = new FlatArrayArbitraryMatrix(3, 3);
        a.set(0, 0, 1);
        a.set(1, 0, 2);
        a.set(2, 0, 3);

        a.set(0, 1, 4);
        a.set(1, 1, 5);
        a.set(2, 1, 6);

        a.set(0, 2, 7);
        a.set(1, 2, 8);
        a.set(2, 2, 9);

        EditableArbitraryMatrix b = new FlatArrayArbitraryMatrix(2, 3);
        b.set(0, 0, 1);
        b.set(1, 0, 2);

        b.set(0, 1, 3);
        b.set(1, 1, 4);

        b.set(0, 2, 5);
        b.set(1, 2, 6);
        ArbitraryMatrix result = algorithm.multiplySwitched(b, a);
        Assertions.assertEquals(result, resultMultiple);
    }

    @Test
    void multiplyWithATransposedMultiple() {
        EditableArbitraryMatrix a = new FlatArrayArbitraryMatrix(3, 3);
        a.set(0, 0, 1);
        a.set(0, 1, 2);
        a.set(0, 2, 3);

        a.set(1, 0, 4);
        a.set(1, 1, 5);
        a.set(1, 2, 6);

        a.set(2, 0, 7);
        a.set(2, 1, 8);
        a.set(2, 2, 9);

        EditableArbitraryMatrix b = new FlatArrayArbitraryMatrix(2, 3);
        b.set(0, 0, 1);
        b.set(1, 0, 2);

        b.set(0, 1, 3);
        b.set(1, 1, 4);

        b.set(0, 2, 5);
        b.set(1, 2, 6);
        ArbitraryMatrix result = algorithm.multiplyWithATransposed(a, b);
        Assertions.assertEquals(result, resultMultiple);
    }

    @Test
    void multiplyWithATransposedOneRow() {
        EditableArbitraryMatrix a = new FlatArrayArbitraryMatrix(4, 1);
        a.set(0, 0, 1);
        a.set(1, 0, 2);
        a.set(2, 0, 3);
        a.set(3, 0, 4);
        EditableArbitraryMatrix b = new FlatArrayArbitraryMatrix(3, 1);
        b.set(0, 0, 1);
        b.set(1, 0, 2);
        b.set(2, 0, 3);
        ArbitraryMatrix result = algorithm.multiplyWithATransposed(a, b);
        Assertions.assertEquals(result, resultOneRow);
    }

    @Test
    void multiplyWithBTransposedSwitchedMultiple() {
        EditableArbitraryMatrix b = new FlatArrayArbitraryMatrix(3, 3);
        b.set(0, 0, 1);
        b.set(0, 1, 2);
        b.set(0, 2, 3);

        b.set(1, 0, 4);
        b.set(1, 1, 5);
        b.set(1, 2, 6);

        b.set(2, 0, 7);
        b.set(2, 1, 8);
        b.set(2, 2, 9);

        EditableArbitraryMatrix a = new FlatArrayArbitraryMatrix(2, 3);
        a.set(0, 0, 1);
        a.set(1, 0, 2);

        a.set(0, 1, 3);
        a.set(1, 1, 4);

        a.set(0, 2, 5);
        a.set(1, 2, 6);
        ArbitraryMatrix result = algorithm.multiplyWithBTransposedSwitched(a, b);
        Assertions.assertEquals(result, resultMultiple);
    }

    @Test
    void multiplyWithBTransposedSwitchedOneRow() {
        EditableArbitraryMatrix b = new FlatArrayArbitraryMatrix(4, 1);
        b.set(0, 0, 1);
        b.set(1, 0, 2);
        b.set(2, 0, 3);
        b.set(3, 0, 4);
        EditableArbitraryMatrix a = new FlatArrayArbitraryMatrix(3, 1);
        a.set(0, 0, 1);
        a.set(1, 0, 2);
        a.set(2, 0, 3);
        ArbitraryMatrix result = algorithm.multiplyWithBTransposedSwitched(a, b);
        Assertions.assertEquals(result, resultOneRow);
    }

    @Test
    void multiplyWithBTransposedMultiple() {
        EditableArbitraryMatrix a = new FlatArrayArbitraryMatrix(3, 3);
        a.set(0, 0, 1);
        a.set(1, 0, 2);
        a.set(2, 0, 3);

        a.set(0, 1, 4);
        a.set(1, 1, 5);
        a.set(2, 1, 6);

        a.set(0, 2, 7);
        a.set(1, 2, 8);
        a.set(2, 2, 9);

        EditableArbitraryMatrix b = new FlatArrayArbitraryMatrix(3, 2);
        b.set(0, 0, 1);
        b.set(0, 1, 2);

        b.set(1, 0, 3);
        b.set(1, 1, 4);

        b.set(2, 0, 5);
        b.set(2, 1, 6);
        ArbitraryMatrix result = algorithm.multiplyWithBTransposed(a, b);
        Assertions.assertEquals(result, resultMultiple);
    }

    @Test
    void multiplyWithBTransposedOneRow() {
        EditableArbitraryMatrix a = new FlatArrayArbitraryMatrix(1, 4);
        a.set(0, 0, 1);
        a.set(0, 1, 2);
        a.set(0, 2, 3);
        a.set(0, 3, 4);
        EditableArbitraryMatrix b = new FlatArrayArbitraryMatrix(1, 3);
        b.set(0, 0, 1);
        b.set(0, 1, 2);
        b.set(0, 2, 3);
        ArbitraryMatrix result = algorithm.multiplyWithBTransposed(a, b);
        Assertions.assertEquals(result, resultOneRow);
    }

    @Test
    void multiplyWithATransposedSwitchedMultiple() {
        EditableArbitraryMatrix b = new FlatArrayArbitraryMatrix(3, 3);
        b.set(0, 0, 1);
        b.set(1, 0, 2);
        b.set(2, 0, 3);

        b.set(0, 1, 4);
        b.set(1, 1, 5);
        b.set(2, 1, 6);

        b.set(0, 2, 7);
        b.set(1, 2, 8);
        b.set(2, 2, 9);

        EditableArbitraryMatrix a = new FlatArrayArbitraryMatrix(3, 2);
        a.set(0, 0, 1);
        a.set(0, 1, 2);

        a.set(1, 0, 3);
        a.set(1, 1, 4);

        a.set(2, 0, 5);
        a.set(2, 1, 6);
        ArbitraryMatrix result = algorithm.multiplyWithATransposedSwitched(a, b);
        Assertions.assertEquals(result, resultMultiple);
    }

    @Test
    void multiplyWithATransposedSwitchedOneRow() {
        EditableArbitraryMatrix b = new FlatArrayArbitraryMatrix(1, 4);
        b.set(0, 0, 1);
        b.set(0, 1, 2);
        b.set(0, 2, 3);
        b.set(0, 3, 4);
        EditableArbitraryMatrix a = new FlatArrayArbitraryMatrix(1, 3);
        a.set(0, 0, 1);
        a.set(0, 1, 2);
        a.set(0, 2, 3);
        ArbitraryMatrix result = algorithm.multiplyWithATransposedSwitched(a, b);
        Assertions.assertEquals(result, resultOneRow);
    }


}