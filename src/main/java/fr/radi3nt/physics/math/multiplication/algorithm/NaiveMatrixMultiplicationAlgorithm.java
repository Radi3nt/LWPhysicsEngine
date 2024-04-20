package fr.radi3nt.physics.math.multiplication.algorithm;

import fr.radi3nt.physics.math.ArbitraryMatrix;
import fr.radi3nt.physics.math.matrices.array.FlatArrayArbitraryMatrix;
import fr.radi3nt.physics.math.multiplication.MatrixMultiplicationAlgorithm;

public class NaiveMatrixMultiplicationAlgorithm implements MatrixMultiplicationAlgorithm<ArbitraryMatrix, ArbitraryMatrix, FlatArrayArbitraryMatrix> {

    public static final NaiveMatrixMultiplicationAlgorithm INSTANCE = new NaiveMatrixMultiplicationAlgorithm();

    private NaiveMatrixMultiplicationAlgorithm() {
    }

    @Override
    public FlatArrayArbitraryMatrix multiply(ArbitraryMatrix a, ArbitraryMatrix b) {
        int width = b.getWidth();
        int height = a.getHeight();
        int common = a.getWidth();
        FlatArrayArbitraryMatrix result = new FlatArrayArbitraryMatrix(width, height);

        for (int x = 0; x < width; x++) {
            for (int i = 0; i < common; i++) {
                float cache = b.get(x, i);
                for (int y = 0; y < height; y++) {
                    result.add(x, y, a.get(i, y) * cache);
                }
            }
        }

        return result;
    }

    @Override
    public FlatArrayArbitraryMatrix multiplySwitched(ArbitraryMatrix a, ArbitraryMatrix b) {
        int width = a.getWidth();
        int height = b.getHeight();
        int common = a.getHeight();
        FlatArrayArbitraryMatrix result = new FlatArrayArbitraryMatrix(width, height);

        for (int y = 0; y < height; y++) {
            for (int i = 0; i < common; i++) {
                float cache = b.get(i, y);
                for (int x = 0; x < width; x++) {
                    result.add(x, y, cache * a.get(x, i));
                }
            }
        }

        return result;
        //or multiply(b, a);
    }

    @Override
    public FlatArrayArbitraryMatrix multiplyWithATransposed(ArbitraryMatrix a, ArbitraryMatrix b) {
        int width = b.getWidth();
        int height = a.getWidth();
        int common = a.getHeight();
        FlatArrayArbitraryMatrix result = new FlatArrayArbitraryMatrix(width, height);

        for (int x = 0; x < width; x++) {
            for (int i = 0; i < common; i++) {
                float cache = b.get(x, i);
                for (int y = 0; y < height; y++) {
                    result.add(x, y, a.get(y, i) * cache);
                }
            }
        }

        return result;
    }

    @Override
    public FlatArrayArbitraryMatrix multiplyWithBTransposedSwitched(ArbitraryMatrix a, ArbitraryMatrix b) {
        int width = a.getWidth();
        int height = b.getWidth();
        int common = b.getHeight();
        FlatArrayArbitraryMatrix result = new FlatArrayArbitraryMatrix(width, height);

        for (int x = 0; x < width; x++) {
            for (int i = 0; i < common; i++) {
                float cache = a.get(x, i);
                for (int y = 0; y < height; y++) {
                    result.add(x, y, b.get(y, i) * cache);
                }
            }
        }

        return result;
    }

    @Override
    public FlatArrayArbitraryMatrix multiplyWithBTransposed(ArbitraryMatrix a, ArbitraryMatrix b) {
        int width = b.getHeight();
        int height = a.getHeight();
        int common = a.getWidth();
        FlatArrayArbitraryMatrix result = new FlatArrayArbitraryMatrix(width, height);

        for (int x = 0; x < width; x++) {
            for (int i = 0; i < common; i++) {
                float cache = b.get(i, x);
                for (int y = 0; y < height; y++) {
                    result.add(x, y, a.get(i, y) * cache);
                }
            }
        }

        return result;
    }

    @Override
    public FlatArrayArbitraryMatrix multiplyWithATransposedSwitched(ArbitraryMatrix a, ArbitraryMatrix b) {
        int width = a.getHeight();
        int height = b.getHeight();
        int common = b.getWidth();
        FlatArrayArbitraryMatrix result = new FlatArrayArbitraryMatrix(width, height);

        for (int x = 0; x < width; x++) {
            for (int i = 0; i < common; i++) {
                float cache = a.get(i, x);
                for (int y = 0; y < height; y++) {
                    result.add(x, y, b.get(i, y) * cache);
                }
            }
        }

        return result;
    }

}
