package fr.radi3nt.physics.math.multiplication.algorithm;

import fr.radi3nt.maths.Maths;
import fr.radi3nt.physics.math.ArbitraryMatrix;
import fr.radi3nt.physics.math.matrices.array.BitFlatArrayArbitraryMatrix;
import fr.radi3nt.physics.math.matrices.array.FlatArrayArbitraryMatrix;
import fr.radi3nt.physics.math.matrices.sparse.SparseArbitraryMatrix;
import fr.radi3nt.physics.math.matrices.sparse.SparseBlock;
import fr.radi3nt.physics.math.multiplication.MatrixMultiplicationAlgorithm;

public class SparseMatrixMultiplicationAlgorithm implements MatrixMultiplicationAlgorithm<SparseArbitraryMatrix, ArbitraryMatrix, FlatArrayArbitraryMatrix> {

    public static final SparseMatrixMultiplicationAlgorithm INSTANCE = new SparseMatrixMultiplicationAlgorithm();
    private static final float EPSILON = 1e-4f;

    private SparseMatrixMultiplicationAlgorithm() {
    }

    @Override
    public FlatArrayArbitraryMatrix multiply(SparseArbitraryMatrix a, ArbitraryMatrix b) {
        int width = b.getWidth();
        int height = a.getHeight();
        FlatArrayArbitraryMatrix result = new FlatArrayArbitraryMatrix(width, height);

        for (SparseBlock sparseBlock : a.getSparseBlocks()) {
            for (int x = 0; x < width; x++) {
                for (int i = 0; i < sparseBlock.getWidth(); i++) {
                    float cache = b.get(x, i+sparseBlock.getStartX());
                    if (cache==0)
                        continue;
                    for (int y = 0; y < sparseBlock.getHeight(); y++) {
                        float value = sparseBlock.getLocal(i, y);
                        if (value==0)
                            continue;
                        result.add(x, y+sparseBlock.getStartY(), value * cache);
                    }
                }
            }
        }

        return result;
    }

    @Override
    public FlatArrayArbitraryMatrix multiplySwitched(SparseArbitraryMatrix a, ArbitraryMatrix b) {
        int width = a.getWidth();
        int height = b.getHeight();
        FlatArrayArbitraryMatrix result = new FlatArrayArbitraryMatrix(width, height);

        for (SparseBlock sparseBlock : a.getSparseBlocks()) {
            for (int y = 0; y < height; y++) {
                for (int i = 0; i < sparseBlock.getHeight(); i++) {
                    float cache = b.get(i+sparseBlock.getStartY(), y);
                    if (cache==0)
                        continue;
                    for (int x = 0; x < sparseBlock.getWidth(); x++) {
                        result.add(x+sparseBlock.getStartX(), y, cache * a.get(x+sparseBlock.getStartX(), i));
                    }
                }
            }
        }

        return result;
    }

    @Override
    public FlatArrayArbitraryMatrix multiplyWithATransposed(SparseArbitraryMatrix a, ArbitraryMatrix b) {
        int width = b.getWidth();
        int height = a.getWidth();
        FlatArrayArbitraryMatrix result = new FlatArrayArbitraryMatrix(width, height);

        for (SparseBlock sparseBlock : a.getSparseBlocks()) {
            for (int x = 0; x < width; x++) {
                for (int i = 0; i < sparseBlock.getHeight(); i++) {
                    float cache = b.get(x, i+sparseBlock.getStartY());
                    if (cache==0)
                        continue;
                    for (int y = 0; y < sparseBlock.getWidth(); y++) {
                        result.add(x, y+sparseBlock.getStartX(), sparseBlock.getLocal(y, i) * cache);
                    }
                }
            }
        }

        return result;
    }

    @Override
    public FlatArrayArbitraryMatrix multiplyWithBTransposedSwitched(SparseArbitraryMatrix a, ArbitraryMatrix b) {
        int width = a.getWidth();
        int height = b.getWidth();

        FlatArrayArbitraryMatrix result = new FlatArrayArbitraryMatrix(width, height);

        for (SparseBlock sparseBlock : a.getSparseBlocks()) {
            for (int y = 0; y < height; y++) {
                for (int i = 0; i < sparseBlock.getHeight(); i++) {
                    float cache = b.get(y, i+sparseBlock.getStartY());
                    if (cache==0)
                        continue;
                    for (int x = 0; x < sparseBlock.getWidth(); x++) {
                        result.add(x+sparseBlock.getStartX(), y, cache * sparseBlock.getLocal(x, i));
                    }
                }
            }
        }

        return result;
    }

    @Override
    public FlatArrayArbitraryMatrix multiplyWithBTransposed(SparseArbitraryMatrix a, ArbitraryMatrix b) {
        int width = b.getHeight();
        int height = a.getHeight();
        FlatArrayArbitraryMatrix result = new FlatArrayArbitraryMatrix(width, height);

        for (SparseBlock sparseBlock : a.getSparseBlocks()) {
            for (int x = 0; x < width; x++) {
                for (int i = 0; i < sparseBlock.getWidth(); i++) {
                    float cache = b.get(i+sparseBlock.getStartX(), x);
                    if (cache==0)
                        continue;
                    for (int y = 0; y < sparseBlock.getHeight(); y++) {
                        result.add(x, y+sparseBlock.getStartY(), sparseBlock.getLocal(i, y) * cache);
                    }
                }
            }
        }

        return result;
    }

    @Override
    public FlatArrayArbitraryMatrix multiplyWithATransposedSwitched(SparseArbitraryMatrix a, ArbitraryMatrix b) {
        int width = a.getHeight();
        int height = b.getHeight();
        FlatArrayArbitraryMatrix result = new FlatArrayArbitraryMatrix(width, height);

        for (SparseBlock sparseBlock : a.getSparseBlocks()) {
            for (int i = 0; i < sparseBlock.getWidth(); i++) {
                for (int y = 0; y < height; y++) {
                    float cache = b.get(i+sparseBlock.getStartX(), y);
                    if (cache==0)
                        continue;
                    for (int x = 0; x < sparseBlock.getHeight(); x++) {
                        float value = sparseBlock.getLocal(i, x);
                        if (value==0)
                            continue;
                        result.add(x+sparseBlock.getStartY(), y, cache * value);
                    }
                }
            }
        }

        return result;
    }


    public FlatArrayArbitraryMatrix multiplyWithATransposedSwitchedBit(SparseArbitraryMatrix a, BitFlatArrayArbitraryMatrix b) {
        int width = a.getHeight();
        int height = b.getHeight();
        FlatArrayArbitraryMatrix result = new FlatArrayArbitraryMatrix(width, height);

        for (SparseBlock sparseBlock : a.getSparseBlocks()) {
            for (int i = 0; i < sparseBlock.getWidth(); i++) {
                int y = -1;
                while (y+1<height && (y=b.next(i+sparseBlock.getStartX(), y+1))!=-1) {
                    float cache = b.get(i+sparseBlock.getStartX(), y);
                    if (Maths.fastAbs(cache)<EPSILON)
                        continue;
                    for (int x = 0; x < sparseBlock.getHeight(); x++) {
                        float value = sparseBlock.getLocal(i, x);
                        if (Maths.fastAbs(value)<EPSILON)
                            continue;
                        result.add(x+sparseBlock.getStartY(), y, cache * value);
                    }
                }
            }
        }

        return result;
    }

}
