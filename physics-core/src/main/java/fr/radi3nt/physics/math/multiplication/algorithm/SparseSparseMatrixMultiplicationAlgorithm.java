package fr.radi3nt.physics.math.multiplication.algorithm;

import fr.radi3nt.maths.Maths;
import fr.radi3nt.physics.math.matrices.array.BitFlatArrayArbitraryMatrix;
import fr.radi3nt.physics.math.matrices.array.FlatArrayArbitraryMatrix;
import fr.radi3nt.physics.math.matrices.sparse.SparseArbitraryMatrix;
import fr.radi3nt.physics.math.matrices.sparse.SparseBlock;
import fr.radi3nt.physics.math.multiplication.MatrixMultiplicationAlgorithm;

public class SparseSparseMatrixMultiplicationAlgorithm implements MatrixMultiplicationAlgorithm<SparseArbitraryMatrix, SparseArbitraryMatrix, FlatArrayArbitraryMatrix> {

    public static final SparseSparseMatrixMultiplicationAlgorithm INSTANCE = new SparseSparseMatrixMultiplicationAlgorithm();
    private static final float EPSILON = 1e-4f;

    private SparseSparseMatrixMultiplicationAlgorithm() {
    }

    @Override
    public FlatArrayArbitraryMatrix multiply(SparseArbitraryMatrix a, SparseArbitraryMatrix b) {
        int width = b.getWidth();
        int height = a.getHeight();
        FlatArrayArbitraryMatrix result = new FlatArrayArbitraryMatrix(width, height);

        for (SparseBlock aSparse : a.getSparseBlocks()) {
            for (SparseBlock bSparse : b.getSparseBlocks()) {
                int startI = Math.max(aSparse.getStartX()-bSparse.getStartY(), 0)-aSparse.getStartX()+bSparse.getStartY();
                int endI = Math.min(aSparse.getStartX()+aSparse.getWidth()-bSparse.getStartY(), bSparse.getHeight())-aSparse.getStartX()+bSparse.getStartY();
                /*
                Equivalent:
                if (currentA<0 || currentA>=bSparse.getHeight()) {
                        continue;
                    }
                 */
                for (int i = startI; i < endI; i++) {
                    int currentA = i+aSparse.getStartX()-bSparse.getStartY();
                    for (int x = 0; x < bSparse.getWidth(); x++) {
                        float cache = bSparse.getLocal(x, currentA);
                        if (cache==0)
                            continue;
                        for (int y = 0; y < aSparse.getHeight(); y++) {
                            float value = aSparse.getLocal(i, y);
                            if (value==0)
                                continue;
                            result.add(x+bSparse.getStartX(), y+aSparse.getStartY(), value * cache);
                        }
                    }
                }
            }
        }

        return result;
    }

    public BitFlatArrayArbitraryMatrix multiplyResultBit(SparseArbitraryMatrix a, SparseArbitraryMatrix b) {
        int width = b.getWidth();
        int height = a.getHeight();
        BitFlatArrayArbitraryMatrix result = new BitFlatArrayArbitraryMatrix(width, height);

        for (SparseBlock aSparse : a.getSparseBlocks()) {
            for (SparseBlock bSparse : b.getSparseBlocks()) {
                int startI = Math.max(aSparse.getStartX()-bSparse.getStartY(), 0)-aSparse.getStartX()+bSparse.getStartY();
                int endI = Math.min(aSparse.getStartX()+aSparse.getWidth()-bSparse.getStartY(), bSparse.getHeight())-aSparse.getStartX()+bSparse.getStartY();
                /*
                Equivalent:
                if (currentA<0 || currentA>=bSparse.getHeight()) {
                        continue;
                    }
                 */
                for (int i = startI; i < endI; i++) {
                    int currentA = i+aSparse.getStartX()-bSparse.getStartY();
                    for (int x = 0; x < bSparse.getWidth(); x++) {
                        float cache = bSparse.getLocal(x, currentA);
                        if (Maths.fastAbs(cache)<EPSILON)
                            continue;
                        for (int y = 0; y < aSparse.getHeight(); y++) {
                            float value = aSparse.getLocal(i, y);
                            if (Maths.fastAbs(value)<EPSILON)
                                continue;
                            result.add(x+bSparse.getStartX(), y+aSparse.getStartY(), value * cache);
                        }
                    }
                }
            }
        }

        result.calcSet();

        return result;
    }

    @Override
    public FlatArrayArbitraryMatrix multiplySwitched(SparseArbitraryMatrix sparseArbitraryMatrix, SparseArbitraryMatrix sparseArbitraryMatrix2) {
        return null;
    }

    @Override
    public FlatArrayArbitraryMatrix multiplyWithATransposed(SparseArbitraryMatrix sparseArbitraryMatrix, SparseArbitraryMatrix sparseArbitraryMatrix2) {
        return null;
    }

    @Override
    public FlatArrayArbitraryMatrix multiplyWithBTransposedSwitched(SparseArbitraryMatrix sparseArbitraryMatrix, SparseArbitraryMatrix sparseArbitraryMatrix2) {
        return null;
    }

    @Override
    public FlatArrayArbitraryMatrix multiplyWithBTransposed(SparseArbitraryMatrix sparseArbitraryMatrix, SparseArbitraryMatrix sparseArbitraryMatrix2) {
        return null;
    }

    @Override
    public FlatArrayArbitraryMatrix multiplyWithATransposedSwitched(SparseArbitraryMatrix sparseArbitraryMatrix, SparseArbitraryMatrix sparseArbitraryMatrix2) {
        return null;
    }

}
