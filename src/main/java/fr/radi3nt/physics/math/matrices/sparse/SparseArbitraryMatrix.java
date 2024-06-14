package fr.radi3nt.physics.math.matrices.sparse;

import fr.radi3nt.maths.components.arbitrary.VectorNf;
import fr.radi3nt.maths.components.arbitrary.vector.ArrayVectorNf;
import fr.radi3nt.physics.math.ArbitraryMatrix;

import java.util.ArrayList;
import java.util.Collection;

public class SparseArbitraryMatrix implements ArbitraryMatrix {

    private final Collection<SparseBlock> sparseBlocks = new ArrayList<>();
    private int width = 0;
    private int height = 0;

    public void add(SparseBlock sparseBlock) {
        width = Math.max(sparseBlock.getWidth()+sparseBlock.getStartX(), width);
        height = Math.max(sparseBlock.getHeight()+sparseBlock.getStartY(), height);
        sparseBlocks.add(sparseBlock);
    }

    @Override
    public float get(int x, int y) {
        for (SparseBlock sparseBlock : sparseBlocks) {
            if (sparseBlock.isInBound(x, y)) {
                return sparseBlock.get(x, y);
            }
        }
        return 0;
    }

    @Override
    public VectorNf transform(VectorNf vector) {
        VectorNf result = new ArrayVectorNf(height);
        for (SparseBlock sparseBlock : sparseBlocks) {
            for (int y = sparseBlock.getStartY(); y < sparseBlock.getStartY() + sparseBlock.getHeight(); y++) {
                for (int x = sparseBlock.getStartX(); x < sparseBlock.getStartX() + sparseBlock.getWidth(); x++) {
                    float value = sparseBlock.get(x, y);
                    result.add(y, value * vector.get(x));
                }
            }
        }
        return result;
    }

    @Override
    public VectorNf transformTransposed(VectorNf vector) {
        VectorNf result = new ArrayVectorNf(width);
        for (SparseBlock sparseBlock : sparseBlocks) {
            for (int y = sparseBlock.getStartY(); y < sparseBlock.getStartY() + sparseBlock.getHeight(); y++) {
                for (int x = sparseBlock.getStartX(); x < sparseBlock.getStartX() + sparseBlock.getWidth(); x++) {
                    float value = sparseBlock.get(x, y);
                    result.add(x, value * vector.get(y));
                }
            }
        }
        return result;
    }

    public Collection<SparseBlock> getSparseBlocks() {
        return sparseBlocks;
    }

    @Override
    public int getWidth() {
        return width;
    }

    @Override
    public int getHeight() {
        return height;
    }
}
