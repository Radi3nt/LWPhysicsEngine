package fr.radi3nt.physics.math.matrices.sparse;

import fr.radi3nt.maths.components.vectors.Vector2i;
import fr.radi3nt.maths.components.vectors.implementations.SimpleVector2i;

import java.util.BitSet;
import java.util.HashMap;
import java.util.Map;

public class HashSparseArbitraryMatrix extends SparseArbitraryMatrix {

    private final Map<Vector2i, SparseBlock> indices = new HashMap<>();
    private final BitSet set = new BitSet();

    public void add(SparseBlock sparseBlock) {
        super.add(sparseBlock);
        for (int x = 0; x < sparseBlock.getWidth(); x++) {
            for (int y = 0; y < sparseBlock.getHeight(); y++) {
                indices.put(new SimpleVector2i(x+sparseBlock.getStartX(), y+sparseBlock.getStartY()), sparseBlock);
            }
        }
    }

    public void calcSet() {
        for (SparseBlock sparseBlock : getSparseBlocks()) {
            for (int y = 0; y < sparseBlock.getHeight(); y++) {
                set.set(sparseBlock.getStartX()+(y+sparseBlock.getStartY())*getWidth(), sparseBlock.getStartX()+sparseBlock.getWidth()+(y+sparseBlock.getStartY())*getWidth());
            }
        }
    }

    @Override
    public float get(int x, int y) {
        if (!set.get(x+y*getWidth()))
            return 0;
        SparseBlock sparseBlock = indices.get(new SimpleVector2i(x, y));
        if (sparseBlock==null)
            return 0;
        return sparseBlock.get(x, y);
    }
}
