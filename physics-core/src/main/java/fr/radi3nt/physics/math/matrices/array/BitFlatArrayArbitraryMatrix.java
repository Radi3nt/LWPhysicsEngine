package fr.radi3nt.physics.math.matrices.array;

import java.util.BitSet;

public class BitFlatArrayArbitraryMatrix extends FlatArrayArbitraryMatrix {

    private final BitSet setElements = new BitSet();

    public BitFlatArrayArbitraryMatrix(int width, int height) {
        super(width, height);
    }

    public int next(int x, int y) {
        int index = setElements.nextSetBit(x*getHeight()+(y));
        if (index>=(x+1)*getHeight())
            return -1;
        int realY = index%getHeight();
        return realY;
    }

    public void calcSet() {
        setElements.clear();
        for (int i = 0, mLength = m.length; i < mLength; i++) {
            float v = m[i];
            if (v != 0) {
                int x = i%width;
                int y = i/width;
                setElements.set(y+x*getHeight());
            }
        }
    }
}
