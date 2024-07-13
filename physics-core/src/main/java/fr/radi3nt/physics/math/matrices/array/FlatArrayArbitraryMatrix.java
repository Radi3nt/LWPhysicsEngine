package fr.radi3nt.physics.math.matrices.array;

import fr.radi3nt.maths.components.arbitrary.VectorNf;
import fr.radi3nt.maths.components.arbitrary.vector.ArrayVectorNf;

import java.util.Arrays;

public class FlatArrayArbitraryMatrix implements ArrayArbitraryMatrix {

    protected final float[] m;
    protected final int width;
    private final int height;

    public FlatArrayArbitraryMatrix(int width, int height) {
        this.width = width;
        this.height = height;
        this.m = new float[width*height];
    }

    @Override
    public void set(int x, int y, float set) {
        m[x+y*width] = set;
    }

    @Override
    public void add(int x, int y, float value) {
        m[x+y*width] += value;
    }

    @Override
    public float get(int x, int y) {
        return m[x+y*width];
    }

    @Override
    public VectorNf transform(VectorNf vector) {
        if (vector.size()!=height)
            throw new IllegalArgumentException("Vector isn't of the required size (" + vector.size() + ") compared to matrix of height " + height);

        VectorNf result = new ArrayVectorNf(height);

        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                float value = get(x, y) * vector.get(x);
                result.add(y, value);
            }
        }

        return result;
    }

    @Override
    public VectorNf transformTransposed(VectorNf vector) {
        if (vector.size()!=width)
            throw new IllegalArgumentException("Vector isn't of the required size (" + vector.size() + ") compared to matrix of width " + width);

        VectorNf result = new ArrayVectorNf(width);
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                float value = get(x, y) * vector.get(y);
                result.add(x, value);
            }
        }
        return result;
    }

    @Override
    public int getWidth() {
        return width;
    }

    @Override
    public int getHeight() {
        return height;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (!(o instanceof FlatArrayArbitraryMatrix)) return false;

        FlatArrayArbitraryMatrix that = (FlatArrayArbitraryMatrix) o;

        if (width != that.width) return false;
        if (height != that.height) return false;
        return Arrays.equals(m, that.m);
    }

    @Override
    public int hashCode() {
        int result = Arrays.hashCode(m);
        result = 31 * result + width;
        result = 31 * result + height;
        return result;
    }

    public int zeroElements() {
        int elements = 0;
        for (float v : m) {
            if (v==0)
                elements++;
        }
        return elements;
    }
}
