package fr.radi3nt.physics.math.matrices.sparse;

public class SparseBlock {

    private final int startX;
    private final int startY;
    private final int width;
    private final int height;

    private final float[] values;

    public SparseBlock(int startX, int startY, int width, int height, float[] values) {
        this.startX = startX;
        this.startY = startY;
        this.width = width;
        this.height = height;
        this.values = values;
    }

    public float get(int i, int j) {
        return values[(i - startX) + (j - startY) * width];
    }

    public float getLocal(int x, int y) {
        return values[x + y * width];
    }

    public int getStartX() {
        return startX;
    }

    public int getStartY() {
        return startY;
    }

    public int getWidth() {
        return width;
    }

    public int getHeight() {
        return height;
    }

    public boolean isInBound(int x, int y) {
        return isInBoundX(x) && isInBoundY(y);
    }

    protected boolean isInBoundY(int y) {
        return y >= startY && y < startY + height;
    }

    protected boolean isInBoundX(int x) {
        return x >= startX && x < startX + width;
    }
}
