package fr.radi3nt.physics.collision.detection.narrow.algorithms.gjk;

import java.util.Iterator;

public class PointBag implements Iterable<GjkPoint>, Cloneable {

    private final GjkPoint[] points = new GjkPoint[4];
    private int size;

    public void addAll(GjkPoint... points) {
        System.arraycopy(points, 0, this.points, 0, points.length);
        size+=points.length;
    }

    public void insertAsFirstElement(GjkPoint point) {
        size++;

        GjkPoint lastPoint = point;
        for (int i = 0; i < size; i++) {
            GjkPoint newLastPoint = points[i];
            points[i] = lastPoint;
            lastPoint = newLastPoint;
        }
    }

    public void clear() {
        size = 0;
    }

    public int size() {
        return size;
    }

    public GjkPoint get(int i) {
        return points[i];
    }

    @Override
    public Iterator<GjkPoint> iterator() {
        return new Iterator<GjkPoint>() {

            int i = 0;

            @Override
            public boolean hasNext() {
                return i<size;
            }

            @Override
            public GjkPoint next() {
                return points[i++];
            }
        };
    }

    @Override
    public PointBag clone() {
        try {
            PointBag clone = (PointBag) super.clone();
            return clone;
        } catch (CloneNotSupportedException e) {
            throw new AssertionError();
        }
    }
}
