package fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.projection;

import fr.radi3nt.physics.collision.detection.broad.aabb.aabb.mapping.ProjectionSettable;
import fr.radi3nt.maths.components.vectors.Vector3f;

import static java.lang.Math.abs;

public class SatProjectedObject implements ProjectionSettable {

    private float min;
    private float max;

    public SatProjectedObject(float min, float max) {
        this.min = min;
        this.max = max;
    }

    public float getEnterTime(float point, float speed) {
        return Math.min((max - point)/speed, (min - point)/speed);
    }

    public float getLeaveTime(float point, float speed) {
        return Math.max((max - point)/speed, (min - point)/speed);
    }

    public int getNormal(float point) {
        if (point>=max)
            return 1;
        if (point<=min)
            return -1;
        return 0;
    }

    public void set(float min, float max) {
        this.min = min;
        this.max = max;
    }

    @Override
    public void setMin(float min) {
        this.min = min;
    }

    @Override
    public void setMax(float max) {
        this.max = max;
    }

    public SatProjectedObject project(float min, float max) {
        this.min = min;
        this.max = max;
        return this;
    }

    public static SatProjectedObject project(Vector3f[] vertices, Vector3f axis) {
        float min = axis.dot(vertices[0]);
        float max = min;
        for (int i = 1; i < vertices.length; i++) {
            float p = axis.dot(vertices[i]);
            min = Math.min(p, min);
            max = Math.max(p, max);
        }

        return new SatProjectedObject(min, max);
    }

    public SatProjectedObject projectReplace(Vector3f[] vertices, Vector3f axis) {
        float min = axis.dot(vertices[0]);
        float max = min;
        for (int i = 1; i < vertices.length; i++) {
            float p = axis.dot(vertices[i]);
            min = Math.min(p, min);
            max = Math.max(p, max);
        }

        this.min = min;
        this.max = max;

        return this;
    }

    public boolean contains(SatProjectedObject p2) {
        return (min <= p2.min && max >= p2.max);
    }

    public static int getOverlapNormal(SatProjectedObject p1, SatProjectedObject p2) {
        if (p2.contains(p1))
            return abs(p2.min - p1.min) > abs(p1.max - p2.max) ? -1 : 1;
        if (p1.contains(p2))
            return abs(p2.min - p1.min) > abs(p1.max - p2.max) ? 1 : -1;
        if (p1.min <= p2.min) {
            if (p1.max >= p2.min)
                return 1;
        } else {
            if (p2.max >= p1.min)
                return -1;
        }
        return 0;
    }

    public static float getShortestDistance(SatProjectedObject p1, SatProjectedObject p2) {
        float overlap = getIntersectingOverlap(p1, p2);
        if (overlap==Float.NEGATIVE_INFINITY) {
            return Math.min(abs(p2.min-p1.max), abs(p1.min-p2.max));
        }

        if (isContainment(p1, p2)) {
            overlap += getContainmentOverlap(p1, p2);
        }
        return -overlap;
    }

    public static float getOverlap(SatProjectedObject p1, SatProjectedObject p2) {
        float overlap = getIntersectingOverlap(p1, p2);
        if (isContainment(p1, p2)) {
            overlap += getContainmentOverlap(p1, p2);
        }
        return overlap;
    }

    private static boolean isContainment(SatProjectedObject p1, SatProjectedObject p2) {
        return p1.contains(p2) || p2.contains(p1);
    }

    private static float getIntersectingOverlap(SatProjectedObject p1, SatProjectedObject p2) {
        if (p1.min <= p2.min) {
            if (p1.max >= p2.min)
                return Math.min(abs(p2.min - p1.max), abs(p2.max-p2.min));
        } else {
            if (p1.min <= p2.max)
                return Math.min(abs(p1.min - p2.max), abs(p1.max-p1.min));
        }
        return Float.NEGATIVE_INFINITY;
    }

    public static float getContainmentOverlap(SatProjectedObject p1, SatProjectedObject p2) {
        float min = abs(p1.min - p2.min);
        float max = abs(p1.max - p2.max);
        return Math.min(min, max);
    }

    public static float getNormalPointingContainmentOverlap(SatProjectedObject p1, SatProjectedObject p2) {
        return abs(p1.max - p2.max);
    }

}
