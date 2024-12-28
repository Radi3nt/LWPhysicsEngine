package fr.radi3nt.physics.collision.detection.narrow.algorithms.gjk;

import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.physics.collision.contact.manifold.manifolds.gjk.GjkIndexedPoint;
import fr.radi3nt.physics.collision.detection.narrow.algorithms.gjk.shapes.GjkProcessedShape;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

public class GjkPoint {

    private final Vector3f point;

    private final Vector3f buildA;
    private final Vector3f buildB;

    private final GjkIndexedPoint indexedPoint;

    public GjkPoint(Vector3f point, Vector3f buildA, Vector3f buildB, GjkIndexedPoint indexedPoint) {
        this.point = point;
        this.buildA = buildA;
        this.buildB = buildB;
        this.indexedPoint = indexedPoint;
    }

    public static GjkPoint from(Vector3f buildA, Vector3f buildB, int a, int b) {
        Vector3f point = buildMinkowskiPoint(buildA, buildB);
        return new GjkPoint(point, buildA, buildB, new GjkIndexedPoint(a, b));
    }

    private static Vector3f buildMinkowskiPoint(Vector3f buildA, Vector3f buildB) {
        Vector3f point = buildA.duplicate().sub(buildB);
        return point;
    }

    public static GjkPoint from(List<GjkProcessedShape.FurthestPoint> furthestA, List<GjkProcessedShape.FurthestPoint> furthestB, Vector3f direction) {
        List<GjkProcessedShape.FurthestPoint[]> pointsGroup = new ArrayList<>();
        for (GjkProcessedShape.FurthestPoint furthestPointA : furthestA) {
            for (GjkProcessedShape.FurthestPoint furthestPointB : furthestB) {
                pointsGroup.add(new GjkProcessedShape.FurthestPoint[]{furthestPointA, furthestPointB});
            }
        }

        if (pointsGroup.isEmpty()) {
            return null;
        }

        return from(pointsGroup, direction);
    }

    private static GjkPoint from(List<GjkProcessedShape.FurthestPoint[]> pointsGroup, Vector3f direction) {
        GjkProcessedShape.FurthestPoint[] furthestGroup = pointsGroup.get(0);
        GjkProcessedShape.FurthestPoint aBest = furthestGroup[0];
        GjkProcessedShape.FurthestPoint bBest = furthestGroup[1];
        float dist = aBest.point.duplicate().sub(bBest.point).lengthSquared();

        for (int i = 1; i < pointsGroup.size(); i++) {
            GjkProcessedShape.FurthestPoint[] currentGroup = pointsGroup.get(i);
            GjkProcessedShape.FurthestPoint currentA = currentGroup[0];
            GjkProcessedShape.FurthestPoint currentB = currentGroup[1];

            float currentDist = currentB.point.duplicate().sub(currentA.point).lengthSquared();
            if (currentDist<dist) {
                dist = currentDist;
                aBest = currentA;
                bBest = currentB;
            }
        }

        return from(aBest, bBest);
    }

    private static GjkPoint from(GjkProcessedShape.FurthestPoint aBest, GjkProcessedShape.FurthestPoint bBest) {
        return from(aBest.point, bBest.point, aBest.index, bBest.index);
    }

    public Vector3f getPoint() {
        return point;
    }

    public Vector3f getBuildA() {
        return buildA;
    }

    public Vector3f getBuildB() {
        return buildB;
    }

    @Override
    public final boolean equals(Object o) {
        if (this == o) return true;
        if (!(o instanceof GjkPoint)) return false;

        GjkPoint gjkPoint = (GjkPoint) o;
        return Objects.equals(indexedPoint, gjkPoint.indexedPoint);
    }

    @Override
    public int hashCode() {
        return Objects.hashCode(indexedPoint);
    }

    public GjkIndexedPoint getIndex() {
        return indexedPoint;
    }
}
