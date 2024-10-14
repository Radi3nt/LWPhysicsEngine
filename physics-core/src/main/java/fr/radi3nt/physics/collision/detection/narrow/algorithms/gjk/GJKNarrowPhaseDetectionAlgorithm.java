package fr.radi3nt.physics.collision.detection.narrow.algorithms.gjk;

import fr.radi3nt.maths.Maths;
import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.maths.components.vectors.implementations.SimpleVector3f;
import fr.radi3nt.physics.collision.contact.GeneratedContactPair;
import fr.radi3nt.physics.collision.contact.cache.PersistentManifoldCache;
import fr.radi3nt.physics.collision.contact.manifold.ManifoldPoint;
import fr.radi3nt.physics.collision.contact.manifold.PersistentManifold;
import fr.radi3nt.physics.collision.contact.manifold.manifolds.gjk.GJKManifoldPoint;
import fr.radi3nt.physics.collision.contact.manifold.manifolds.gjk.GjkIndexedPoint;
import fr.radi3nt.physics.collision.detection.narrow.NarrowPhaseDetectionAlgorithm;
import fr.radi3nt.physics.collision.detection.narrow.algorithms.gjk.shapes.GjkProcessedShape;
import fr.radi3nt.physics.collision.detection.narrow.manifold.RegularManifoldComputer;
import fr.radi3nt.physics.collision.detection.narrow.processed.ProcessedShapeProvider;
import fr.radi3nt.physics.collision.shape.shapes.CollisionShape;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.List;

public class GJKNarrowPhaseDetectionAlgorithm implements NarrowPhaseDetectionAlgorithm {

    private final GJKDCollisionGenerator generator = new GJKDCollisionGenerator();
    private final RegularManifoldComputer regularManifoldComputer;
    private final ProcessedShapeProvider<GjkProcessedShape> gjkShapeProcessedShapeProvider;

    public GJKNarrowPhaseDetectionAlgorithm(ProcessedShapeProvider<GjkProcessedShape> gjkShapeProcessedShapeProvider) {
        this.gjkShapeProcessedShapeProvider = gjkShapeProcessedShapeProvider;
        regularManifoldComputer = new RegularManifoldComputer();
    }

    @Override
    public PersistentManifold buildManifolds(PersistentManifoldCache manifoldCache, GeneratedContactPair pairs, long currentStep) {
        CollisionShape shapeA = pairs.shapeA;
        CollisionShape shapeB = pairs.shapeB;

        GjkProcessedShape processedShapeA = gjkShapeProcessedShapeProvider.getShape(shapeA, pairs.objectA);
        GjkProcessedShape processedShapeB = gjkShapeProcessedShapeProvider.getShape(shapeB, pairs.objectB);

        GJKDCollisionGenerator.CollisionResult result = generator.test(processedShapeA, processedShapeB);
        if (result.collided)
            return null;
        if (result.closestDistance > 0) {
            return null;
        }

        Collection<ManifoldPoint> manifoldPoints = new ArrayList<>();

        if (result.collidingSimplex.isPoint()) {
            pointCase(pairs, result, processedShapeA, processedShapeB, manifoldPoints);
        }
        if (result.collidingSimplex.isLine()) {
            lineCase(pairs, result, processedShapeA, processedShapeB, manifoldPoints);
        }
        if (result.collidingSimplex.isTriangle()) {
            PointBag points = result.collidingSimplex.getPoints();
            GjkPoint point1 = points.get(0);
            GjkPoint point2 = points.get(1);
            GjkPoint point3 = points.get(2);
            boolean isPointFaceA = point1.getBuildA().equals(point2.getBuildA()) && point1.getBuildA().equals(point3.getBuildA());
            boolean isPointFaceB = point1.getBuildB().equals(point2.getBuildB()) && point1.getBuildB().equals(point3.getBuildB());

            boolean pointFaceCase = isPointFaceA || isPointFaceB;
            if (!pointFaceCase) {

                boolean isSameAP1P2 = point1.getBuildA().equals(point2.getBuildA());
                boolean isSameAP2P3 = point2.getBuildA().equals(point3.getBuildA());
                boolean isSameAP1P3 = point1.getBuildA().equals(point3.getBuildA());

                boolean isSameBP1P2 = point1.getBuildB().equals(point2.getBuildB());
                boolean isSameBP2P3 = point2.getBuildB().equals(point3.getBuildB());
                boolean isSameBP1P3 = point1.getBuildB().equals(point3.getBuildB());

                boolean someAreSameOnA = (isSameAP1P2 || isSameAP2P3 || isSameAP1P3);
                boolean someAreSameOnB = (isSameBP1P2 || isSameBP2P3 || isSameBP1P3);
                boolean edgeEdgeCase = someAreSameOnA && someAreSameOnB;

                if (edgeEdgeCase) {
                    //edge edge case
                    Vector3f ap1 = point1.getBuildA();
                    Vector3f ap2 = isSameAP1P2 ? point3.getBuildA() : point2.getBuildA();

                    Vector3f bp1 = point1.getBuildB();
                    Vector3f bp2 = isSameBP1P2 ? point3.getBuildB() : point2.getBuildB();

                    Vector3f d1 = ap2.duplicate().sub(ap1);
                    Vector3f d2 = bp2.duplicate().sub(bp1);

                    Vector3f pp1 = ap1;
                    Vector3f pp2 = bp1;

                    Vector3f closestPointA = new SimpleVector3f();
                    Vector3f closestPointB = new SimpleVector3f();
                    closestPointOnEdges(pp2, pp1, d1, d2, closestPointA, closestPointB);
                    correctPoints(pairs, result, processedShapeA, processedShapeB, manifoldPoints, closestPointA, closestPointB);
                } else {
                    //tri edge case
                    boolean isEdgeOnA = someAreSameOnA;

                    Vector3f triangleVert1 = isEdgeOnA ? point1.getBuildB() : point1.getBuildA();
                    Vector3f triangleVert2 = isEdgeOnA ? point2.getBuildB() : point2.getBuildA();
                    Vector3f triangleVert3 = isEdgeOnA ? point3.getBuildB() : point3.getBuildA();

                    Vector3f edgePoint1 = isEdgeOnA ? point1.getBuildA() : point1.getBuildB();
                    Vector3f edgePoint2 = isEdgeOnA ? (isSameAP1P2 ? point3.getBuildA() : point2.getBuildA()) : (isSameBP1P2 ? point3.getBuildB() : point2.getBuildB());
                    Vector3f edgeNormal = edgePoint2.duplicate().sub(edgePoint1);


                    float minTri = Float.MAX_VALUE;
                    float maxTri = -Float.MAX_VALUE;

                    minTri = Math.min(minTri, edgeNormal.dot(triangleVert1));
                    minTri = Math.min(minTri, edgeNormal.dot(triangleVert2));
                    minTri = Math.min(minTri, edgeNormal.dot(triangleVert3));

                    maxTri = Math.max(maxTri, edgeNormal.dot(triangleVert1));
                    maxTri = Math.max(maxTri, edgeNormal.dot(triangleVert2));
                    maxTri = Math.max(maxTri, edgeNormal.dot(triangleVert3));

                    float edge1Dot = edgeNormal.dot(edgePoint1);
                    float edge2Dot = edgeNormal.dot(edgePoint2);

                    float lowerLimit = Math.max(minTri, edge1Dot);
                    float upperLimit = Math.min(maxTri, edge2Dot);

                    float midPoint = (upperLimit+lowerLimit)/2-edge1Dot;

                    float edgeLengthSquared = edgeNormal.lengthSquared();

                    Vector3f pointOnEdge = edgeNormal.duplicate().mul(midPoint/edgeLengthSquared).add(edgePoint1);
                    Vector3f faceNormal = computeFaceNormal(triangleVert1, triangleVert2, triangleVert3);

                    pointTriangleCase(pairs, isEdgeOnA, processedShapeA, processedShapeB, pointOnEdge, faceNormal, triangleVert1, manifoldPoints, points);
                }

            } else {
                Vector3f pointOnFace = isPointFaceA ? point1.getBuildA() : point1.getBuildB();
                Vector3f firstPointTri = isPointFaceA ? point1.getBuildB() : point1.getBuildA();
                Vector3f faceNormal = computeFaceNormal(firstPointTri, isPointFaceA ? point2.getBuildB() : point2.getBuildA(), isPointFaceA ? point3.getBuildB() : point3.getBuildA());

                pointTriangleCase(pairs, isPointFaceA, processedShapeA, processedShapeB, pointOnFace, faceNormal, firstPointTri, manifoldPoints, points);
            }
        }


        return regularManifoldComputer.compute(manifoldCache, pairs, manifoldPoints, currentStep);
    }

    private static void pointTriangleCase(GeneratedContactPair pairs, boolean isPointFaceA, GjkProcessedShape processedShapeA, GjkProcessedShape processedShapeB, Vector3f pointOnFace, Vector3f faceNormal, Vector3f firstPointTri, Collection<ManifoldPoint> manifoldPoints, PointBag points) {
        float realDistance = (isPointFaceA ? -processedShapeA.transformDistance(0) : processedShapeB.transformDistance(0));
        Vector3f realPoint = pointOnFace.duplicate().add(faceNormal.duplicate().mul(realDistance));

        float pointDot = realPoint.dot(faceNormal);
        float faceDot = firstPointTri.dot(faceNormal);

        float otherDistance = faceDot - pointDot;
        otherDistance = (!isPointFaceA ? -processedShapeA.transformDistance(-otherDistance) : processedShapeB.transformDistance(otherDistance));

        Vector3f otherPoint = realPoint.duplicate().add(faceNormal.duplicate().mul(otherDistance));

        Vector3f closestPointA = isPointFaceA ? realPoint : otherPoint;
        Vector3f closestPointB = isPointFaceA ? otherPoint : realPoint;

        addManifold(manifoldPoints, pairs, closestPointA, closestPointB, faceNormal, buildIndices(points));
    }

    private static void pointCase(GeneratedContactPair pairs, GJKDCollisionGenerator.CollisionResult result, GjkProcessedShape processedShapeA, GjkProcessedShape processedShapeB, Collection<ManifoldPoint> manifoldPoints) {
        GjkPoint solePoint = result.collidingSimplex.getPoints().get(0);
        Vector3f a = solePoint.getBuildA();
        Vector3f b = solePoint.getBuildB();
        correctPoints(pairs, result, processedShapeA, processedShapeB, manifoldPoints, a, b);
    }

    private static void correctPoints(GeneratedContactPair pairs, GJKDCollisionGenerator.CollisionResult result, GjkProcessedShape processedShapeA, GjkProcessedShape processedShapeB, Collection<ManifoldPoint> manifoldPoints, Vector3f a, Vector3f b) {
        float distA = -processedShapeA.transformDistance(0);
        float distB = processedShapeB.transformDistance(0);

        Vector3f normal = b.duplicate().sub(a).normalize();

        Vector3f aCorrected = a.duplicate().add(normal.duplicate().mul(distA));
        Vector3f bCorrected = b.duplicate().add(normal.duplicate().mul(distB));

        addManifold(manifoldPoints, pairs, aCorrected, bCorrected, normal, buildIndices(result.collidingSimplex.getPoints()));
    }

    private static void addManifold(Collection<ManifoldPoint> manifoldPoints, GeneratedContactPair pairs, Vector3f aCorrected, Vector3f bCorrected, Vector3f normal, GjkIndexedPoint[] collidingSimplex) {
        manifoldPoints.add(new GJKManifoldPoint(pairs.objectA.toLocalSpace(aCorrected), pairs.objectB.toLocalSpace(bCorrected), normal, collidingSimplex));
    }

    private static GjkIndexedPoint[] buildIndices(PointBag points) {

        List<GjkIndexedPoint> gjkPoints = new ArrayList<>();
        for (GjkPoint point : points) {
            gjkPoints.add(point.getIndex());
        }
        Collections.sort(gjkPoints);
        return gjkPoints.toArray(new GjkIndexedPoint[0]);
    }

    private Vector3f computeFaceNormal(Vector3f first, Vector3f second, Vector3f third) {
        Vector3f normal = new SimpleVector3f();
        normal.setX(second.getX());
        normal.setY(second.getY());
        normal.setZ(second.getZ());

        final Vector3f tangentA = normal.sub(first);

        float pos3X = third.getX() - first.getX();
        float pos3Y = third.getY() - first.getY();
        float pos3Z = third.getZ() - first.getZ();

        float x = tangentA.getY() * pos3Z - tangentA.getZ() * pos3Y;
        float y = tangentA.getZ() * pos3X - tangentA.getX() * pos3Z;
        float z = tangentA.getX() * pos3Y - tangentA.getY() * pos3X;

        normal.setX(x);
        normal.setY(y);
        normal.setZ(z);
        normal.normalize();

        return normal;
    }

    private static void lineCase(GeneratedContactPair pairs, GJKDCollisionGenerator.CollisionResult result, GjkProcessedShape processedShapeA, GjkProcessedShape processedShapeB, Collection<ManifoldPoint> manifoldPoints) {
        GJKSimplex simplex = result.collidingSimplex;

        GjkPoint p1 = simplex.getPoints().get(0);
        GjkPoint p2 = simplex.getPoints().get(1);

        boolean aIsPoint = p2.getBuildA().equals(p1.getBuildA());
        boolean bIsPoint = p2.getBuildB().equals(p1.getBuildB());
        boolean linePoint = aIsPoint || bIsPoint;

        Vector3f closestPointA = new SimpleVector3f();
        Vector3f closestPointB = new SimpleVector3f();
        if (linePoint) {
            linePointCase(aIsPoint, closestPointA, p1, closestPointB, p2);
        } else {
            Vector3f d1 = p2.getBuildA().duplicate().sub(p1.getBuildA());
            Vector3f d2 = p2.getBuildB().duplicate().sub(p1.getBuildB());

            Vector3f pp1 = p1.getBuildA();
            Vector3f pp2 = p1.getBuildB();

            closestPointOnEdges(pp2, pp1, d1, d2, closestPointA, closestPointB);
        }
        Vector3f normal = closestPointB.duplicate().sub(closestPointA).normalize();
        float distA = -processedShapeA.transformDistance(0);
        float distB = processedShapeB.transformDistance(0);
        closestPointA.add(normal.duplicate().mul(distA));
        closestPointB.add(normal.duplicate().mul(distB));

        GjkIndexedPoint[] indices = buildIndices(result.collidingSimplex.getPoints());

        addManifold(manifoldPoints, pairs, closestPointA, closestPointB, normal, indices);
    }

    private static void closestPointOnEdges(Vector3f pp2, Vector3f pp1, Vector3f d1, Vector3f d2, Vector3f closestPointA, Vector3f closestPointB) {
        Vector3f p2SubP1 = pp2.duplicate().sub(pp1);
        Vector3f p1SubP2 = pp1.duplicate().sub(pp2);

        Vector3f n = d1.duplicate().cross(d2);

        if (n.lengthSquared()==0) {
            Vector3f qA = d1.duplicate().mul(0.5f).add(pp1);
            Vector3f qB;
            float dot = (d2.dot(qA)-d2.dot(pp2))/d2.lengthSquared();
            if (dot>1 || dot<0) {
                qB = d2.duplicate().mul(0.5f).add(pp2);
                float dot2 = (d1.dot(qB)-d1.dot(pp1))/d1.lengthSquared();
                qA = d1.duplicate().mul(dot2).add(pp1);
            } else {
                qB = d2.duplicate().mul(dot).add(pp2);
            }

            closestPointA.copy(qA);
            closestPointB.copy(qB);
            return;
        }

        Vector3f n1 = d1.duplicate().cross(n);
        Vector3f n2 = d2.duplicate().cross(n);

        float t1 = n2.dot(p2SubP1)/(n2.dot(d1));
        float t2 = n1.dot(p1SubP2)/(n1.dot(d2));

        t1 = Maths.clamp(t1, 0, 1);
        t2 = Maths.clamp(t2, 0, 1);

        // https://en.wikipedia.org/wiki/Skew_lines#Nearest_Points

        Vector3f qA = d1.duplicate().mul(t1).add(pp1);
        Vector3f qB = d2.duplicate().mul(t2).add(pp2);

        closestPointA.copy(qA);
        closestPointB.copy(qB);
    }

    private static void linePointCase(boolean aIsPoint, Vector3f closestPointA, GjkPoint p1, Vector3f closestPointB, GjkPoint p2) {
        if (aIsPoint)
            closestPointA.copy(p1.getBuildA());
        else
            closestPointB.copy(p1.getBuildB());
        Vector3f edgeStartPoint = aIsPoint ? p1.getBuildB() : p1.getBuildA();
        Vector3f edgeNormal = aIsPoint ? p2.getBuildB().duplicate().sub(p1.getBuildB()) : p2.getBuildA().duplicate().sub(p1.getBuildA());

        float length = edgeNormal.lengthSquared();

        float pointOnLine = edgeNormal.dot(aIsPoint ? p1.getBuildA() : p1.getBuildB());
        float startPointOnLine = edgeNormal.dot(edgeStartPoint);
        float percent = (pointOnLine-startPointOnLine)/length;

        Vector3f selectedPointInverse = aIsPoint ? closestPointB : closestPointA;

        percent = Maths.clamp(percent, 0, 1);
        selectedPointInverse.copy(edgeNormal);
        selectedPointInverse.mul(percent).add(edgeStartPoint);
    }

    @Override
    public boolean isSupported(CollisionShape collisionShape) {
        return gjkShapeProcessedShapeProvider.isSupported(collisionShape);
    }
}
