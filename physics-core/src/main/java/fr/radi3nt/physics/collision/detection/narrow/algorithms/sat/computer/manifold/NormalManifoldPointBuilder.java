package fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.computer.manifold;

import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.maths.components.vectors.implementations.SimpleVector3f;
import fr.radi3nt.physics.collision.contact.GeneratedContactPair;
import fr.radi3nt.physics.collision.contact.manifold.ManifoldPoint;
import fr.radi3nt.physics.collision.contact.manifold.PointIndex;
import fr.radi3nt.physics.collision.contact.manifold.manifolds.clip.ClipManifoldPoint;
import fr.radi3nt.physics.collision.contact.manifold.manifolds.edges.EdgeManifoldPoint;
import fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.computer.NormalSatCollisionDetector;
import fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.clip.AbstractClippingSurface;
import fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.clip.ClipPlane;
import fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.clip.ClipPlanes;
import fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.clip.Edge;
import fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.info.SatEdge;
import fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.info.SatFace;
import fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.shape.SatProcessedShape;
import fr.radi3nt.physics.core.TransformedObject;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.List;

import static java.lang.Math.abs;

public class NormalManifoldPointBuilder implements ManifoldPointBuilder {

    private static final float MERGE_THRESHOLD = 2e-2f;
    private static final float EPSILON = 1e-1f;


    public NormalManifoldPointBuilder() {
    }

    public List<ManifoldPoint> createContactPoints(GeneratedContactPair pair, SatProcessedShape sa, SatProcessedShape sb, NormalSatCollisionDetector.ResultInfo resultInfo) {
        List<ManifoldPoint> manifoldPoints = computePairs(pair, sa, sb, resultInfo);
        removeDuplicatedContactsPoints(manifoldPoints, pair, resultInfo.worldSpaceNormal);
        return manifoldPoints;
    }

    private List<ManifoldPoint> computePairs(GeneratedContactPair pair, SatProcessedShape sa, SatProcessedShape sb, NormalSatCollisionDetector.ResultInfo resultInfo) {
        List<ManifoldPoint> resultPoints = runFaces(pair, sa, sb, resultInfo);
        if (resultPoints != null) return resultPoints;

        resultPoints = runEdges(pair, resultInfo);
        if (resultPoints != null) return resultPoints;

        return Collections.emptyList();
    }

    private List<ManifoldPoint> runEdges(GeneratedContactPair pair, NormalSatCollisionDetector.ResultInfo resultInfo) {
        if (resultInfo.contactType.isFace()) {
            return null;
        }

        List<ManifoldPoint> resultPoints = new ArrayList<>();

        for (NormalSatCollisionDetector.EdgeCross edgeCross : resultInfo.edgeCrosses) {
            addClosestPoint(resultPoints, edgeCross, pair.objectA, pair.objectB, resultInfo.worldSpaceNormal, resultInfo.overlap);
        }

        return resultPoints;
    }

    private void addClosestPoint(List<ManifoldPoint> resultPoints, NormalSatCollisionDetector.EdgeCross edgeCross, TransformedObject ta, TransformedObject tb, Vector3f worldSpaceNormal, float overlap) {
        SatEdge aEdge = edgeCross.aEdge, bEdge = edgeCross.bEdge;

        for (Edge edgeA : aEdge.getEdges()) {
            for (Edge edgeB : bEdge.getEdges()) {
                if (!areSupportEdge(edgeA, edgeB, worldSpaceNormal, ta, tb, overlap))
                    continue;

                addPoint(resultPoints, ta, tb, edgeA, edgeB, worldSpaceNormal, overlap);
            }
        }
    }

    private boolean areSupportEdge(Edge edgeA, Edge edgeB, Vector3f worldSpaceNormal, TransformedObject ta, TransformedObject tb, float overlap) {
        Vector3f p1 = edgeA.getVertex1();
        p1 = ta.toWorldSpace(p1);

        Vector3f p2 = edgeB.getVertex1();
        p2 = tb.toWorldSpace(p2);

        float p1Dot = worldSpaceNormal.dot(p1);
        float p2Dot = worldSpaceNormal.dot(p2);

        return p1Dot > p2Dot && p1Dot-p2Dot<=abs(overlap);
    }

    private static void addPoint(List<ManifoldPoint> resultPoints, TransformedObject ta, TransformedObject tb, Edge bestEdgeA, Edge bestEdgeB, Vector3f worldSpaceNormal, float overlap) {
        Vector3f p1 = bestEdgeA.getVertex1();
        p1 = ta.toWorldSpace(p1);

        Vector3f p2 = bestEdgeB.getVertex1();
        p2 = tb.toWorldSpace(p2);

        Vector3f d1 = bestEdgeA.getVertex2().duplicate().sub(bestEdgeA.getVertex1());
        ta.getRotation().transform(d1);
        Vector3f d2 = bestEdgeB.getVertex2().duplicate().sub(bestEdgeB.getVertex1());
        tb.getRotation().transform(d2);

        Vector3f p2SubP1 = p2.duplicate().sub(p1);
        Vector3f p1SubP2 = p1.duplicate().sub(p2);

        Vector3f n = d1.duplicate().cross(d2);

        Vector3f n1 = d1.duplicate().cross(n);
        Vector3f n2 = d2.duplicate().cross(n);

        float t1 = n2.dot(p2SubP1)/(n2.dot(d1));
        float t2 = n1.dot(p1SubP2)/(n1.dot(d2));

        // https://en.wikipedia.org/wiki/Skew_lines#Nearest_Points
        // https://math.stackexchange.com/questions/2213165/find-shortest-distance-between-lines-in-3d

        Vector3f qA = d1.duplicate().mul(t1).add(p1);
        Vector3f qB = d2.duplicate().mul(t2).add(p2);

        Vector3f lA = ta.toLocalSpace(qA);
        Vector3f lB = tb.toLocalSpace(qB);

        if (qB.duplicate().sub(qA).lengthSquared()>overlap*overlap)
            return;

        float qAOnEdge1 = d1.dot(qA);
        float qAOnEdge2 = d2.dot(qA);

        float p1OnEdge = d1.dot(p1);
        float p2OnEdge = d2.dot(p2);

        if (qAOnEdge1<p1OnEdge || qAOnEdge1>p1OnEdge+d1.lengthSquared())
            return;

        if (qAOnEdge2<p2OnEdge || qAOnEdge2>p2OnEdge+d2.lengthSquared())
            return;


        resultPoints.add(new EdgeManifoldPoint(lA, lB, worldSpaceNormal, bestEdgeA, bestEdgeB));
    }

    private List<ManifoldPoint> runFaces(GeneratedContactPair pair, SatProcessedShape sa, SatProcessedShape sb, NormalSatCollisionDetector.ResultInfo resultInfo) {
        if (!resultInfo.contactType.isFace()) {
            return null;
        }

        List<ManifoldPoint> resultPoints = new ArrayList<>();

        if (resultInfo.contactType.isFromA()) {
            Result result = findReferenceFace(resultInfo.aAxis.getCorrespondingPlanes(), resultInfo.worldSpaceNormal, pair.objectA);
            SatFace referenceFace = result.satFace;
            SatFace incidentFace = findIncidentFace(result.transformedNormal, sb.getSatFaces(), pair.objectB);

            Edge[] incidentEdges = incidentFace.getFaceEdge();
            ClipPlane[] relevantPlanes = referenceFace.getNeighbourPlanes();

            ClipPlanes clipping = new ClipPlanes(relevantPlanes);
            List<AbstractClippingSurface.ClippedPoint> points = clipping.clipUsingWorldSpace(pair.objectA, pair.objectB, incidentEdges);

            deduplicateEdgesPoints(points);

            getPointsFromClippedEdges(pair, resultInfo, resultPoints, referenceFace, points, pair.objectA, false);

        }
        if (resultInfo.contactType.isFromB()) {
            Result result = findReferenceFace(resultInfo.bAxis.getCorrespondingPlanes(), resultInfo.worldSpaceNormal.duplicate().mul(-1), pair.objectB);
            SatFace referenceFace = result.satFace;
            SatFace incidentFace = findIncidentFace(result.transformedNormal, sa.getSatFaces(), pair.objectA);

            Edge[] incidentEdges = incidentFace.getFaceEdge();
            ClipPlane[] relevantPlanes = referenceFace.getNeighbourPlanes();

            ClipPlanes clipping = new ClipPlanes(relevantPlanes);
            List<AbstractClippingSurface.ClippedPoint> points = clipping.clipUsingWorldSpace(pair.objectB, pair.objectA, incidentEdges);
            deduplicateEdgesPoints(points);

            getPointsFromClippedEdges(pair, resultInfo, resultPoints, referenceFace, points, pair.objectB, true);

        }

        //based of https://www.gamedev.net/forums/topic/588070-seperating-axis-theorem---how-to-resolve-contact-points/

        return resultPoints;
    }

    private static void deduplicateEdgesPoints(List<AbstractClippingSurface.ClippedPoint> points) {
        for (int i = 0; i < points.size(); i++) {
            AbstractClippingSurface.ClippedPoint firstPoint = points.get(i);

            for (int j = 0; j < points.size(); j++) {
                if (j<=i)
                    continue;

                AbstractClippingSurface.ClippedPoint secondPoint = points.get(j);

                if (firstPoint.clipped.duplicate().sub(secondPoint.clipped).lengthSquared()<MERGE_THRESHOLD) {
                    points.remove(j);
                    j--;
                }
            }
        }
    }

    private void getPointsFromClippedEdges(GeneratedContactPair pair, NormalSatCollisionDetector.ResultInfo resultInfo, List<ManifoldPoint> resultPoints, SatFace referenceFace, Collection<AbstractClippingSurface.ClippedPoint> points, TransformedObject t, boolean fromA) {
        Vector3f worldSpaceNormal = referenceFace.getClipPlane().getNormal().duplicate();
        t.getRotation().transform(worldSpaceNormal);

        Vector3f worldSpaceVertex = t.toWorldSpace(referenceFace.getClipPlane().getVertexOnPlane());

        float dottedPointOnReference = worldSpaceVertex.dot(worldSpaceNormal);
        for (AbstractClippingSurface.ClippedPoint point : points) {
            Vector3f clipped = point.clipped;
            float penetration = clipped.dot(worldSpaceNormal);
            if (isOnTopReferenceFace(dottedPointOnReference, penetration)) {
                continue;
            }


            float dist = (dottedPointOnReference-penetration);
            dist = Math.min(resultInfo.overlap, dist);
            resultPoints.add(ClipManifoldPoint.fromWorld(point.clipped, dist*(fromA ? -1 : 1), fromA, resultInfo.worldSpaceNormal, pair.objectA, pair.objectB, new PointIndex(point.index, fromA)));
        }
    }

    private boolean isOnTopReferenceFace(float face, float vertex) {
        return vertex>face;
    }

    private Result findReferenceFace(SatFace[] faces, Vector3f directedNormal, TransformedObject tA) {
        SatFace referenceFace = null;
        Vector3f referenceNormal = null;

        float mostParallel = -Float.MAX_VALUE;

        for (SatFace satFace : faces) {
            ClipPlane clipPlane = satFace.getClipPlane();
            Vector3f transformedNormal = clipPlane.getNormal().duplicate();
            tA.getRotation().transform(transformedNormal);
            float dot = transformedNormal.dot(directedNormal);
            if (dot < mostParallel) {
                continue;
            }

            mostParallel = dot;
            referenceNormal = transformedNormal;
            referenceFace = satFace;
        }
        return new Result(referenceFace, referenceNormal);
    }

    private SatFace findIncidentFace(Vector3f directedNormal, SatFace[] surface, TransformedObject tB) {
        float currentParallel = Float.MAX_VALUE;
        SatFace selected = null;

        for (SatFace axis : surface) {
            Vector3f transformedNormal = axis.getClipPlane().getNormal().duplicate();
            tB.getRotation().transform(transformedNormal);
            float dot = transformedNormal.dot(directedNormal);

            if (dot<currentParallel) {
                selected = axis;
                currentParallel = dot;
            }
        }

        return selected;
    }

    private void removeDuplicatedContactsPoints(List<ManifoldPoint> pairResults, GeneratedContactPair pair, Vector3f directedNormal) {
        for (ManifoldPoint pairResult : pairResults) {
            pairResult.refresh(pair.objectA, pair.objectB);
        }

        Vector3f cache = new SimpleVector3f();

        for (int i = 0; i < pairResults.size(); i++) {
            ManifoldPoint manifold1 = pairResults.get(i);

            Vector3f pointA1 = manifold1.worldSpaceA;
            Vector3f pointB1 = manifold1.worldSpaceB;

            for (int j = 0; j < pairResults.size(); j++) {
                if (j<=i)
                    continue;

                ManifoldPoint manifold2 = pairResults.get(j);

                Vector3f pointA2 = manifold2.worldSpaceA;
                Vector3f pointB2 = manifold2.worldSpaceB;

                float epsilon = EPSILON;

                cache.copy(pointA1);
                if (cache.sub(pointA2).lengthSquared() < epsilon) {
                    float projected1 = directedNormal.dot(pointB1);
                    float projected2 = directedNormal.dot(pointB2);
                    if (projected1>projected2) {
                        pairResults.remove(j);
                    } else {
                        pairResults.remove(i);
                        i--;
                        break;
                    }
                    j--;
                    continue;
                }
                cache.copy(pointB1);
                if (cache.sub(pointB2).lengthSquared() < epsilon) {
                    float projected1 = directedNormal.dot(pointA1);
                    float projected2 = directedNormal.dot(pointA2);
                    if (projected1>projected2) {
                        pairResults.remove(j);
                    } else {
                        pairResults.remove(i);
                        i--;
                        break;
                    }
                    j--;
                    continue;
                }
                cache.copy(pointA1);
                if (cache.sub(pointB2).lengthSquared() < epsilon) {
                    float projected1 = directedNormal.dot(pointB1);
                    float projected2 = directedNormal.dot(pointA2);
                    if (projected1>projected2) {
                        pairResults.remove(j);
                    } else {
                        pairResults.remove(i);
                        i--;
                        break;
                    }
                    j--;
                    continue;
                }
                cache.copy(pointB1);
                if (cache.sub(pointA2).lengthSquared() < epsilon) {
                    float projected1 = directedNormal.dot(pointA1);
                    float projected2 = directedNormal.dot(pointB2);
                    if (projected1>projected2) {
                        pairResults.remove(j);
                    } else {
                        pairResults.remove(i);
                        i--;
                        break;
                    }
                    j--;
                    continue;
                }
            }
        }
    }

    private static class Result {

        public SatFace satFace;
        public Vector3f transformedNormal;

        public Result(SatFace satFace, Vector3f transformedNormal) {
            this.satFace = satFace;
            this.transformedNormal = transformedNormal;
        }
    }
}
