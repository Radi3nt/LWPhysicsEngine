package fr.radi3nt.physics.collision.detection.narrow.sat.computer.part.normal;

import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.physics.collision.contact.ContactPair;
import fr.radi3nt.physics.collision.contact.manifold.ManifoldPoint;
import fr.radi3nt.physics.collision.contact.manifold.ClipIndex;
import fr.radi3nt.physics.collision.contact.manifold.PointIndex;
import fr.radi3nt.physics.collision.detection.narrow.sat.computer.part.ManifoldPointBuilder;
import fr.radi3nt.physics.collision.detection.narrow.sat.computer.part.ShapedPair;
import fr.radi3nt.physics.collision.shape.sat.clip.ClipPlanes;
import fr.radi3nt.physics.collision.shape.sat.clip.Edge;
import fr.radi3nt.physics.collision.shape.sat.clip.ResultEdge;
import fr.radi3nt.physics.collision.shape.sat.shape.SatShapeObject;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

import static java.lang.Math.abs;

public class NormalReuseManifoldPointBuilder implements ManifoldPointBuilder {

    private static final float PROJECTION_DUPLICATION = 0.1f;

    public NormalReuseManifoldPointBuilder() {
    }

    private List<ManifoldPoint> createContactPoints(ContactPair pair, SatShapeObject sa, SatShapeObject sb, Vector3f directedNormal, float overlap) {
        List<ManifoldPoint> pairResults = new ArrayList<>();

        ClipPlanes saClip = sa.getClipPlanes();
        ClipPlanes sbClip = sb.getClipPlanes();

        Collection<ClipPlanes.ClippedPoint> contactPointsB = saClip.clipUsingWorldSpace(pair.objectA, pair.objectB, sb.getClipEdges());
        Collection<ClipPlanes.ClippedPoint> contactPointsA = sbClip.clipUsingWorldSpace(pair.objectB, pair.objectA, sa.getClipEdges());

        Collection<ClipPlanes.ClippedPoint> projectingPointsA = new ArrayList<>();
        Collection<ClipPlanes.ClippedPoint> projectingPointsB = new ArrayList<>();

        Collection<Integer> pointIndicesInSa = new ArrayList<>();
        Collection<Integer> pointIndicesInSb = new ArrayList<>();
        Collection<Vector3f> pointResults = new ArrayList<>();

        for (ClipPlanes.ClippedPoint worldContactB : contactPointsB) {
            Vector3f projectedPoint = directedNormal.duplicate().mul(overlap).add(worldContactB.clipped);
            ResultEdge[] result = saClip.clipEdgesProvidingWorldSpace(pair.objectA, pair.objectB, new Edge[]{new Edge(worldContactB.clipped, projectedPoint)});

            boolean projected = result[0] == null;
            if (projected) {
                projectingPointsB.add(worldContactB);
                continue;
            }


            Vector3f worldContactA = result[0].getVertex2();
            pointResults.add(worldContactA);

            pairResults.add(toLocalSpace(pair, worldContactA, worldContactB.clipped, new PointIndex(worldContactB.index, false)));
            pointIndicesInSa.add(result[0].getPlaneClippedForVertex2());
        }
        for (ClipPlanes.ClippedPoint worldContact : contactPointsA) {
            Vector3f projectedPoint = directedNormal.duplicate().mul(-overlap).add(worldContact.clipped);
            ResultEdge[] result = saClip.clipEdgesProvidingWorldSpace(pair.objectB, pair.objectA, new Edge[]{new Edge(worldContact.clipped, projectedPoint)});
            boolean projected = result[0] == null;
            if (projected) {
                projectingPointsA.add(worldContact);
                continue;
            }

            Vector3f worldContactB = result[0].getVertex2();
            pointResults.add(worldContactB);
            pointIndicesInSb.add(result[0].getPlaneClippedForVertex2());
            pairResults.add(toLocalSpace(pair, worldContact.clipped, worldContactB, new PointIndex(worldContact.index, true)));
        }

        for (ClipPlanes.ClippedPoint worldContactB : projectingPointsB) {
            if (isInvalidProjectedPoint(pointIndicesInSa, pointResults, worldContactB)) continue;
            Vector3f worldContactA = directedNormal.duplicate().mul(overlap).add(worldContactB.clipped);
            pairResults.add(toLocalSpace(pair, worldContactA, worldContactB.clipped, new PointIndex(worldContactB.index, false)));
        }

        for (ClipPlanes.ClippedPoint worldContactA : projectingPointsA) {
            if (isInvalidProjectedPoint(pointIndicesInSb, pointResults, worldContactA)) continue;
            Vector3f worldContactB = directedNormal.duplicate().mul(-overlap).add(worldContactA.clipped);
            pairResults.add(toLocalSpace(pair, worldContactA.clipped, worldContactB, new PointIndex(worldContactA.index, true)));
        }


        return pairResults;
    }

    private boolean isInvalidProjectedPoint(Collection<Integer> pointIndicesInSa, Collection<Vector3f> pointResults, ClipPlanes.ClippedPoint worldContactB) {
        ClipIndex index = worldContactB.index;
        if (pointIndicesInSa.contains(index.clippedPlane)) {
            return true;
        }

        boolean found = false;
        for (Vector3f pointResult : pointResults) {
            if (pointResult.duplicate().sub(worldContactB.clipped).lengthSquared()<PROJECTION_DUPLICATION) {
                found = true;
                break;
            }
        }
        return found;
    }

    private ManifoldPoint toLocalSpace(ContactPair pair, Vector3f worldAPoint, Vector3f worldBPoint, PointIndex index) {
        return new ManifoldPoint(index, pair.objectA.toLocalSpace(worldAPoint), pair.objectB.toLocalSpace(worldBPoint));
    }

    @Override
    public List<ManifoldPoint> computeManifolds(ShapedPair shapedPair, Vector3f worldSpaceDirectedNormal, float overlap, int normal) {
        return createContactPoints(shapedPair.contactPair, shapedPair.sA, shapedPair.sB, worldSpaceDirectedNormal, overlap);
    }
}
