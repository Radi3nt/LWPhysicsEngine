package fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.clip;

import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.physics.collision.contact.manifold.manifolds.clip.ClipIndex;
import fr.radi3nt.physics.core.TransformedObject;

import java.util.ArrayList;
import java.util.List;

public abstract class AbstractClippingSurface {

    private static final double EPSILON = 1e-4f;

    public ResultEdge[] clipEdgesProvidingWorldSpace(TransformedObject planeTransform, Edge[] edges) {
        ResultEdge[] worldEdges = new ResultEdge[edges.length];
        for (int i = 0; i < edges.length; i++) {
            worldEdges[i] = new ResultEdge(edges[i].getVertex1().duplicate(), edges[i].getVertex2().duplicate());
        }
        clip(planeTransform, worldEdges);
        return worldEdges;
    }

    protected abstract void clip(TransformedObject planeTransform, ResultEdge[] worldEdges);

    public List<ClippedPoint> clipUsingWorldSpace(TransformedObject planeTransform, TransformedObject edgeTransform, Edge[] edges) {
        ResultEdge[] edgeList = clipEdgesWorldSpace(planeTransform, edgeTransform, edges);
        return deduplicate(edgeList);
    }

    private ResultEdge[] clipEdgesWorldSpace(TransformedObject planeTransform, TransformedObject edgesTransform, Edge[] edges) {
        ResultEdge[] worldEdges = new ResultEdge[edges.length];
        for (int i = 0; i < edges.length; i++) {
            worldEdges[i] = new ResultEdge(edgesTransform.toWorldSpace(edges[i].getVertex1()), edgesTransform.toWorldSpace(edges[i].getVertex2()));
        }
        clip(planeTransform, worldEdges);
        return worldEdges;
    }

    private static List<ClippedPoint> deduplicate(ResultEdge[] edgeList) {
        List<ClippedPoint> result = new ArrayList<>(edgeList.length);
        for (int i = 0; i < edgeList.length; i++) {
            ResultEdge edge = edgeList[i];
            if (edge == null)
                continue;
            result.add(new ClippedPoint(edge.getVertex1(), new ClipIndex(i, false, edge.getPlaneClippedForVertex1()), false));
            if (notSameVertices(edge))
                result.add(new ClippedPoint(edge.getVertex2(), new ClipIndex(i, true, edge.getPlaneClippedForVertex2()), true));
        }
        return result;
    }

    private static boolean notSameVertices(ResultEdge edge) {
        float dist = edge.getVertex1().duplicate().sub(edge.getVertex2()).lengthSquared();
        return dist > EPSILON;
    }

    public static class ClippedPoint {

        public final Vector3f clipped;
        public final ClipIndex index;
        public final boolean tip;

        public ClippedPoint(Vector3f clipped, ClipIndex clipIndex, boolean tip) {
            this.clipped = clipped;
            this.index = clipIndex;
            this.tip = tip;
        }

    }


}
