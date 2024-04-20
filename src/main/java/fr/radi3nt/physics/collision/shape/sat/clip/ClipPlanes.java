package fr.radi3nt.physics.collision.shape.sat.clip;

import fr.radi3nt.maths.components.advanced.quaternions.Quaternion;
import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.physics.collision.contact.manifold.ClipIndex;
import fr.radi3nt.physics.core.TransformedObject;

import java.util.ArrayList;
import java.util.Collection;

public class ClipPlanes {

    private static final double EPSILON = 1e-4f;
    private final ClipPlane[] clipPlane;

    public ClipPlanes(ClipPlane... clipPlane) {
        this.clipPlane = clipPlane;
    }

    public ResultEdge[] clipEdgesWorldSpace(TransformedObject a, TransformedObject b, Edge[] edges) {
        ResultEdge[] worldEdges = new ResultEdge[edges.length];
        for (int i = 0; i < edges.length; i++) {
            worldEdges[i] = new ResultEdge(b.toWorldSpace(edges[i].getVertex1()), b.toWorldSpace(edges[i].getVertex2()));
        }
        for (int i = 0; i < clipPlane.length; i++) {
            ClipPlane plane = clipPlane[i];
            plane.clipUsingWorldSpace(a, b, worldEdges, i);
        }
        return worldEdges;
    }

    public ResultEdge[] clipEdgesProvidingWorldSpace(TransformedObject a, TransformedObject b, Edge[] edges) {
        ResultEdge[] worldEdges = new ResultEdge[edges.length];
        for (int i = 0; i < edges.length; i++) {
            worldEdges[i] = new ResultEdge(edges[i].getVertex1().duplicate(), edges[i].getVertex2().duplicate());
        }
        for (int i = 0; i < clipPlane.length; i++) {
            ClipPlane plane = clipPlane[i];
            plane.clipUsingWorldSpace(a, b, worldEdges, i);
        }
        return worldEdges;
    }

    public ResultEdge[] clipEdgesBSpace(TransformedObject a, TransformedObject b, Edge[] edges) {
        Quaternion fromAToBSpaceRot = a.getRotation().duplicate();
        Quaternion inverseB = b.getRotation().duplicate();
        inverseB.inverse();
        fromAToBSpaceRot.multiply(inverseB);

        ResultEdge[] worldEdges = new ResultEdge[edges.length];
        for (int i = 0; i < edges.length; i++) {
            worldEdges[i] = new ResultEdge(edges[i].getVertex1().duplicate(), edges[i].getVertex2().duplicate());
        }
        for (int i = 0; i < clipPlane.length; i++) {
            ClipPlane plane = clipPlane[i];
            plane.clipUsingBSpace(fromAToBSpaceRot, a, b, worldEdges, i);
        }
        return worldEdges;
    }

    public Collection<ClippedPoint> clipUsingWorldSpace(TransformedObject a, TransformedObject b, Edge[] edges) {
        ResultEdge[] edgeList = clipEdgesWorldSpace(a, b, edges);
        return deduplicate(edgeList);
    }

    public Collection<ClippedPoint> clipUsingBSpace(TransformedObject a, TransformedObject b, Edge[] edges) {
        ResultEdge[] edgeList = clipEdgesBSpace(a, b, edges);
        return deduplicate(edgeList);
    }

    private static Collection<ClippedPoint> deduplicate(ResultEdge[] edgeList) {
        Collection<ClippedPoint> result = new ArrayList<>(edgeList.length);
        for (int i = 0; i < edgeList.length; i++) {
            ResultEdge edge = edgeList[i];
            if (edge == null)
                continue;
            result.add(new ClippedPoint(edge.getVertex1(), new ClipIndex(i, edge.getPlaneClippedForVertex1()), false));
            if (notSameVertices(edge))
                result.add(new ClippedPoint(edge.getVertex2(), new ClipIndex(i, edge.getPlaneClippedForVertex2()), true));
        }
        return result;
    }

    private static boolean notSameVertices(ResultEdge edge) {
        return edge.getVertex1().duplicate().sub(edge.getVertex2()).lengthSquared() > EPSILON;
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
