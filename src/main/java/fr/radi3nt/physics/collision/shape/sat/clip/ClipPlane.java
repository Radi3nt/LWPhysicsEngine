package fr.radi3nt.physics.collision.shape.sat.clip;

import fr.radi3nt.maths.components.advanced.quaternions.Quaternion;
import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.physics.core.TransformedObject;

public class ClipPlane {

    private static final float EPSILON = 1e-5f;
    private final Vector3f normal;
    private final Vector3f vertexOnPlane;
    private boolean flipNormal = false;

    public ClipPlane(Vector3f normal, Vector3f vertexOnPlane) {
        this.normal = normal;
        this.vertexOnPlane = vertexOnPlane;
    }

    public ClipPlane(Vector3f normal, Vector3f vertexOnPlane, boolean flipNormal) {
        this.normal = normal;
        this.vertexOnPlane = vertexOnPlane;
        this.flipNormal = flipNormal;
    }

    public void clipUsingBSpace(Quaternion fromAToBSpaceRot, TransformedObject a, TransformedObject b, ResultEdge[] otherEdges, int index) {
        Vector3f bSpaceNormal = normal.duplicate();
        if (flipNormal)
            bSpaceNormal.negate();
        //fromAToBSpaceRot.transform(bSpaceNormal);
        a.getRotation().transform(bSpaceNormal);
        Quaternion inverseB = b.getRotation();
        inverseB.inverse();
        inverseB.transform(bSpaceNormal);

        float projectedClip = bSpaceNormal.dot(b.toLocalSpace(a.toWorldSpace(vertexOnPlane)));

        clip(otherEdges, bSpaceNormal, projectedClip, index);
    }

    public void clipUsingWorldSpace(TransformedObject a, TransformedObject b, ResultEdge[] otherEdges, int index) {
        Vector3f worldSpaceNormal = normal.duplicate();
        if (flipNormal)
            worldSpaceNormal.negate();
        a.getRotation().transform(worldSpaceNormal);

        float projectedClip = worldSpaceNormal.dot(a.toWorldSpace(vertexOnPlane));

        clip(otherEdges, worldSpaceNormal, projectedClip, index);
    }

    public void clip(ResultEdge[] otherEdges, Vector3f normal, float projectedClip, int index) {
        for (int i = 0; i < otherEdges.length; i++) {
            ResultEdge edge = otherEdges[i];
            if (edge==null)
                continue;

            float projectedVertex1 = normal.dot(edge.getVertex1());
            float projectedVertex2 = normal.dot(edge.getVertex2());

            if (projectedVertex1+EPSILON >= projectedClip && projectedVertex2+EPSILON >= projectedClip) {
                continue;
            }

            if (projectedVertex1==projectedVertex2) {
                otherEdges[i] = null;
                continue;
            }
            boolean twoClipped = projectedVertex2 < projectedClip+EPSILON;
            boolean oneClipped = projectedVertex1 < projectedClip+EPSILON;

            if (oneClipped && twoClipped)
                otherEdges[i] = null;

            if (twoClipped) {
                float on = (projectedClip - projectedVertex1) / (projectedVertex2 - projectedVertex1);
                Vector3f computedNewVertex = computeInterpolation(edge.getVertex1(), edge.getVertex2(), on);
                edge.setVertices(edge.getVertex1(), computedNewVertex);
                edge.setVertexTwoClipped(index);
                //projectedVertex2 = normal.dot(edge.getVertex2());
            } else if (oneClipped) {
                float on = (projectedClip - projectedVertex2) / (projectedVertex1 - projectedVertex2);
                Vector3f computedNewVertex = computeInterpolation(edge.getVertex2(), edge.getVertex1(), on);
                edge.setVertices(computedNewVertex, edge.getVertex2());
                edge.setVertexOneClipped(index);
            }
        }
    }

    private Vector3f computeInterpolation(Vector3f vertex1, Vector3f vertex2, float on) {
        Vector3f edgeLocal = vertex2.duplicate().sub(vertex1);
        return edgeLocal.mul(on).add(vertex1);
    }
}
