package fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.clip;

import fr.radi3nt.physics.core.TransformedObject;

public class ClipPlanes extends AbstractClippingSurface {

    private final ClipPlane[] clipPlane;

    public ClipPlanes(ClipPlane... clipPlane) {
        this.clipPlane = clipPlane;
    }

    protected void clip(TransformedObject planeTransform, ResultEdge[] worldEdges) {
        for (int i = 0; i < clipPlane.length; i++) {
            ClipPlane plane = clipPlane[i];
            plane.clipUsingWorldSpace(planeTransform, worldEdges, i);
        }
    }

    public ClipPlane[] getClipPlanes() {
        return clipPlane;
    }
}
