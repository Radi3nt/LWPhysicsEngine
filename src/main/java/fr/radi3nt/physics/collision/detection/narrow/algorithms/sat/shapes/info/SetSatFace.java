package fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.info;

import fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.clip.ClipPlane;
import fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.clip.Edge;

public class SetSatFace implements SatFace {

    private final ClipPlane clipPlane;
    private final ClipPlane[] neighbouringPlanes;
    private final Edge[] faceEdges;

    public SetSatFace(ClipPlane clipPlane, ClipPlane[] neighbouringPlanes, Edge[] faceEdges) {
        this.clipPlane = clipPlane;
        this.neighbouringPlanes = neighbouringPlanes;
        this.faceEdges = faceEdges;
    }

    @Override
    public ClipPlane getClipPlane() {
        return clipPlane;
    }

    @Override
    public ClipPlane[] getNeighbourPlanes() {
        return neighbouringPlanes;
    }

    @Override
    public Edge[] getFaceEdge() {
        return faceEdges;
    }
}
