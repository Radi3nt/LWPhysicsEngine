package fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.info;

import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.clip.Edge;

import static java.lang.Math.abs;

public class SetSatEdge implements SatEdge {

    private final Vector3f axis;
    private final Edge[] currentEdges;

    public SetSatEdge(Vector3f axis, Edge[] currentEdges) {
        this.axis = axis;
        this.currentEdges = currentEdges;
    }

    public SetSatEdge(Edge currentEdge) {
        this.axis = currentEdge.getVertex2().duplicate().sub(currentEdge.getVertex1()).normalize();
        this.currentEdges = new Edge[] {currentEdge};
    }

    @Override
    public Vector3f getAxis() {
        return axis;
    }

    @Override
    public Edge[] getEdges() {
        return currentEdges;
    }
}
