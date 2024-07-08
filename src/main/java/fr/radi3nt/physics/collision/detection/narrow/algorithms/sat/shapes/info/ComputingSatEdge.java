package fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.info;

import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.clip.Edge;

import java.util.ArrayList;
import java.util.Collection;

import static java.lang.Math.abs;

public class ComputingSatEdge implements SatEdge {

    private final Vector3f axis;
    private final Edge[] currentEdges;

    public ComputingSatEdge(Vector3f axis, Edge[] allEdge, Vector3f[] edgesAxis) {
        this.axis = axis;

        Collection<Edge> edges = new ArrayList<>();
        for (int i = 0; i < edgesAxis.length; i++) {
            Vector3f currentAxis = edgesAxis[i];
            if (abs(currentAxis.dot(axis)) == 1)
                edges.add(allEdge[i]);
        }
        this.currentEdges = edges.toArray(new Edge[0]);
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
