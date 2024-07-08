package fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.info;

import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.clip.ClipPlane;
import fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.clip.Edge;

import java.util.ArrayList;
import java.util.Collection;

public class ComputingSatFace implements SatFace {

    private final ClipPlane plane;

    private final ClipPlane[] allPlanes;
    private final Edge[] allEdges;

    public ComputingSatFace(ClipPlane plane, ClipPlane[] allPlanes, Edge[] allEdges) {
        this.plane = plane;
        this.allPlanes = allPlanes;
        this.allEdges = allEdges;
    }

    @Override
    public ClipPlane getClipPlane() {
        return plane;
    }

    @Override
    public ClipPlane[] getNeighbourPlanes() {
        Collection<ClipPlane> clipPlanes = new ArrayList<>();
        for (ClipPlane allPlane : allPlanes) {
            if (allPlane==plane)
                continue;

            clipPlanes.add(allPlane);
        }

        return clipPlanes.toArray(new ClipPlane[0]);
    }

    @Override
    public Edge[] getFaceEdge() {
        Collection<Edge> edges = new ArrayList<>();

        Vector3f currentNormal = plane.getNormal();
        float projectedPlaneVertex = currentNormal.dot(plane.getVertexOnPlane());
        for (Edge allEdge : allEdges) {
            if (allEdge.getVertex1().dot(currentNormal)==projectedPlaneVertex && allEdge.getVertex2().dot(currentNormal)==projectedPlaneVertex)
                edges.add(allEdge);
        }

        return edges.toArray(new Edge[0]);
    }
}
