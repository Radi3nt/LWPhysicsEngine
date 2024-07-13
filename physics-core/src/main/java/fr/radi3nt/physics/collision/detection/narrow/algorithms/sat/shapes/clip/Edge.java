package fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.clip;

import fr.radi3nt.maths.components.vectors.Vector3f;

import java.util.Objects;

public class Edge {

    private Vector3f vertex1;
    private Vector3f vertex2;

    public Edge(Vector3f vertex1, Vector3f vertex2) {
        this.vertex1 = vertex1;
        this.vertex2 = vertex2;
    }

    public Vector3f getVertex1() {
        return vertex1;
    }

    public Vector3f getVertex2() {
        return vertex2;
    }

    public void setVertices(Vector3f vertex1, Vector3f vertex2) {
        this.vertex1 = vertex1;
        this.vertex2 = vertex2;
    }

    @Override
    public final boolean equals(Object o) {
        if (this == o) return true;
        if (!(o instanceof Edge)) return false;

        Edge edge = (Edge) o;
        return Objects.equals(vertex1, edge.vertex1) && Objects.equals(vertex2, edge.vertex2);
    }

    @Override
    public int hashCode() {
        int result = Objects.hashCode(vertex1);
        result = 31 * result + Objects.hashCode(vertex2);
        return result;
    }
}
