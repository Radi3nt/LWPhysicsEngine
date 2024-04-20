package fr.radi3nt.physics.collision.shape.sat.clip;

import fr.radi3nt.maths.components.vectors.Vector3f;

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
}
