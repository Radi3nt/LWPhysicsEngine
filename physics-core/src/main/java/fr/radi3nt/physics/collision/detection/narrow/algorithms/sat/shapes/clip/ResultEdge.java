package fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.clip;

import fr.radi3nt.maths.components.vectors.Vector3f;

public class ResultEdge {

    private Vector3f vertex1;
    private Vector3f vertex2;
    private int vertexOneClipped = -1;
    private int vertexTwoClipped = -1;

    public ResultEdge(Vector3f vertex1, Vector3f vertex2) {
        this.vertex1 = vertex1;
        this.vertex2 = vertex2;
    }

    public Vector3f getVertex1() {
        return vertex1;
    }

    public Vector3f getVertex2() {
        return vertex2;
    }

    public void setVertexOneClipped(int clipped) {
        this.vertexOneClipped = clipped;
    }

    public void setVertexTwoClipped(int clipped) {
        this.vertexTwoClipped = clipped;
    }

    public boolean isVertexOneClipped() {
        return vertexOneClipped!=-1;
    }

    public boolean isVertexTwoClipped() {
        return vertexTwoClipped!=-1;
    }

    public void setVertices(Vector3f vertex1, Vector3f vertex2) {
        this.vertex1 = vertex1;
        this.vertex2 = vertex2;
    }

    public int getPlaneClippedForVertex1() {
        return vertexOneClipped;
    }

    public int getPlaneClippedForVertex2() {
        return vertexTwoClipped;
    }
}
