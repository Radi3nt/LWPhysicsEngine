package fr.radi3nt.physics.collision.shape.sat.shape;

import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.maths.components.vectors.implementations.SimpleVector3f;
import fr.radi3nt.physics.collision.shape.sat.clip.ClipPlane;
import fr.radi3nt.physics.collision.shape.sat.clip.ClipPlanes;
import fr.radi3nt.physics.collision.shape.sat.clip.Edge;

public class SatBoxShapeObject extends StoredSatShapeObject implements SatShapeObject {

    private final Vector3f size;

    public SatBoxShapeObject(Vector3f size) {
        this.size = size;
    }

    public SatBoxShapeObject compute() {
        computeVertices();
        computeAxisArray();
        computeClipEdges();
        computeClipPlanes();
        computeRadius();
        return this;
    }

    private void computeRadius() {
        radius = size.length();
    }

    private void computeClipPlanes() {
        clipPlanes = new ClipPlanes(
                new ClipPlane(new SimpleVector3f(1, 0, 0), vertices[0]),
                new ClipPlane(new SimpleVector3f(0, 1, 0), vertices[0]),
                new ClipPlane(new SimpleVector3f(0, 0, 1), vertices[0]),
                new ClipPlane(new SimpleVector3f(-1, 0, 0), vertices[1]),
                new ClipPlane(new SimpleVector3f(0, -1, 0), vertices[1]),
                new ClipPlane(new SimpleVector3f(0, 0, -1), vertices[1])
        );
    }

    private void computeClipEdges() {
        clipEdges = new Edge[] {
                new Edge(vertices[0], vertices[6]),
                new Edge(vertices[0], vertices[4]),
                new Edge(vertices[0], vertices[2]),

                new Edge(vertices[1], vertices[7]),
                new Edge(vertices[1], vertices[5]),
                new Edge(vertices[1], vertices[3]),

                new Edge(vertices[6], vertices[7]),
                new Edge(vertices[7], vertices[2]),

                new Edge(vertices[5], vertices[4]),
                new Edge(vertices[4], vertices[3]),

                new Edge(vertices[5], vertices[6]),
                new Edge(vertices[3], vertices[2])
        };
    }

    private void computeAxisArray() {
        axis = new Vector3f[] {
                vertices[6].duplicate().sub(vertices[0]).normalize(),
                vertices[4].duplicate().sub(vertices[0]).normalize(),
                vertices[2].duplicate().sub(vertices[0]).normalize(),
        };
    }

    protected void computeVertices() {
        Vector3f min = size.duplicate().negate();
        Vector3f max = size.duplicate();
        vertices = new Vector3f[]{
                min.duplicate(),
                max.duplicate(),
                new SimpleVector3f(max.getX(), min.getY(), min.getZ()),
                new SimpleVector3f(max.getX(), max.getY(), min.getZ()),
                new SimpleVector3f(min.getX(), max.getY(), min.getZ()),
                new SimpleVector3f(min.getX(), max.getY(), max.getZ()),
                new SimpleVector3f(min.getX(), min.getY(), max.getZ()),
                new SimpleVector3f(max.getX(), min.getY(), max.getZ())
        };
    }

    @Override
    public Vector3f[] getEdges() {
        return axis;
    }

    public Vector3f getSize() {
        return size;
    }
}
