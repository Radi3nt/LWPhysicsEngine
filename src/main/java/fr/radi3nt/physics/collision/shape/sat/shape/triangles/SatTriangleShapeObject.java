package fr.radi3nt.physics.collision.shape.sat.shape.triangles;

import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.maths.components.vectors.implementations.SimpleVector3f;
import fr.radi3nt.physics.collision.shape.sat.clip.ClipPlane;
import fr.radi3nt.physics.collision.shape.sat.clip.ClipPlanes;
import fr.radi3nt.physics.collision.shape.sat.clip.Edge;
import fr.radi3nt.physics.collision.shape.sat.shape.SatShapeObject;
import fr.radi3nt.physics.collision.shape.sat.shape.StoredSatShapeObject;

public class SatTriangleShapeObject extends StoredSatShapeObject implements SatShapeObject, ReadTriangle, SatTriangle {

    private final Vector3f first;
    private final Vector3f second;
    private final Vector3f third;

    protected Vector3f normal;
    private Vector3f center;

    public SatTriangleShapeObject(Vector3f first, Vector3f second, Vector3f third) {
        this.first = first;
        this.second = second;
        this.third = third;
        compute();
    }

    public void compute() {
        computeAxisArray();
        computeVertices();
        computeClipEdges();
        computeClipPlanes();
        computeRadius();
    }

    private void computeRadius() {
        center = first.duplicate().add(second).add(third).div(3);
        radius = center.duplicate().sub(first).length();
    }

    protected void computeClipPlanes() {
        clipPlanes = new ClipPlanes(
                new ClipPlane(normal, vertices[0])
        );
    }

    protected void computeClipEdges() {
        clipEdges = new Edge[] {
                new Edge(vertices[0], vertices[1]),
                new Edge(vertices[1], vertices[2]),
                new Edge(vertices[2], vertices[0]),
        };
    }

    protected void computeAxisArray() {
        computeNormal();

        axis = new Vector3f[] {
                normal
        };
    }

    protected void computeNormal() {
        normal = new SimpleVector3f();

        normal.setX(second.getX());
        normal.setY(second.getY());
        normal.setZ(second.getZ());

        final Vector3f tangentA = normal.sub(first);

        float pos3X = third.getX() - first.getX();
        float pos3Y = third.getY() - first.getY();
        float pos3Z = third.getZ() - first.getZ();

        float x = tangentA.getY() * pos3Z - tangentA.getZ() * pos3Y;
        float y = tangentA.getZ() * pos3X - tangentA.getX() * pos3Z;
        float z = tangentA.getX() * pos3Y - tangentA.getY() * pos3X;

        normal.setX(x);
        normal.setY(y);
        normal.setZ(z);
        normal.normalize();
    }

    private void computeVertices() {
        vertices = new Vector3f[]{
                first,
                second,
                third
        };
    }

    public Vector3f getCenter() {
        return center;
    }

    @Override
    public Vector3f[] getEdges() {
        Vector3f[] edges = new Vector3f[3];
        for (int i = 0; i < clipEdges.length; i++) {
            Edge edge = clipEdges[i];
            edges[i] = edge.getVertex2().duplicate().sub(edge.getVertex1()).normalize();
        }
        return edges;
    }

    @Override
    public Vector3f getVertex1() {
        return first;
    }

    @Override
    public Vector3f getVertex2() {
        return second;
    }

    @Override
    public Vector3f getVertex3() {
        return third;
    }

    @Override
    public Vector3f getNormal() {
        return normal;
    }
}
