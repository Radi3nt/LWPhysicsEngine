package fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.shape.triangles;

import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.maths.components.vectors.implementations.SimpleVector3f;
import fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.clip.ClipPlane;
import fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.clip.ClipPlanes;
import fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.clip.Edge;
import fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.info.*;
import fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.shape.SatProcessedShape;
import fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.shape.StoredSatProcessedShape;

public class SatTriangleProcessedShape extends StoredSatProcessedShape implements SatProcessedShape, ReadTriangle, SatProcessedTriangle {

    private final Vector3f first;
    private final Vector3f second;
    private final Vector3f third;

    protected Vector3f normal;

    public SatTriangleProcessedShape(Vector3f first, Vector3f second, Vector3f third) {
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

    @Override
    public SatAxis[] getSatAxis() {
        return new SatAxis[] {
                new SetSatAxis(normal, getSatFaces())
        };
    }

    @Override
    public SatFace[] getSatFaces() {
        return new SatFace[]{
                new SetSatFace(getClipPlanes().getClipPlanes()[0], new ClipPlane[0], clipEdges)
        };
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
    public SatEdge[] getSatEdges() {
        return new SatEdge[] {
                new SetSatEdge(new Edge(first, second)),
                new SetSatEdge(new Edge(second, third)),
                new SetSatEdge(new Edge(third, first)),
        };
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
