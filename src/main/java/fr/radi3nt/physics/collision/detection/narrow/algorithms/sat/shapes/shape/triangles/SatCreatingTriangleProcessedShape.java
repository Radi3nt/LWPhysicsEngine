package fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.shape.triangles;

import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.maths.components.vectors.implementations.SimpleVector3f;
import fr.radi3nt.maths.pool.ObjectPool;
import fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.info.*;
import fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.projection.SatProjectionProvider;
import fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.clip.ClipPlane;
import fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.clip.ClipPlanes;
import fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.clip.Edge;
import fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.projection.VerticesProjectionProvider;
import fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.shape.SatProcessedShape;
import fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.shape.StoredSatProcessedShape;
import fr.radi3nt.physics.core.TransformedObject;

public class SatCreatingTriangleProcessedShape implements SatProcessedShape, ReadTriangle, SatProcessedTriangle {

    private final Vector3f first;
    private final Vector3f second;
    private final Vector3f third;

    protected Vector3f normal = new SimpleVector3f();
    private final VerticesProjectionProvider verticesProjectionProvider;

    public SatCreatingTriangleProcessedShape(TransformedObject object, Vector3f first, Vector3f second, Vector3f third) {
        this.first = first;
        this.second = second;
        this.third = third;
        computeNormal();

        verticesProjectionProvider = new VerticesProjectionProvider(StoredSatProcessedShape.getTransformedArray(getVertices(), object));
    }

    protected void computeNormal() {
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

    @Override
    public Vector3f[] getAxis() {
        return new Vector3f[] {normal};
    }

    @Override
    public Vector3f[] getEdges() {
        return new Vector3f[] {
                second.duplicate().sub(first),
                third.duplicate().sub(second),
                first.duplicate().sub(third)
        };
    }

    @Override
    public ClipPlanes getClipPlanes() {
        return new ClipPlanes(new ClipPlane(normal, first));
    }

    @Override
    public Edge[] getClipEdges() {
        return actualEdges();
    }

    private Edge[] actualEdges() {
        return new Edge[]{
                new Edge(first, second),
                new Edge(second, third),
                new Edge(third, first),
        };
    }

    @Override
    public boolean canUseEdgesAsMinimum() {
        return true;
    }

    @Override
    public SatProjectionProvider getSatProjectionProvider(TransformedObject object, ObjectPool<Vector3f> pool) {
        return verticesProjectionProvider;
    }

    private Vector3f[] getVertices() {
        return new Vector3f[]{
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
                new SetSatFace(getClipPlanes().getClipPlanes()[0], new ClipPlane[0], actualEdges())
        };
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
