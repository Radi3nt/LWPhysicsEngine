package fr.radi3nt.physics.collision.shape.sat.shape.triangles;

import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.maths.components.vectors.implementations.SimpleVector3f;
import fr.radi3nt.physics.collision.shape.sat.clip.ClipPlane;
import fr.radi3nt.physics.collision.shape.sat.clip.ClipPlanes;
import fr.radi3nt.physics.collision.shape.sat.clip.Edge;
import fr.radi3nt.physics.collision.shape.sat.shape.SatShapeObject;

public class SatCreatingTriangleShapeObject implements SatShapeObject, ReadTriangle, SatTriangle {

    private final Vector3f first;
    private final Vector3f second;
    private final Vector3f third;

    protected Vector3f normal = new SimpleVector3f();
    private float radius;

    public SatCreatingTriangleShapeObject(Vector3f first, Vector3f second, Vector3f third, Vector3f cache) {
        this.first = first;
        this.second = second;
        this.third = third;
        computeRadius(cache);
        computeNormal();
    }

    private void computeRadius(Vector3f cache) {
        radius = getCenter(cache).sub(first).length();
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
    public Vector3f getCenter() {
        return first.duplicate().add(second).add(third).div(3);
    }

    public Vector3f getCenter(Vector3f cache) {
        cache.copy(first);
        return cache.add(second).add(third).div(3);
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
        return new ClipPlanes(new ClipPlane(normal, first, true));
    }

    @Override
    public Edge[] getClipEdges() {
        return new Edge[] {
                new Edge(first, second),
                new Edge(second, third),
                new Edge(third, first),
        };
    }

    @Override
    public Vector3f[] getVertices() {
        return new Vector3f[]{
                first,
                second,
                third
        };
    }

    @Override
    public float getRadius() {
        return radius;
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
