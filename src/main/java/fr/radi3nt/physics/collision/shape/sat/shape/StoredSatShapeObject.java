package fr.radi3nt.physics.collision.shape.sat.shape;

import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.physics.collision.shape.sat.clip.ClipPlanes;
import fr.radi3nt.physics.collision.shape.sat.clip.Edge;

public abstract class StoredSatShapeObject implements SatShapeObject {

    protected Vector3f[] axis;
    protected Vector3f[] vertices;
    protected ClipPlanes clipPlanes;
    protected Edge[] clipEdges;
    protected float radius;

    @Override
    public Vector3f[] getAxis() {
        return axis;
    }

    @Override
    public ClipPlanes getClipPlanes() {
        return clipPlanes;
    }

    @Override
    public Edge[] getClipEdges() {
        return clipEdges;
    }

    @Override
    public Vector3f[] getVertices() {
        return vertices;
    }

    @Override
    public float getRadius() {
        return radius;
    }
}
