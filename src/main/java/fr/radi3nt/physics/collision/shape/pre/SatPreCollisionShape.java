package fr.radi3nt.physics.collision.shape.pre;

import fr.radi3nt.maths.aabb.AABB;
import fr.radi3nt.maths.aabb.VerticesAABB;
import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.physics.collision.detection.broad.aabb.CachingAABB;
import fr.radi3nt.physics.collision.detection.broad.aabb.VerticesCachingAABB;
import fr.radi3nt.physics.collision.shape.sat.shape.SatShapeObject;

public class SatPreCollisionShape implements PreCollisionShape {

    private final VerticesAABB verticesAABB;
    private final SatShapeObject satShapeObject;
    private final Vector3f offset;

    private final VerticesCachingAABB cachingAABB;

    public SatPreCollisionShape(SatShapeObject satShapeObject, Vector3f offset) {
        verticesAABB = new VerticesAABB(satShapeObject.getVertices());
        this.satShapeObject = satShapeObject;
        this.offset = offset;
        cachingAABB = new VerticesCachingAABB(satShapeObject.getVertices());
    }

    @Override
    public CachingAABB getAABB() {
        return cachingAABB;
    }

    @Override
    public AABB getLocalAABB() {
        return verticesAABB;
    }

    @Override
    public Vector3f getSphereOffset() {
        return offset;
    }

    @Override
    public float getRadius() {
        return satShapeObject.getRadius();
    }

}
