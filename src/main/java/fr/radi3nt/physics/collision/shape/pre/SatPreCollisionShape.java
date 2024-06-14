package fr.radi3nt.physics.collision.shape.pre;

import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.physics.collision.detection.broad.aabb.CachingAABB;
import fr.radi3nt.physics.collision.shape.sat.shape.SatShapeObject;

public class SatPreCollisionShape implements PreCollisionShape {

    private final SatShapeObject satShapeObject;
    private final Vector3f offset;

    private final CachingAABB cachingAABB;

    public SatPreCollisionShape(SatShapeObject satShapeObject, Vector3f offset) {
        this.satShapeObject = satShapeObject;
        this.offset = offset;
        cachingAABB = satShapeObject.createCachingAABB();
    }

    @Override
    public CachingAABB getAABB() {
        return cachingAABB;
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
