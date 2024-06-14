package fr.radi3nt.physics.collision.shape.pre;

import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.physics.collision.detection.broad.aabb.CachingAABB;

public class AABBPreCollisionShape implements PreCollisionShape {

    private final CachingAABB aabb;
    private final Vector3f sphereOffset;
    private final float radius;

    public AABBPreCollisionShape(CachingAABB aabb, Vector3f sphereOffset, float radius) {
        this.aabb = aabb;
        this.sphereOffset = sphereOffset;
        this.radius = radius;
    }

    @Override
    public float getRadius() {
        return radius;
    }

    @Override
    public Vector3f getSphereOffset() {
        return sphereOffset;
    }

    @Override
    public CachingAABB getAABB() {
        return aabb;
    }

}
