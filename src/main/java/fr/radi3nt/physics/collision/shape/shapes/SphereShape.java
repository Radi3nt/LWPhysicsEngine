package fr.radi3nt.physics.collision.shape.shapes;

import fr.radi3nt.maths.components.advanced.quaternions.ComponentsQuaternion;
import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.maths.components.vectors.implementations.SimpleVector3f;
import fr.radi3nt.physics.collision.detection.broad.aabb.aabb.AABB;
import fr.radi3nt.physics.collision.detection.broad.aabb.aabb.OBBDynamicAABB;
import fr.radi3nt.physics.collision.detection.broad.pre.PreCollisionShape;
import fr.radi3nt.physics.collision.detection.broad.sphere.BoundingSphere;
import fr.radi3nt.physics.collision.detection.broad.sphere.SetBoundingSphere;
import fr.radi3nt.physics.core.TransformedObject;

public class SphereShape implements CollisionShape, PreCollisionShape {

    private final Vector3f centerOffset;
    private final float radius;

    public SphereShape(Vector3f centerOffset, float radius) {
        this.centerOffset = centerOffset;
        this.radius = radius;
    }

    public SphereShape(float radius) {
        this(new SimpleVector3f(), radius);
    }

    @Override
    public AABB getBoundingBox(TransformedObject object) {
        return new OBBDynamicAABB(object, new SimpleVector3f(radius, radius, radius), centerOffset, ComponentsQuaternion.zero());
    }

    @Override
    public BoundingSphere getBoundingSphere(TransformedObject object) {
        return SetBoundingSphere.from(object, centerOffset, radius);
    }

    public float getRadius() {
        return radius;
    }

    public Vector3f getOffset() {
        return centerOffset;
    }
}
