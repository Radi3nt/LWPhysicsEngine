package fr.radi3nt.physics.collision.shape.shapes;

import fr.radi3nt.maths.components.advanced.quaternions.Quaternion;
import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.maths.components.vectors.implementations.SimpleVector3f;
import fr.radi3nt.physics.collision.detection.broad.aabb.aabb.AABB;
import fr.radi3nt.physics.collision.detection.broad.aabb.aabb.OBBDynamicAABB;
import fr.radi3nt.physics.collision.detection.broad.pre.PreCollisionShape;
import fr.radi3nt.physics.collision.detection.broad.sphere.BoundingSphere;
import fr.radi3nt.physics.collision.detection.broad.sphere.SetBoundingSphere;
import fr.radi3nt.physics.core.TransformedObject;

public class CapsuleShape extends TransformedShape implements PreCollisionShape {

    protected float height;
    protected float radius;

    public CapsuleShape(Vector3f offset, Quaternion rotation, float height, float radius) {
        super(offset, rotation);
        this.height = height;
        this.radius = radius;
    }

    public CapsuleShape(float height, float radius) {
        this.height = height;
        this.radius = radius;
    }

    public float getHeight() {
        return height;
    }

    public float getRadius() {
        return radius;
    }

    @Override
    public AABB getBoundingBox(TransformedObject object) {
        return new OBBDynamicAABB(object, new SimpleVector3f(radius, radius*2+height, radius), offset, rotation);
    }

    @Override
    public BoundingSphere getBoundingSphere(TransformedObject object) {
        return SetBoundingSphere.from(object, offset, height+radius*2);
    }

    @Override
    public final boolean equals(Object o) {
        if (this == o) return true;
        if (!(o instanceof CapsuleShape)) return false;
        if (!super.equals(o)) return false;

        CapsuleShape that = (CapsuleShape) o;
        return Float.compare(height, that.height) == 0 && Float.compare(radius, that.radius) == 0;
    }

    @Override
    public int hashCode() {
        int result = super.hashCode();
        result = 31 * result + Float.hashCode(height);
        result = 31 * result + Float.hashCode(radius);
        return result;
    }
}
