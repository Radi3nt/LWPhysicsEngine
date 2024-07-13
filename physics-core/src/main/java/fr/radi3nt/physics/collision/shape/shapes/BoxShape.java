package fr.radi3nt.physics.collision.shape.shapes;

import fr.radi3nt.maths.components.advanced.quaternions.Quaternion;
import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.physics.collision.detection.broad.aabb.aabb.AABB;
import fr.radi3nt.physics.collision.detection.broad.aabb.aabb.OBBDynamicAABB;
import fr.radi3nt.physics.collision.detection.broad.pre.PreCollisionShape;
import fr.radi3nt.physics.collision.detection.broad.sphere.BoundingSphere;
import fr.radi3nt.physics.collision.detection.broad.sphere.SetBoundingSphere;
import fr.radi3nt.physics.core.TransformedObject;

import java.util.Objects;

public class BoxShape extends TransformedShape implements PreCollisionShape {

    private final Vector3f size;

    public BoxShape(Vector3f offset, Quaternion rotation, Vector3f size) {
        super(offset, rotation);
        this.size = size;
    }

    public BoxShape(Vector3f size) {
        this.size = size;
    }

    public Vector3f getSize() {
        return size;
    }

    @Override
    public AABB getBoundingBox(TransformedObject object) {
        return new OBBDynamicAABB(object, size, offset, rotation);
    }

    @Override
    public BoundingSphere getBoundingSphere(TransformedObject object) {
        return SetBoundingSphere.from(object, offset, size.length());
    }

    @Override
    public final boolean equals(Object o) {
        if (this == o) return true;
        if (!(o instanceof BoxShape)) return false;
        if (!super.equals(o)) return false;

        BoxShape boxShape = (BoxShape) o;
        return Objects.equals(size, boxShape.size);
    }

    @Override
    public int hashCode() {
        int result = super.hashCode();
        result = 31 * result + Objects.hashCode(size);
        return result;
    }
}
