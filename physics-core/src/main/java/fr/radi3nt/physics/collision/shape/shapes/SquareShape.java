package fr.radi3nt.physics.collision.shape.shapes;

import fr.radi3nt.maths.components.advanced.quaternions.Quaternion;
import fr.radi3nt.maths.components.vectors.Vector2f;
import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.maths.components.vectors.implementations.SimpleVector3f;
import fr.radi3nt.physics.collision.detection.broad.aabb.aabb.AABB;
import fr.radi3nt.physics.collision.detection.broad.aabb.aabb.OBBDynamicAABB;
import fr.radi3nt.physics.collision.detection.broad.pre.PreCollisionShape;
import fr.radi3nt.physics.collision.detection.broad.sphere.BoundingSphere;
import fr.radi3nt.physics.collision.detection.broad.sphere.SetBoundingSphere;
import fr.radi3nt.physics.core.TransformedObject;

public class SquareShape extends TransformedShape implements PreCollisionShape {

    private final Vector2f size;
    private final float radius;

    public SquareShape(Vector3f offset, Quaternion rotation, Vector2f size) {
        super(offset, rotation);
        this.size = size;
        radius = size.length();
    }

    public SquareShape(Vector2f size) {
        this.size = size;
        radius = size.length();
    }

    public Vector2f getSize() {
        return size;
    }

    @Override
    public AABB getBoundingBox(TransformedObject object) {
        return new OBBDynamicAABB(object, new SimpleVector3f(size.getX(), size.getY(), 0), offset, rotation);
    }

    @Override
    public BoundingSphere getBoundingSphere(TransformedObject object) {
        return new SetBoundingSphere(object.toWorldSpace(offset), radius);
    }
}
