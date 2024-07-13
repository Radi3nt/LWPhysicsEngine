package fr.radi3nt.physics.collision.detection.broad.aabb.shapes;

import fr.radi3nt.physics.collision.detection.broad.aabb.aabb.AABB;
import fr.radi3nt.physics.collision.detection.broad.aabb.aabb.ExpendingAABB;
import fr.radi3nt.physics.collision.detection.broad.pre.PreCollisionShape;
import fr.radi3nt.physics.collision.detection.broad.sphere.BoundingSphere;
import fr.radi3nt.physics.collision.detection.broad.sphere.SetBoundingSphere;
import fr.radi3nt.physics.core.TransformedObject;

public class ParentAABBPreCollisionShape implements PreCollisionShape {

    private final PreCollisionShape child;
    private final float added;

    public ParentAABBPreCollisionShape(PreCollisionShape child, float added) {
        this.child = child;
        this.added = added;
    }

    @Override
    public AABB getBoundingBox(TransformedObject object) {
        AABB boundingBox = child.getBoundingBox(object);
        return new ExpendingAABB(boundingBox, added);
    }

    @Override
    public BoundingSphere getBoundingSphere(TransformedObject object) {
        BoundingSphere boundingSphere = child.getBoundingSphere(object);
        return new SetBoundingSphere(boundingSphere.getPosition(), boundingSphere.getRadius()+added*2f);
    }
}
