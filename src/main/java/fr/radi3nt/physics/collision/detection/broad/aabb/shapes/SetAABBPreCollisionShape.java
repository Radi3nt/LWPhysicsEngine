package fr.radi3nt.physics.collision.detection.broad.aabb.shapes;

import fr.radi3nt.physics.collision.detection.broad.aabb.aabb.AABB;
import fr.radi3nt.physics.collision.detection.broad.pre.PreCollisionShape;
import fr.radi3nt.physics.collision.detection.broad.sphere.BoundingSphere;
import fr.radi3nt.physics.core.TransformedObject;

public class SetAABBPreCollisionShape implements PreCollisionShape {

    private final AABB aabb;
    private final BoundingSphere boundingSphere;

    public SetAABBPreCollisionShape(AABB aabb, BoundingSphere boundingSphere) {
        this.aabb = aabb;
        this.boundingSphere = boundingSphere;
    }

    @Override
    public AABB getBoundingBox(TransformedObject object) {
        return aabb;
    }

    @Override
    public BoundingSphere getBoundingSphere(TransformedObject object) {
        return boundingSphere;
    }
}
