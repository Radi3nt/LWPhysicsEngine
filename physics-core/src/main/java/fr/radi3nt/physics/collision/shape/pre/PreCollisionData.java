package fr.radi3nt.physics.collision.shape.pre;

import fr.radi3nt.physics.collision.detection.broad.aabb.aabb.AABB;
import fr.radi3nt.physics.collision.detection.broad.pre.PreCollisionShape;
import fr.radi3nt.physics.collision.detection.broad.sphere.BoundingSphere;
import fr.radi3nt.physics.core.TransformedObject;

public class PreCollisionData {

    private final PreCollisionShape shape;
    private final TransformedObject object;

    private AABB aabb;
    private BoundingSphere boundingSphere;

    public PreCollisionData(PreCollisionShape shape, TransformedObject object) {
        this.shape = shape;
        this.object = object;
    }

    public AABB getBoundingBox() {
        if (aabb == null) {
            aabb = shape.getBoundingBox(object);
        }
        return aabb;
    }

    public BoundingSphere getBoundingSphere() {
        if (boundingSphere == null) {
            boundingSphere = shape.getBoundingSphere(object);
        }
        return boundingSphere;
    }
}
