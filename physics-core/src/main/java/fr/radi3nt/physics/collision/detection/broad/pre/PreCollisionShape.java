package fr.radi3nt.physics.collision.detection.broad.pre;

import fr.radi3nt.physics.collision.detection.broad.aabb.aabb.AABB;
import fr.radi3nt.physics.collision.detection.broad.sphere.BoundingSphere;
import fr.radi3nt.physics.core.TransformedObject;

public interface PreCollisionShape {

    AABB getBoundingBox(TransformedObject object);
    BoundingSphere getBoundingSphere(TransformedObject object);

}
