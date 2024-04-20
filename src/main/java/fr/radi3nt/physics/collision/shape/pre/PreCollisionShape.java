package fr.radi3nt.physics.collision.shape.pre;

import fr.radi3nt.maths.aabb.AABB;
import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.physics.collision.detection.broad.aabb.CachingAABB;

public interface PreCollisionShape {

    float getRadius();
    Vector3f getSphereOffset();

    CachingAABB getAABB();
    AABB getLocalAABB();

}
