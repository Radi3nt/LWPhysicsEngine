package fr.radi3nt.physics.collision.detection.narrow.sat.computer;

import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.physics.collision.contact.manifold.ManifoldPoint;

import java.util.List;

public interface CollisionResult {

    boolean isCollision();

    float getOverlap();
    Vector3f getWorldSpaceNormal();

    List<ManifoldPoint> createPoints();

}
