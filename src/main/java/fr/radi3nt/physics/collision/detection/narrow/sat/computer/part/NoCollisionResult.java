package fr.radi3nt.physics.collision.detection.narrow.sat.computer.part;

import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.physics.collision.contact.manifold.ManifoldPoint;
import fr.radi3nt.physics.collision.detection.narrow.sat.computer.CollisionResult;

import java.util.Collections;
import java.util.List;

public class NoCollisionResult implements CollisionResult {

    public static final NoCollisionResult INSTANCE = new NoCollisionResult();

    private NoCollisionResult() {
    }

    @Override
    public boolean isCollision() {
        return false;
    }

    @Override
    public float getOverlap() {
        return 0;
    }

    @Override
    public Vector3f getWorldSpaceNormal() {
        return null;
    }

    @Override
    public List<ManifoldPoint> createPoints() {
        return Collections.EMPTY_LIST;
    }
}
