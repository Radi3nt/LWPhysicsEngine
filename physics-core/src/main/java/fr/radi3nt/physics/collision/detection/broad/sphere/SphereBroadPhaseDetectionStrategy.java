package fr.radi3nt.physics.collision.detection.broad.sphere;

import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.maths.components.vectors.implementations.SimpleVector3f;
import fr.radi3nt.physics.collision.detection.broad.BroadPhaseDetectionStrategy;
import fr.radi3nt.physics.collision.detection.broad.pre.PreCollisionPair;
import fr.radi3nt.physics.collision.shape.pre.PreCollisionData;

public class SphereBroadPhaseDetectionStrategy implements BroadPhaseDetectionStrategy {

    private final Vector3f cache = new SimpleVector3f();

    @Override
    public boolean canSkip(PreCollisionPair preCollisionPair) {
        return canSkip(preCollisionPair.getShapeA(), preCollisionPair.getShapeB());
    }

    private boolean canSkip(PreCollisionData shapeA, PreCollisionData shapeB) {
        BoundingSphere sphereA = shapeA.getBoundingSphere();
        BoundingSphere sphereB = shapeB.getBoundingSphere();
        Vector3f shapeAPos = sphereA.getPosition();
        Vector3f shapeBPos = sphereB.getPosition();

        float radiusA = sphereA.getRadius();
        float radiusB = sphereB.getRadius();

        cache.copy(shapeAPos);
        float dist = cache.sub(shapeBPos).length();

        float sumDist = radiusA + radiusB;
        return dist > sumDist;
    }

}
