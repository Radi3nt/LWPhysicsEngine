package fr.radi3nt.physics.collision.detection.broad;

import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.maths.components.vectors.implementations.SimpleVector3f;
import fr.radi3nt.physics.collision.shape.pre.PreCollisionPair;
import fr.radi3nt.physics.collision.shape.pre.PreCollisionShape;
import fr.radi3nt.physics.core.TransformedObject;

public class SphereBroadPhaseDetectionStrategy implements BroadPhaseDetectionStrategy {

    private final Vector3f cache = new SimpleVector3f();

    @Override
    public boolean canSkip(PreCollisionPair preCollisionPair) {
        return canSkip(preCollisionPair.getShapeA(), preCollisionPair.getShapeB(), preCollisionPair.getBodyA(), preCollisionPair.getBodyB());
    }

    private boolean canSkip(PreCollisionShape shapeA, PreCollisionShape shapeB, TransformedObject bodyA, TransformedObject bodyB) {
        float radiusA = shapeA.getRadius();
        float radiusB = shapeB.getRadius();

        cache.copy(bodyA.getPosition());
        float dist = cache.add(shapeA.getSphereOffset()).sub(bodyB.getPosition()).sub(shapeB.getSphereOffset()).length();

        float sumDist = radiusA + radiusB;
        return dist > sumDist * sumDist;
    }

}
