package fr.radi3nt.physics.collision.detection.narrow.dispacher;

import fr.radi3nt.physics.collision.contact.GeneratedContactPair;
import fr.radi3nt.physics.collision.contact.cache.PersistentManifoldCache;
import fr.radi3nt.physics.collision.contact.manifold.PersistentManifold;
import fr.radi3nt.physics.collision.detection.narrow.NarrowPhaseDetectionAlgorithm;
import fr.radi3nt.physics.collision.shape.shapes.CollisionShape;

public class SupportedCollisionDispatcher implements CollisionDispatcher {

    private final NarrowPhaseDetectionAlgorithm[] algorithms;

    public SupportedCollisionDispatcher(NarrowPhaseDetectionAlgorithm... algorithms) {
        this.algorithms = algorithms;
    }

    @Override
    public PersistentManifold dispatch(PersistentManifoldCache manifoldCache, GeneratedContactPair pair, long currentStep) {

        CollisionShape shapeA = pair.shapeA;
        CollisionShape shapeB = pair.shapeB;

        for (NarrowPhaseDetectionAlgorithm algorithm : algorithms) {
            if (algorithm.isSupported(shapeA) && algorithm.isSupported(shapeB))
                return algorithm.buildManifolds(manifoldCache, pair, currentStep);
        }

        return null;
    }
}
