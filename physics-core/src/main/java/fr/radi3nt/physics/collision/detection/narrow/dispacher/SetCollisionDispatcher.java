package fr.radi3nt.physics.collision.detection.narrow.dispacher;

import fr.radi3nt.physics.collision.contact.GeneratedContactPair;
import fr.radi3nt.physics.collision.contact.cache.PersistentManifoldCache;
import fr.radi3nt.physics.collision.contact.manifold.PersistentManifold;
import fr.radi3nt.physics.collision.detection.narrow.NarrowPhaseDetectionAlgorithm;

public class SetCollisionDispatcher implements CollisionDispatcher {

    private final NarrowPhaseDetectionAlgorithm detectionAlgorithm;

    public SetCollisionDispatcher(NarrowPhaseDetectionAlgorithm detectionAlgorithm) {
        this.detectionAlgorithm = detectionAlgorithm;
    }

    @Override
    public PersistentManifold dispatch(PersistentManifoldCache manifoldCache, GeneratedContactPair pair, long currentStep) {
        return detectionAlgorithm.buildManifolds(manifoldCache, pair, currentStep);
    }
}
