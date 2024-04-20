package fr.radi3nt.physics.collision.detection.narrow.dispacher;

import fr.radi3nt.physics.collision.contact.ContactPair;
import fr.radi3nt.physics.collision.contact.cache.PersistentManifoldCache;
import fr.radi3nt.physics.collision.contact.manifold.PersistentManifold;
import fr.radi3nt.physics.collision.detection.narrow.NarrowPhaseDetectionAlgorithm;
import fr.radi3nt.physics.collision.response.CollisionContactSolver;

import java.util.Optional;

public class SetCollisionDispatcher implements CollisionDispatcher {

    private final NarrowPhaseDetectionAlgorithm detectionAlgorithm;

    public SetCollisionDispatcher(NarrowPhaseDetectionAlgorithm detectionAlgorithm) {
        this.detectionAlgorithm = detectionAlgorithm;
    }

    @Override
    public Optional<PersistentManifold> dispatch(PersistentManifoldCache manifoldCache, ContactPair pair) {
        return detectionAlgorithm.buildManifolds(manifoldCache, pair);
    }
}
