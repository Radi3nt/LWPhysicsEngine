package fr.radi3nt.physics.collision.detection.narrow.dispacher;

import fr.radi3nt.physics.collision.contact.GeneratedContactPair;
import fr.radi3nt.physics.collision.contact.cache.PersistentManifoldCache;
import fr.radi3nt.physics.collision.contact.manifold.ManifoldPoint;
import fr.radi3nt.physics.collision.contact.manifold.PersistentManifold;
import fr.radi3nt.physics.collision.detection.narrow.NarrowPhaseDetectionAlgorithm;
import fr.radi3nt.physics.core.state.RigidBody;

import java.util.List;

public class SetCollisionDispatcher implements CollisionDispatcher {

    private final NarrowPhaseDetectionAlgorithm detectionAlgorithm;

    public SetCollisionDispatcher(NarrowPhaseDetectionAlgorithm detectionAlgorithm) {
        this.detectionAlgorithm = detectionAlgorithm;
    }

    @Override
    public PersistentManifold dispatch(PersistentManifoldCache manifoldCache, GeneratedContactPair<RigidBody> pair, long currentStep) {
        return detectionAlgorithm.buildManifolds(manifoldCache, pair, currentStep);
    }

    @Override
    public List<ManifoldPoint> dispatch(GeneratedContactPair<?> pair) {
        return detectionAlgorithm.buildManifoldPoints(pair);
    }
}
