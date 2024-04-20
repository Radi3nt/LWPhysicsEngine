package fr.radi3nt.physics.collision.detection.narrow;

import fr.radi3nt.physics.collision.contact.ContactPair;
import fr.radi3nt.physics.collision.contact.cache.PersistentManifoldCache;
import fr.radi3nt.physics.collision.contact.manifold.PersistentManifold;

import java.util.Optional;

public interface NarrowPhaseDetectionAlgorithm {

    Optional<PersistentManifold> buildManifolds(PersistentManifoldCache manifoldCache, ContactPair pairs);

}
