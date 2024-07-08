package fr.radi3nt.physics.collision.detection.narrow.dispacher;

import fr.radi3nt.physics.collision.contact.GeneratedContactPair;
import fr.radi3nt.physics.collision.contact.cache.PersistentManifoldCache;
import fr.radi3nt.physics.collision.contact.manifold.PersistentManifold;

public interface CollisionDispatcher {

    PersistentManifold dispatch(PersistentManifoldCache manifoldCache, GeneratedContactPair pair, long currentStep);

}
