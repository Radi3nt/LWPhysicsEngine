package fr.radi3nt.physics.collision.contact.cache;

import fr.radi3nt.physics.collision.contact.ContactPair;
import fr.radi3nt.physics.collision.contact.manifold.PersistentManifold;

import java.util.Optional;

public interface PersistentManifoldCache {

    Optional<PersistentManifold> getCachedManifold(ContactPair contactPair);
    PersistentManifold newManifold(ContactPair contactPair);
    void releaseManifold(ContactPair pair);

}
