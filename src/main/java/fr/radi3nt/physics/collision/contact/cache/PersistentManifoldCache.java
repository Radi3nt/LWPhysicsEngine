package fr.radi3nt.physics.collision.contact.cache;

import fr.radi3nt.physics.collision.contact.GeneratedContactPair;
import fr.radi3nt.physics.collision.contact.manifold.PersistentManifold;

import java.util.Optional;

public interface PersistentManifoldCache {

    PersistentManifold getCachedManifold(GeneratedContactPair contactPair);
    PersistentManifold newManifold(GeneratedContactPair contactPair);
    void releaseManifold(GeneratedContactPair pair);

}
