package fr.radi3nt.physics.collision.contact.cache;

import fr.radi3nt.physics.collision.contact.GeneratedContactPair;
import fr.radi3nt.physics.collision.contact.manifold.PersistentManifold;
import fr.radi3nt.physics.core.state.RigidBody;

public interface PersistentManifoldCache {

    PersistentManifold getCachedManifold(GeneratedContactPair<RigidBody> contactPair);
    PersistentManifold newManifold(GeneratedContactPair<RigidBody> contactPair);
    void releaseManifold(GeneratedContactPair<RigidBody> pair);

}
