package fr.radi3nt.physics.collision.contact.cache;

import fr.radi3nt.physics.core.state.RigidBody;
import fr.radi3nt.physics.dynamics.island.RigidBodyIsland;

public interface ContactPairCacheProvider {

    ContactPairCache<RigidBody> newFilledCache(RigidBodyIsland island);

}
